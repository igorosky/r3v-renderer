//! Shared parts of the binding test drivers: finding the cdylib of this test
//! run, compiling C and C++ programs against it and running programs.
#![allow(dead_code)]

use std::{
    env::{self, consts},
    ffi::OsString,
    path::{Path, PathBuf},
    process::Command,
    sync::OnceLock,
};

/// Variable the dynamic loader searches before the paths baked into a binary.
const LIB_PATH_VAR: &str = if cfg!(target_os = "macos") {
    "DYLD_LIBRARY_PATH"
} else {
    "LD_LIBRARY_PATH"
};

pub fn root() -> &'static Path {
    Path::new(env!("CARGO_MANIFEST_DIR"))
}

/// A path unique to this process, so that concurrent test runs do not
/// overwrite each other's files.
pub fn temp_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{name}-{}", std::process::id()))
}

/// [`temp_path`] for an executable.
pub fn temp_exe(name: &str) -> PathBuf {
    let mut path = temp_path(name).into_os_string();
    path.push(consts::EXE_SUFFIX);
    path.into()
}

/// The features of the crate this test run was built with, all of them, so
/// that the library is built with the same ones.
const FEATURES: [(&str, bool); 6] = [
    ("c", cfg!(feature = "c")),
    ("cpp", cfg!(feature = "cpp")),
    ("demo", cfg!(feature = "demo")),
    ("js", cfg!(feature = "js")),
    ("multi-threaded", cfg!(feature = "multi-threaded")),
    ("python", cfg!(feature = "python")),
];

/// The cdylib built with this test run's features.
///
/// Where cargo keeps the one it built for the test run is not a stable
/// interface: up to Rust 1.99 it was next to the test binary, in
/// `target/<profile>/deps`; since 1.100 (build-dir layout v2) every unit has
/// a directory of its own and nothing in the test run points to the
/// library's. Cargo's JSON messages are stable, so cargo is asked: building
/// the library again with the same features, profile and target directory
/// finds it fresh and reports the cdylib, copied to `target/<profile>`
/// (`cargo test` alone leaves that copy stale). The build inherits the
/// environment, so `RUSTFLAGS` (`./test.sh`) and the instrumentation of
/// cargo-llvm-cov match too. Should anything else differ (another profile, a
/// `--target`), cargo builds the library once more with the same features.
pub fn lib_path() -> &'static Path {
    static LIB: OnceLock<PathBuf> = OnceLock::new();
    LIB.get_or_init(build_lib)
}

fn build_lib() -> PathBuf {
    let lib_name = format!("{}r3v_renderer{}", consts::DLL_PREFIX, consts::DLL_SUFFIX);
    let features: Vec<_> = FEATURES
        .iter()
        .filter(|(_, enabled)| *enabled)
        .map(|(name, _)| *name)
        .collect();
    // `cargo test` builds with the test profile, `--release` with release
    let profile = if cfg!(debug_assertions) {
        "test"
    } else {
        "release"
    };
    // `<target dir>/tmp`. cargo-llvm-cov passes its own target directory on
    // the command line only, the environment does not have it.
    let target_dir = Path::new(env!("CARGO_TARGET_TMPDIR"))
        .parent()
        .expect("the target tmp directory is inside the target directory");
    let output = Command::new(env!("CARGO"))
        .args(["build", "--lib", "--frozen", "--no-default-features"])
        .args(["--message-format=json-render-diagnostics", "--profile", profile])
        .arg("--features")
        .arg(features.join(","))
        .arg("--manifest-path")
        .arg(root().join("Cargo.toml"))
        .arg("--target-dir")
        .arg(target_dir)
        .output()
        .expect("failed to run cargo");
    assert!(
        output.status.success(),
        "building {lib_name} failed ({}):\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
        .filter(|message| {
            message["reason"] == "compiler-artifact" && message["target"]["name"] == "r3v_renderer"
        })
        .flat_map(|message| message["filenames"].as_array().cloned().unwrap_or_default())
        .filter_map(|file| file.as_str().map(PathBuf::from))
        .find(|file| file.file_name() == Some(lib_name.as_ref()))
        .unwrap_or_else(|| panic!("cargo built no {lib_name}; build with a binding feature"))
}

fn lib_dir() -> PathBuf {
    lib_path()
        .parent()
        .expect("library has a parent directory")
        .to_path_buf()
}

/// Compiles `sources` into `exe` with the compiler from `compiler_var` (or
/// `default_compiler`), the headers in `bindings/` and the cdylib.
pub fn compile(
    compiler_var: &str,
    default_compiler: &str,
    flags: &[&str],
    sources: &[PathBuf],
    exe: &Path,
) {
    let compiler = env::var(compiler_var).unwrap_or_else(|_| default_compiler.to_owned());
    let output = Command::new(&compiler)
        .args(flags)
        .args(["-Wall", "-Wextra", "-Werror", "-g", "-I"])
        .arg(root().join("bindings"))
        .args(sources)
        .arg("-o")
        .arg(exe)
        .arg("-L")
        .arg(lib_dir())
        .args(["-lr3v_renderer", "-lm"])
        .output()
        .unwrap_or_else(|e| panic!("failed to run compiler `{compiler}`: {e}"));
    assert!(
        output.status.success(),
        "compiling {sources:?} failed ({}):\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );
}

/// A command running `exe` against [`lib_path`].
///
/// Its directory goes first in the library path: the one `cargo test` set
/// can hold other copies of the library, e.g. the stale one in
/// `target/<profile>` when [`lib_path`] is in another profile's directory (an
/// rpath cannot help: the environment variable takes precedence over it).
pub fn command(exe: &Path) -> Command {
    let mut paths = vec![lib_dir()];
    if let Some(existing) = env::var_os(LIB_PATH_VAR) {
        paths.extend(env::split_paths(&existing));
    }
    let joined: OsString = env::join_paths(paths).expect("library path is joinable");
    let mut command = Command::new(exe);
    command.env(LIB_PATH_VAR, joined);
    command
}

/// Runs `command`, an error with its stderr when it fails.
pub fn run(command: &mut Command) -> Result<(), String> {
    let output = command
        .output()
        .map_err(|e| format!("failed to start {command:?}: {e}"))?;
    if output.status.success() {
        Ok(())
    } else {
        Err(format!(
            "{}\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        ))
    }
}
