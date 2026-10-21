//! Shared parts of the binding test drivers: finding the cdylib of this test
//! run, compiling C and C++ programs against it and running programs.
#![allow(dead_code)]

use std::{
    env::{self, consts},
    ffi::OsString,
    path::{Path, PathBuf},
    process::Command,
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

/// The cdylib rustc wrote for this test run, in `target/<profile>/deps`. The
/// copy in `target/<profile>` is only refreshed by `cargo build`.
pub fn lib_path() -> PathBuf {
    let lib_name = format!("{}r3v_renderer{}", consts::DLL_PREFIX, consts::DLL_SUFFIX);
    let exe = env::current_exe().expect("test binary path");
    let lib = exe
        .parent()
        .expect("test binary has a parent directory")
        .join(&lib_name);
    assert!(
        lib.is_file(),
        "{lib_name} not found next to {}; build with a binding feature",
        exe.display()
    );
    lib
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

/// A command running `exe` against the cdylib of this test run.
///
/// `cargo test` puts `target/<profile>` before `target/<profile>/deps` in the
/// library path, and `target/<profile>/libr3v_renderer.so` is only refreshed
/// by `cargo build`. Without putting `deps` first the program could load a
/// stale copy (an rpath cannot help: the environment variable takes
/// precedence over it).
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
