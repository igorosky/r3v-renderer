//! Runs the JavaScript test suite in `tests/js` and the JavaScript example in
//! `examples/js` against the JS bindings.
//!
//! The host cdylib of this test run is of no use to JavaScript, so the crate
//! is built for `wasm32-unknown-unknown` with wasm-pack (the build the README
//! documents: `--target web`). Every JS test case runs in its own Node.js
//! process, so a trap (e.g. a Rust panic) fails only that case.
//!
//! Node.js has no WebGPU: the tests take it from the `webgpu` package (Dawn),
//! installed into `tests/js/node_modules` by `npm ci` in `tests/js`, which
//! needs a Vulkan or Metal device (a software one such as lavapipe works).
//!
//! Needs `wasm-pack` (or `WASM_PACK`), the `wasm32-unknown-unknown` target
//! (`rustup target add wasm32-unknown-unknown`) and `node` (or `NODE`).

mod native;

use std::{
    env,
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::Command,
    sync::Arc,
};

use libtest_mimic::{Arguments, Failed, Trial};

/// Name of the generated JS module (and of its `_bg.wasm` file).
const MODULE: &str = "r3v_renderer";

const TEST_SCRIPT: &str = "tests/js/renderer_tests.mjs";

const EXAMPLE: &str = "examples/js/voxels_to_ppm.mjs";

/// Gives Node.js the WebGPU of the `webgpu` package.
const WEBGPU_SETUP: &str = "tests/js/webgpu.mjs";

const WEBGPU_PACKAGE: &str = "tests/js/node_modules/webgpu";

/// The generated JS package, the Node.js that runs the tests and the
/// directory the example writes to.
struct JsTests {
    node: OsString,
    pkg_dir: PathBuf,
    tmp_dir: PathBuf,
}

impl JsTests {
    /// Builds the package into a directory unique to this process, so that
    /// concurrent test runs do not overwrite each other's package.
    fn setup() -> Self {
        let root = native::root();
        assert!(
            root.join(WEBGPU_PACKAGE).is_dir(),
            "{WEBGPU_PACKAGE} not found; install it with `npm ci` in tests/js"
        );
        let pkg_dir = native::temp_path("js");
        let tmp_dir = native::temp_path("js_files");
        fs::create_dir_all(&tmp_dir)
            .unwrap_or_else(|e| panic!("failed to create {}: {e}", tmp_dir.display()));
        let wasm_pack = env::var_os("WASM_PACK").unwrap_or_else(|| "wasm-pack".into());
        // A target directory of its own: a different target than the test
        // run, and kept between runs so the wasm build stays incremental.
        let output = Command::new(&wasm_pack)
            .args(["build", "--dev", "--target", "web", "--no-pack"])
            .args(["--no-typescript", "--out-name", MODULE, "--out-dir"])
            .arg(&pkg_dir)
            .arg(root)
            .args(["--", "--no-default-features", "--features", "js"])
            .env(
                "CARGO_TARGET_DIR",
                Path::new(env!("CARGO_TARGET_TMPDIR")).join("js-wasm-target"),
            )
            .output()
            .unwrap_or_else(|e| {
                panic!(
                    "failed to run `{}` (install wasm-pack or set WASM_PACK): {e}",
                    wasm_pack.to_string_lossy()
                )
            });
        assert!(
            output.status.success(),
            "building the wasm package failed ({}); it needs the \
             wasm32-unknown-unknown target:\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        Self {
            node: env::var_os("NODE").unwrap_or_else(|| "node".into()),
            pkg_dir,
            tmp_dir,
        }
    }

    /// A command running the JS script `script` against the built package.
    fn command(&self, script: &str) -> Command {
        let mut command = Command::new(&self.node);
        command
            .arg(native::root().join(script))
            .env("R3V_RENDERER_PKG", &self.pkg_dir);
        command
    }

    fn list_cases(&self) -> Vec<String> {
        let output = self
            .command(TEST_SCRIPT)
            .arg("list")
            .output()
            .unwrap_or_else(|e| {
                panic!(
                    "failed to run Node.js `{}` (install it or set NODE): {e}",
                    self.node.to_string_lossy()
                )
            });
        assert!(
            output.status.success(),
            "listing JS tests failed ({}):\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8(output.stdout)
            .expect("JS test names are UTF-8")
            .lines()
            .map(str::to_owned)
            .collect()
    }

    fn run_case(&self, name: &str) -> Result<(), Failed> {
        Ok(native::run(self.command(TEST_SCRIPT).args(["run", name]))?)
    }

    fn run_example(&self) -> Result<(), Failed> {
        let image = self.tmp_dir.join("example.ppm");
        let mut command = Command::new(&self.node);
        command
            .arg("--import")
            .arg(native::root().join(WEBGPU_SETUP))
            .arg(native::root().join(EXAMPLE))
            .arg(&image)
            .env("R3V_RENDERER_PKG", &self.pkg_dir);
        native::run(&mut command)?;
        let ppm = fs::read(&image).map_err(|e| format!("example wrote no image: {e}"))?;
        if !ppm.starts_with(b"P6\n") {
            return Err("example image is not a binary PPM".into());
        }
        Ok(())
    }
}

fn main() {
    let args = Arguments::from_args();
    let js_tests = Arc::new(JsTests::setup());
    let mut trials: Vec<Trial> = js_tests
        .list_cases()
        .into_iter()
        .map(|name| {
            let js_tests = js_tests.clone();
            Trial::test(name.clone(), move || js_tests.run_case(&name))
        })
        .collect();
    let example = js_tests.clone();
    trials.push(Trial::test("example::voxels_to_ppm", move || {
        example.run_example()
    }));
    let conclusion = libtest_mimic::run(&args, trials);
    let _ = fs::remove_dir_all(&js_tests.pkg_dir);
    let _ = fs::remove_dir_all(&js_tests.tmp_dir);
    conclusion.exit();
}
