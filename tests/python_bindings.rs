//! Runs the Python test suite in `tests/python` and the Python example in
//! `examples/python` against the Python bindings.
//!
//! The cdylib that cargo built for this test run is copied, under the file
//! name of the extension module, into a temporary directory and imported from
//! there. Every Python test case runs in its own interpreter process, so a
//! crash fails only that case.
//!
//! The renderer cases need a GPU (a software Vulkan one such as lavapipe
//! works).

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

/// Name of the cdylib and of the Python module it defines.
const MODULE: &str = "r3v_renderer";

const TEST_SCRIPT: &str = "tests/python/renderer_tests.py";

const EXAMPLE: &str = "examples/python/voxels_to_ppm.py";

/// File name suffix under which Python imports an extension module.
const MODULE_SUFFIX: &str = if cfg!(windows) { ".pyd" } else { ".so" };

/// The importable copy of the module, the interpreter that runs the tests and
/// the directory the tests write files to.
struct PythonTests {
    python: OsString,
    module_dir: PathBuf,
    tmp_dir: PathBuf,
}

impl PythonTests {
    fn setup() -> Self {
        let module_dir = native::temp_path("python");
        let tmp_dir = native::temp_path("python_files");
        for dir in [&module_dir, &tmp_dir] {
            fs::create_dir_all(dir)
                .unwrap_or_else(|e| panic!("failed to create {}: {e}", dir.display()));
        }
        let lib = native::lib_path();
        let module = module_dir.join(format!("{MODULE}{MODULE_SUFFIX}"));
        fs::copy(&lib, &module).unwrap_or_else(|e| {
            panic!(
                "failed to copy {} to {}: {e}",
                lib.display(),
                module.display()
            )
        });
        Self {
            python: python(),
            module_dir,
            tmp_dir,
        }
    }

    /// A command running the Python script `script` with the module
    /// importable.
    fn command(&self, script: &str) -> Command {
        let mut paths = vec![self.module_dir.clone()];
        if let Some(existing) = env::var_os("PYTHONPATH") {
            paths.extend(env::split_paths(&existing));
        }
        let joined: OsString = env::join_paths(paths).expect("PYTHONPATH is joinable");
        let mut command = Command::new(&self.python);
        // -B: no __pycache__ inside tests/python.
        command
            .arg("-B")
            .arg(native::root().join(script))
            .env("PYTHONPATH", joined)
            .env("R3V_TEST_TMPDIR", &self.tmp_dir);
        command
    }

    fn list_cases(&self) -> Vec<String> {
        let output = self
            .command(TEST_SCRIPT)
            .arg("list")
            .output()
            .unwrap_or_else(|e| {
                panic!(
                    "failed to run Python interpreter `{}`: {e}",
                    self.python.to_string_lossy()
                )
            });
        assert!(
            output.status.success(),
            "listing Python tests failed ({}):\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8(output.stdout)
            .expect("Python test names are UTF-8")
            .lines()
            .map(str::to_owned)
            .collect()
    }

    fn run_case(&self, name: &str) -> Result<(), Failed> {
        Ok(native::run(self.command(TEST_SCRIPT).args(["run", name]))?)
    }

    fn run_example(&self) -> Result<(), Failed> {
        let image = self.tmp_dir.join("example.ppm");
        native::run(self.command(EXAMPLE).arg(&image))?;
        let ppm = fs::read(&image).map_err(|e| format!("example wrote no image: {e}"))?;
        if !ppm.starts_with(b"P6\n") {
            return Err("example image is not a binary PPM".into());
        }
        Ok(())
    }
}

/// The interpreter to run the tests with, found the way PyO3 finds the one
/// it builds for: `PYO3_PYTHON` when set, otherwise `python3` from `PATH`
/// (which is the active virtualenv's, if any).
fn python() -> OsString {
    env::var_os("PYO3_PYTHON")
        .unwrap_or_else(|| if cfg!(windows) { "python" } else { "python3" }.into())
}

fn remove_dir(dir: &Path) {
    let _ = fs::remove_dir_all(dir);
}

fn main() {
    let args = Arguments::from_args();
    let python_tests = Arc::new(PythonTests::setup());
    let mut trials: Vec<Trial> = python_tests
        .list_cases()
        .into_iter()
        .map(|name| {
            let python_tests = python_tests.clone();
            Trial::test(name.clone(), move || python_tests.run_case(&name))
        })
        .collect();
    let example = python_tests.clone();
    trials.push(Trial::test("example::voxels_to_ppm", move || {
        example.run_example()
    }));
    let conclusion = libtest_mimic::run(&args, trials);
    remove_dir(&python_tests.module_dir);
    remove_dir(&python_tests.tmp_dir);
    conclusion.exit();
}
