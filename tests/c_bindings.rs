//! Runs the C test suite in `tests/c` and the C example in `examples/c`
//! against the C bindings.
//!
//! The C sources are compiled against `bindings/r3v-renderer.h` and the
//! cdylib that cargo built for this test run. Every C test case runs in its
//! own process, so a crash (e.g. a NULL dereference) fails only that case.
//!
//! The renderer cases need a GPU (a software Vulkan one such as lavapipe
//! works).

mod native;

use std::{fs, path::PathBuf, sync::Arc};

use libtest_mimic::{Arguments, Failed, Trial};

const C_SOURCES: [&str; 2] = ["tests/c/harness.c", "tests/c/renderer_tests.c"];

const EXAMPLE: &str = "examples/c/voxels_to_ppm.c";

/// The compiled C programs and the directory the tests write files to.
struct CTests {
    tests_exe: PathBuf,
    example_exe: PathBuf,
    tmp_dir: PathBuf,
}

impl CTests {
    fn build() -> Self {
        let compile = |sources: &[&str], exe: &PathBuf| {
            let sources: Vec<_> = sources.iter().map(|src| native::root().join(src)).collect();
            native::compile("CC", "cc", &["-std=c11"], &sources, exe);
        };
        let tests_exe = native::temp_exe("c_tests");
        compile(&C_SOURCES, &tests_exe);
        let example_exe = native::temp_exe("c_example");
        compile(&[EXAMPLE], &example_exe);
        let tmp_dir = native::temp_path("c_files");
        fs::create_dir_all(&tmp_dir)
            .unwrap_or_else(|e| panic!("failed to create {}: {e}", tmp_dir.display()));
        Self {
            tests_exe,
            example_exe,
            tmp_dir,
        }
    }

    fn list_cases(&self) -> Vec<String> {
        let output = native::command(&self.tests_exe)
            .arg("list")
            .output()
            .expect("failed to list C tests");
        assert!(
            output.status.success(),
            "listing C tests failed ({}):\n{}",
            output.status,
            String::from_utf8_lossy(&output.stderr)
        );
        String::from_utf8(output.stdout)
            .expect("C test names are UTF-8")
            .lines()
            .map(str::to_owned)
            .collect()
    }

    fn run_case(&self, name: &str) -> Result<(), Failed> {
        Ok(native::run(
            native::command(&self.tests_exe)
                .args(["run", name])
                .env("R3V_TEST_TMPDIR", &self.tmp_dir),
        )?)
    }

    fn run_example(&self) -> Result<(), Failed> {
        let image = self.tmp_dir.join("example.ppm");
        native::run(native::command(&self.example_exe).arg(&image))?;
        let ppm = fs::read(&image).map_err(|e| format!("example wrote no image: {e}"))?;
        if !ppm.starts_with(b"P6\n") {
            return Err("example image is not a binary PPM".into());
        }
        Ok(())
    }
}

fn main() {
    let args = Arguments::from_args();
    let c_tests = Arc::new(CTests::build());
    let mut trials: Vec<Trial> = c_tests
        .list_cases()
        .into_iter()
        .map(|name| {
            let c_tests = c_tests.clone();
            Trial::test(name.clone(), move || c_tests.run_case(&name))
        })
        .collect();
    let example = c_tests.clone();
    trials.push(Trial::test("example::voxels_to_ppm", move || {
        example.run_example()
    }));
    let conclusion = libtest_mimic::run(&args, trials);
    let _ = fs::remove_file(&c_tests.tests_exe);
    let _ = fs::remove_file(&c_tests.example_exe);
    let _ = fs::remove_dir_all(&c_tests.tmp_dir);
    conclusion.exit();
}
