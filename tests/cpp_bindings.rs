//! Compiles `tests/cpp/smoke.cpp` against `bindings/r3v-renderer.hpp` and
//! the cdylib of this test run, then runs it. Built with `cpp` alone this
//! catches a library that does not export what the header declares.
//!
//! The program renders a frame, so it needs a GPU (a software Vulkan one such
//! as lavapipe works).

mod native;

use std::fs;

use libtest_mimic::{Arguments, Trial};

fn main() {
    let args = Arguments::from_args();
    let exe = native::temp_exe("cpp_smoke");
    let trial = {
        let exe = exe.clone();
        Trial::test("smoke", move || {
            native::compile(
                "CXX",
                "c++",
                &["-std=c++17"],
                &[native::root().join("tests/cpp/smoke.cpp")],
                &exe,
            );
            Ok(native::run(&mut native::command(&exe))?)
        })
    };
    let conclusion = libtest_mimic::run(&args, vec![trial]);
    let _ = fs::remove_file(&exe);
    conclusion.exit();
}
