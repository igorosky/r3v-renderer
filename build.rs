// Only the bindings module: the rest of the crate has public items (e.g. the
// `Axis` enum) that are not part of the C API
#[cfg(any(feature = "c", feature = "cpp"))]
const BINDINGS_SRC: &str = "src/bindings/c/mod.rs";

#[cfg(any(feature = "c", feature = "cpp"))]
fn header_config(language: cbindgen::Language) -> cbindgen::Config {
    cbindgen::Config {
        language,
        braces: cbindgen::Braces::SameLine,
        usize_is_size_t: true,
        ..Default::default()
    }
}

#[cfg(feature = "c")]
fn gen_c() {
    let config = cbindgen::Config {
        include_guard: Some("R3V_RENDERER_GUARD".to_owned()),
        ..header_config(cbindgen::Language::C)
    };
    cbindgen::Builder::new()
        .with_src(BINDINGS_SRC)
        .with_config(config)
        .generate()
        .expect("Failed to generate C bindings")
        .write_to_file("bindings/r3v-renderer.h");
}

#[cfg(feature = "cpp")]
fn gen_cpp() {
    let config = cbindgen::Config {
        pragma_once: true,
        namespaces: Some(vec!["r3v".to_owned(), "renderer".to_owned()]),
        ..header_config(cbindgen::Language::Cxx)
    };
    cbindgen::Builder::new()
        .with_src(BINDINGS_SRC)
        .with_config(config)
        .generate()
        .expect("Failed to generate C++ bindings")
        .write_to_file("bindings/r3v-renderer.hpp");
}

#[cfg(feature = "python")]
fn gen_python() {
    pyo3_build_config::add_extension_module_link_args();
}

fn main() {
    // A Python extension module leaves the Python symbols to the interpreter
    // loading it, so it is no library for C
    #[cfg(all(any(feature = "c", feature = "cpp"), feature = "python"))]
    compile_error!("Cannot build for C/C++ and Python at the same time");
    // The web has no threads for rayon
    #[cfg(all(feature = "js", feature = "multi-threaded"))]
    compile_error!("Cannot build for JS with multi-threaded feature");
    #[cfg(all(feature = "js", feature = "python"))]
    compile_error!("Cannot build for JS and Python at the same time");
    #[cfg(all(any(feature = "c", feature = "cpp"), feature = "js"))]
    compile_error!("Cannot build for C/C++ and JS at the same time");
    #[cfg(feature = "c")]
    gen_c();
    #[cfg(feature = "cpp")]
    gen_cpp();
    #[cfg(feature = "python")]
    gen_python();
}
