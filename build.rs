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

fn main() {
    #[cfg(feature = "c")]
    gen_c();
    #[cfg(feature = "cpp")]
    gen_cpp();
}
