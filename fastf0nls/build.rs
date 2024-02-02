use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rustc-link-search=native={}", cmake::build(".").display());
    for x in ["fastf0nls", "CMSISDSP"] {
        println!("cargo:rustc-link-lib=static={}", x);
    }
    for x in ["CMakeLists.txt", "fastf0nls.c", "fastf0nls.h", "lookup.h"] {
        println!("cargo:rerun-if-changed={}", x);
    }
    let bindings = bindgen::Builder::default()
        .header("fastf0nls.h")
        .use_core()
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .unwrap();
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap()).join("bindings.rs");
    bindings.write_to_file(out_path).unwrap();
}
