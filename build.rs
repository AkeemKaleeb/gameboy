fn main() {
    println!("cargo:rustc-link-search=native=C:/SDL2/lib/x64");
    println!("cargo:rustc-link-lib=dylib=SDL2");
}