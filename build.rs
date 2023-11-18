#![deny(warnings)]

extern crate version_check as rustc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .build_client(false)
        .compile(&["proto/toxicblend.proto"], &["proto"])
        .unwrap();
    println!("cargo:rerun-if-changed=build.rs");
    Ok(())
}
