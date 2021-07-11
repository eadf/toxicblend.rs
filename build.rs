#![deny(warnings)]

extern crate version_check as rustc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tonic_build::configure()
        .build_client(false)
        .compile(&["proto/toxicblend.proto"], &["proto"]).unwrap();
    println!("cargo:rerun-if-changed=build.rs");
    if let Some(is_feature_flaggable) = rustc::is_feature_flaggable() {
        // enable the "hash_drain_filter" and "map_first_last" features if using +nightly
        if is_feature_flaggable {
            println!("cargo:rustc-cfg=feature=\"map_first_last\"");
            println!("cargo:rustc-cfg=feature=\"hash_drain_filter\"");
        }
    }
    Ok(())
}
