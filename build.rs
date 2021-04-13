fn main() -> Result<(), Box<dyn std::error::Error>> {
    //tonic_build::configure()
    //    .extern_path(".toxicblend.Model", "crate::protobuf::Model")
    //    .compile(&["proto/toxicblend.proto"], &["."])?;
    tonic_build::compile_protos("proto/toxicblend.proto")?;
    Ok(())
}
