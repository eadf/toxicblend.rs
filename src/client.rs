use toxicblend::{toxic_blend_service_client::ToxicBlendServiceClient, KeyValuePair, Model};

pub mod toxicblend {
    tonic::include_proto!("toxicblend");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = ToxicBlendServiceClient::connect("http://[::1]:50069").await?;

    let request = tonic::Request::new(toxicblend::Command {
        command: "test".to_string(),
        options: vec![KeyValuePair {
            key: "request".to_string(),
            value: "somevalue".to_string(),
        }],
        models: Vec::<Model>::new(),
    });

    let response = client.execute(request).await?;

    println!("RESPONSE={:?}", response);

    Ok(())
}
