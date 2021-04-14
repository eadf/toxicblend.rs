//use tonic::{transport::Server, Request, Response, Status};

use toxicblend::toxic_blend_service_client::ToxicBlendServiceClient;
use toxicblend::{ KeyValuePair, Model};

pub mod toxicblend {
    tonic::include_proto!("toxicblend");
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = ToxicBlendServiceClient::connect("http://[::1]:50051").await?;

    let request = tonic::Request::new(toxicblend::Command {
        command: "test".to_string(),
        options: vec![KeyValuePair {
            key: "request".to_string(),
            value: "skit?".to_string(),
        }],
        models: Vec::<Model>::new(),
        //message: format!("Hello {}!", request.into_inner().name).into()
    });

    let response = client.execute(request).await?;

    println!("RESPONSE={:?}", response);

    Ok(())
}
