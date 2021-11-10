use tonic::{
    transport::Server, Request as PB_Request, Response as PB_Response, Status as PB_Status,
};
use toxicblend::toxicblend_pb::toxic_blend_service_server::{
    ToxicBlendService, ToxicBlendServiceServer,
};
use toxicblend::{execute_command, PB_Command, PB_KeyValuePair, PB_Reply};

#[derive(Debug, Default)]
pub struct ToxicBlendServiceImpl {}

#[tonic::async_trait]
impl ToxicBlendService for ToxicBlendServiceImpl {
    async fn execute(
        &self,
        request: PB_Request<PB_Command>,
    ) -> Result<PB_Response<PB_Reply>, PB_Status> {
        let rv = execute_command(request.into_inner(), true);

        // convert TBError to a valid Error message
        if let Err(err) = rv {
            eprintln!("Detected error {:?}", err);
            Ok(PB_Response::new(PB_Reply {
                options: vec![PB_KeyValuePair {
                    key: "ERROR".to_string(),
                    value: format!("{:?}", err),
                }],
                models: Vec::with_capacity(0),
                models32: Vec::with_capacity(0),
            }))
        } else {
            // unwrap is safe now
            Ok(PB_Response::new(rv.unwrap()))
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let address = "[::1]:50069".parse()?;
    let service = ToxicBlendServiceImpl::default();

    println!(
        "Toxicblend server starting, will listen for connections @{:?}",
        address
    );
    Server::builder()
        .add_service(ToxicBlendServiceServer::new(service))
        .serve(address)
        .await?;

    Ok(())
}
