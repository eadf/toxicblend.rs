#![feature(hash_drain_filter)]
//#![feature(hash_set_entry)]

mod cmd_2d_outline;
mod cmd_centerline;
mod cmd_knife_intersect;
mod cmd_simplify;

use crate::toxicblend::Command as PB_Command;
use crate::toxicblend::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend::Model as PB_Model;
use crate::toxicblend::Reply as PB_Reply;
use crate::toxicblend::Vertex as PB_Vertex;
use std::collections::HashMap;
use tonic::transport::Server;
use tonic::Request as PB_Request;
use tonic::Response as PB_Response;
use tonic::Status as PB_Status;
use toxicblend::toxic_blend_service_server::{ToxicBlendService, ToxicBlendServiceServer};

pub mod toxicblend {
    tonic::include_proto!("toxicblend");
}
impl From<cgmath::Point3<f64>> for PB_Vertex {
    fn from(other: cgmath::Point3<f64>) -> PB_Vertex {
        PB_Vertex {
            x: other.x,
            y: other.y,
            z: other.z,
        }
    }
}

impl PB_Vertex {
    pub fn distance_squared(&self, other: &PB_Vertex) -> f64 {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;
        x * x + y * y + z * z
    }
}

#[derive(thiserror::Error, Debug)]
pub enum TBError {
    #[error("Unknown command received: {0}")]
    UnknownCommand(String),

    #[error("Model should not contain any faces")]
    ModelContainsFaces,

    #[error("Something is wrong with the internal logic: {0}")]
    InternalError(String),

    #[error("Something is wrong with the input data")]
    CouldNotCalculateInverseMatrix,

    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData,

    #[error("The input data is not 2D")]
    InputNotPLane(String),

    #[error("Invalid input data: {0}")]
    InvalidInputData(String),

    #[error(transparent)]
    BvError(#[from] boostvoronoi::BvError),

    #[error(transparent)]
    IoError(#[from] std::io::Error),

    #[error(transparent)]
    LinestringError(#[from] linestring::LinestringError),

    #[error(transparent)]
    CenterLineError(#[from] centerline::CenterlineError),
}

#[derive(Debug, Default)]
pub struct TheToxicBlendService {}

/// convert the options to a hashmap
fn options_to_map(options: &[PB_KeyValuePair]) -> HashMap<String, String> {
    let mut rv = HashMap::<String, String>::new();
    for kv in options.iter() {
        rv.insert(kv.key.clone(), kv.value.clone());
    }
    rv
}

#[tonic::async_trait]
impl ToxicBlendService for TheToxicBlendService {
    async fn execute(
        &self,
        request: PB_Request<PB_Command>,
    ) -> Result<PB_Response<PB_Reply>, PB_Status> {
        let a_command = request.get_ref();
        let map = options_to_map(&a_command.options);

        let rv = match a_command.command.as_str() {
            "2d_outline" => cmd_2d_outline::command(a_command, map),
            "simplify" => cmd_simplify::command(a_command, map),
            "knife_intersect" => cmd_knife_intersect::command(a_command, map),
            "centerline" => cmd_centerline::command(a_command, map),
            _ => Err(TBError::UnknownCommand(a_command.command.clone())),
        };
        // convert TBError to a valid Error message
        if let Err(err) = rv {
            eprintln!("Detected error {:?}", err);
            Ok(PB_Response::new(PB_Reply {
                options: vec![PB_KeyValuePair {
                    key: "ERROR".to_string(),
                    value: format!("{:?}", err),
                }],
                models: Vec::<PB_Model>::with_capacity(0),
            }))
        } else {
            Ok(PB_Response::new(rv.unwrap()))
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let address = "[::1]:50051".parse()?;
    let service = TheToxicBlendService::default();

    Server::builder()
        .add_service(ToxicBlendServiceServer::new(service))
        .serve(address)
        .await?;

    Ok(())
}
