#![feature(hash_drain_filter)]
#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
#![deny(unused_imports)]

mod cmd_2d_outline;
mod cmd_centerline;
mod cmd_centerline_mesh;
mod cmd_knife_intersect;
mod cmd_simplify;
mod cmd_voronoi;

use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use cgmath::Zero;
use std::collections::HashMap;
use tonic::transport::Server;
use tonic::Request as PB_Request;
use tonic::Response as PB_Response;
use tonic::Status as PB_Status;
use toxicblend_pb::toxic_blend_service_server::{ToxicBlendService, ToxicBlendServiceServer};

/// The largest allowed error when selecting a plane for 2D<->3D conversion
const EPSILON: f64 = 0.0001;

/// The largest dimension of the voronoi input, totally arbitrarily selected.
const MAX_VORONOI_DIMENSION: f64 = 256.0 * 800.0;

/// The length of one 'step' for curved edges discretization
const VORONOI_DISCRETE_DISTANCE: f64 = 0.0001;

pub mod toxicblend_pb {
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
    #[inline(always)]
    pub fn distance_squared(&self, other: &PB_Vertex) -> f64 {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;
        x * x + y * y + z * z
    }
}

/// converts a Point2 to Point3 using the XY coordinates
#[inline(always)]
pub fn xy_to_3d(point: &cgmath::Point2<f64>) -> cgmath::Point3<f64> {
    cgmath::Point3 {
        x: point.x,
        y: point.y,
        z: f64::zero(),
    }
}

/// converts a Point3 to Point2 using the XY coordinates
fn xy_to_2d(point: &cgmath::Point3<f64>) -> cgmath::Point2<f64> {
    cgmath::Point2 {
        x: point.x,
        y: point.y,
    }
}

#[derive(thiserror::Error, Debug)]
pub enum TBError {
    #[error("Unknown command received: {0}")]
    UnknownCommand(String),

    #[error("Model should not contain any faces")]
    ModelContainsFaces(String),

    #[error("Something is wrong with the internal logic: {0}")]
    InternalError(String),

    #[error("Something is wrong with the input data")]
    CouldNotCalculateInvertMatrix,

    #[error("Your line-strings are self-intersecting.")]
    SelfIntersectingData,

    #[error("The input data is not 2D")]
    InputNotPLane(String),

    #[error("Invalid input data: {0}")]
    InvalidInputData(String),

    #[error(transparent)]
    IoError(#[from] std::io::Error),

    #[error(transparent)]
    LinestringError(#[from] linestring::LinestringError),

    #[error(transparent)]
    CenterLineError(#[from] centerline::CenterlineError),

    #[error(transparent)]
    BoostVoronoiError(#[from] boostvoronoi::BvError),
}

#[derive(Debug, Default)]
pub struct TheToxicBlendService {}

/// convert the options to a hashmap
fn options_to_map(options: &[PB_KeyValuePair]) -> HashMap<String, String> {
    let mut rv = HashMap::<String, String>::new();
    for kv in options.iter() {
        let _ = rv.insert(kv.key.clone(), kv.value.clone());
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
            "centerline_mesh" => cmd_centerline_mesh::command(a_command, map),
            "voronoi" => cmd_voronoi::command(a_command, map),
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
    let address = "[::1]:50069".parse()?;
    let service = TheToxicBlendService::default();

    Server::builder()
        .add_service(ToxicBlendServiceServer::new(service))
        .serve(address)
        .await?;

    Ok(())
}
