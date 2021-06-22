#![feature(hash_drain_filter)]
#![deny(non_camel_case_types)]
#![deny(unused_parens)]
#![deny(non_upper_case_globals)]
#![deny(unused_qualifications)]
#![deny(unused_results)]
#![deny(unused_imports)]

mod cmd_2d_outline;
mod cmd_centerline;
mod cmd_knife_intersect;
mod cmd_lsystems;
mod cmd_sdf;
mod cmd_simplify;
mod cmd_voronoi;
mod cmd_voronoi_mesh;
mod cmd_voxel;
mod lsystems_3d;
mod voronoi_utils;

use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
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

/// The default of the largest dimension of the voronoi input, totally arbitrarily selected.
const DEFAULT_MAX_VORONOI_DIMENSION: f64 = 200000.0;

/// The default value of length of one 'step' for curved edges discretization as a
/// percentage of the longest AABB axis of the object.
const DEFAULT_VORONOI_DISCRETE_DISTANCE: f64 = 0.0001;

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
    /// Returns the distance² between two vertices
    pub fn distance_squared(&self, other: &PB_Vertex) -> f64 {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;
        x * x + y * y + z * z
    }
}

/// converts a Point2 to Point3 using the XY coordinates, sets Z to zero.
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
    #[error("Overflow error: {0}")]
    Overflow(String),

    #[error("Unknown command received: {0}")]
    UnknownCommand(String),

    #[error("Model should not contain any faces")]
    ModelContainsFaces(String),

    #[error("Something is wrong with the internal logic: {0}")]
    InternalError(String),

    #[error("Could not parse string: {0}")]
    ParseError(String),

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

    #[error(transparent)]
    JoinError(#[from] tokio::task::JoinError),

    #[error("Error in LSystems {0}")]
    LSystems3D(String),
}

/// The main gRPC dispatcher. It receives the commands and sends them to the correct operations.
#[derive(Debug, Default)]
pub struct TheToxicBlendService {}

#[inline(always)]
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
        let options_map = options_to_map(&a_command.options);

        let rv = match a_command.command.as_str() {
            "2d_outline" => cmd_2d_outline::command(a_command, options_map),
            "simplify" => cmd_simplify::command(a_command, options_map),
            "knife_intersect" => cmd_knife_intersect::command(a_command, options_map),
            "centerline" => cmd_centerline::command(a_command, options_map),
            "voronoi_mesh" => cmd_voronoi_mesh::command(a_command, options_map),
            "voronoi" => cmd_voronoi::command(a_command, options_map),
            "voxel" => cmd_voxel::command(a_command, options_map),
            "lsystems" => cmd_lsystems::command(a_command, options_map),
            "sdf" => cmd_sdf::command(a_command, options_map),
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
/// Main function, will start the gRPC ToxicBlendService
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let address = "[::1]:50069".parse()?;
    let service = TheToxicBlendService::default();

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
