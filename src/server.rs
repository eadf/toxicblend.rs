#![deny(
    rust_2018_compatibility,
    rust_2018_idioms,
    nonstandard_style,
    unused,
    future_incompatible,
    non_camel_case_types,
    unused_parens,
    non_upper_case_globals,
    unused_qualifications,
    unused_results,
    unused_imports,
    unused_variables
)]
#![cfg_attr(feature = "hash_drain_filter", feature(hash_drain_filter))]
#![cfg_attr(feature = "map_first_last", feature(map_first_last))]

#[cfg(feature = "saft")]
extern crate saft_cr as saft;
mod cmd_2d_outline;
// Replacing building_blocks with fast_surface_nets, but keeping the files for now
//mod cmd_bb_sdf;
//mod cmd_bb_voxel;
mod cmd_centerline;
mod cmd_fsn_sdf;
mod cmd_fsn_voxel;
mod cmd_knife_intersect;
mod cmd_lsystems;
#[cfg(feature = "saft")]
mod cmd_saft_voxel;
//#[cfg(feature = "saft")]
//mod cmd_saft_sdf;
mod cmd_simplify;
mod cmd_voronoi;
mod cmd_voronoi_mesh;
mod lsystems_3d;
mod type_utils;
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

/// The largest dimension of the voronoi input, totally arbitrarily selected.
const DEFAULT_MAX_VORONOI_DIMENSION: f64 = 200000.0;

/// The length of one 'step' for curved edges discretization as a percentage of the longest
/// AABB axis of the object.
const DEFAULT_VORONOI_DISCRETE_DISTANCE: f64 = 0.0001;

#[allow(unused_qualifications)] // tonic codegen has unused_qualifications
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

pub(crate) trait GrowingVob {
    fn fill(initial_size: usize) -> vob::Vob<u32>;
    fn set_grow(&mut self, bit: usize, state: bool) -> bool;
    /// get with default value: false
    fn get_f(&self, bit: usize) -> bool;
}

impl GrowingVob for vob::Vob<u32> {
    fn fill(initial_size: usize) -> Self {
        let mut v: vob::Vob<u32> = vob::Vob::<u32>::new_with_storage_type(0);
        v.resize(initial_size, false);
        v
    }
    #[inline]
    fn set_grow(&mut self, bit: usize, state: bool) -> bool {
        if bit >= self.len() {
            self.resize(bit + 512, false);
        }
        self.set(bit, state)
    }
    #[inline]
    fn get_f(&self, bit: usize) -> bool {
        self.get(bit).unwrap_or(false)
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
    #[error("In compatibleVersion : {0}")]
    InCompatibleVersion(String),

    #[error("Not implemented : {0}")]
    NotImplemented(String),

    #[error("Overflow error: {0}")]
    Overflow(String),

    #[error("Feature not active: {0}")]
    DisabledFeature(String),

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

    #[cfg(feature = "saft")]
    #[error(transparent)]
    SaftError(#[from] saft::Error),
}

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
            "voxel" => Err(TBError::InCompatibleVersion(String::from(
                "Please update your toxicblend blender addons",
            ))),
            //"voxel_bb" => cmd_bb_voxel::command(a_command, options_map),
            "voxel_fsn" => cmd_fsn_voxel::command(a_command, options_map),
            #[cfg(feature = "saft")]
            "voxel_saft" => cmd_saft_voxel::command(a_command, options_map),
            #[cfg(not(feature = "saft"))]
            "voxel_saft" => Err(TBError::DisabledFeature(String::from(
                "The feature 'saft' is not enabled in the server",
            ))),
            "lsystems" => cmd_lsystems::command(a_command, options_map),
            "sdf" => Err(TBError::InCompatibleVersion(String::from(
                "Please update your toxicblend blender addons",
            ))),
            //"sdf_bb" => cmd_bb_sdf::command(a_command, options_map),
            "sdf_fsn" => cmd_fsn_sdf::command(a_command, options_map),
            "sdf_saft" => Err(TBError::NotImplemented(String::from(
                "The sdf backend 'saft' is not implemented (yet)",
            ))),
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
