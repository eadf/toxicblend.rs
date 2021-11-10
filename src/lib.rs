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
mod cmd_centerline;
mod cmd_fsn_mavoxel;
mod cmd_fsn_sdf;
mod cmd_fsn_voxel;
mod cmd_knife_intersect;
mod cmd_lsystems;
#[cfg(feature = "saft")]
mod cmd_saft_voxel;
mod cmd_simplify;
mod cmd_voronoi;
mod cmd_voronoi_mesh;
mod lsystems_3d;
mod type_utils;
mod voronoi_utils;

#[allow(unused_qualifications)] // tonic codegen has unused_qualifications
pub mod toxicblend_pb {
    tonic::include_proto!("toxicblend");
}

pub use toxicblend_pb::{
    Command as PB_Command, Face as PB_Face, Face32 as PB_Face32, KeyValuePair as PB_KeyValuePair,
    Matrix4x432 as PB_Matrix4x432, Model as PB_Model, Model32 as PB_Model32, Reply as PB_Reply,
    Vertex as PB_Vertex, Vertex32 as PB_Vertex32,
};

use std::collections::HashMap;

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
/// The largest allowed error when selecting a plane for 2D<->3D conversion
const EPSILON: f64 = 0.0001;

/// The largest dimension of the voronoi input, totally arbitrarily selected.
const DEFAULT_MAX_VORONOI_DIMENSION: f64 = 200000.0;

/// The length of one 'step' for curved edges discretization as a percentage of the longest
/// AABB axis of the object.
const DEFAULT_VORONOI_DISCRETE_DISTANCE: f64 = 0.0001;

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

#[inline(always)]
/// convert the options to a hashmap
fn options_to_map(options: &[PB_KeyValuePair]) -> HashMap<String, String> {
    let mut rv = HashMap::<String, String>::with_capacity(options.len());
    for kv in options.iter() {
        let _ = rv.insert(kv.key.clone(), kv.value.clone());
    }
    rv
}

#[inline]
pub fn execute_command(a_command: PB_Command, verbose: bool) -> Result<PB_Reply, TBError> {
    let options_map = options_to_map(&a_command.options);

    match a_command.command.as_str() {
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
        "voxel_fsn" => cmd_fsn_voxel::command(a_command, options_map, verbose),
        "mavoxel_fsn" => cmd_fsn_mavoxel::command(a_command, options_map),
        #[cfg(feature = "saft")]
        "voxel_saft" => cmd_saft_voxel::command(a_command, options_map, verbose),
        #[cfg(not(feature = "saft"))]
        "voxel_saft" => Err(TBError::DisabledFeature(String::from(
            "The feature 'saft' is not enabled in the server",
        ))),
        "lsystems" => cmd_lsystems::command(a_command, options_map),
        "sdf" => Err(TBError::InCompatibleVersion(String::from(
            "Please update your toxicblend blender addons",
        ))),
        //"sdf_bb" => cmd_bb_sdf::command(a_command, options_map),
        "sdf_fsn" => cmd_fsn_sdf::command(a_command, options_map, verbose),
        "sdf_saft" => Err(TBError::NotImplemented(String::from(
            "The sdf backend 'saft' is not implemented (yet)",
        ))),
        _ => Err(TBError::UnknownCommand(a_command.command)),
    }
}
