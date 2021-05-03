use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use building_blocks::core::prelude::PointN;
use building_blocks::core::prelude::*;
use building_blocks::core::sdfu::{self, SDF};
use building_blocks::mesh::*;
use building_blocks::storage::prelude::*;
use itertools::Itertools;
use std::collections::HashMap;
use std::time;

/// unpack the input PB_Model
pub fn parse_input_pb_model(
    obj: &PB_Model,
) -> Result<
    (
        Vec<PointN<[f32; 3]>>,
        Vec<(usize, usize)>,
        linestring::cgmath_3d::Aabb3<f64>,
    ),
    TBError,
> {
    let mut aabb = linestring::cgmath_3d::Aabb3::<f64>::default();
    let vertices: Vec<PointN<[f32; 3]>> = obj
        .vertices
        .iter()
        .map(|vertex| {
            aabb.update_point(&cgmath::Point3 {
                x: vertex.x,
                y: vertex.y,
                z: vertex.z,
            });
            PointN([vertex.x as f32, vertex.y as f32, vertex.z as f32])
        })
        .collect();
    let mut edges = Vec::<(usize, usize)>::with_capacity(vertices.len() + 100);

    for face in obj.faces.iter() {
        if face.vertices.len() > 2 {
            return Err(TBError::ModelContainsFaces("Model can't contain any faces, only edges. Use the 2d_outline tool to remove faces".to_string()));
        }
        if face.vertices.len() < 2 {
            return Err(TBError::InvalidInputData(
                "Edge containing none or only one vertex".to_string(),
            ));
        };
        edges.push((
            *face.vertices.first().unwrap() as usize,
            *face.vertices.last().unwrap() as usize,
        ));
    }
    Ok((vertices, edges, aabb))
}

/// Build the voxel data from the input
fn build_voxel(
    radius_multiplier: f64,
    divisions: f64,
    vertices: Vec<PointN<[f32; 3]>>,
    edges: Vec<(usize, usize)>,
    aabb: linestring::cgmath_3d::Aabb3<f64>,
) -> Result<PosNormMesh, TBError> {
    let dimensions = aabb.get_high().unwrap() - aabb.get_low().unwrap();
    let max_dimension = dimensions.x.max(dimensions.y).max(dimensions.z);
    //let center = (aabb.get_high().unwrap() + aabb.get_low().unwrap())/2.0;

    let radius = max_dimension * radius_multiplier;
    let scale = (divisions / max_dimension) as f32;
    let thickness = (radius * 2.0) as f32 * (scale.sqrt());
    println!(
        "Voxelizing using tube thickness. {} = {}*{}*sqrt({})",
        thickness, max_dimension, radius_multiplier, scale
    );

    println!(
        "Voxelizing using divisions {}, max dimension = {}, scale factor={} test={}",
        divisions,
        max_dimension,
        scale,
        (max_dimension as f32) * scale
    );
    //println!("lines:{:?}", edges);
    println!("aabb.high:{:?}", aabb.get_high().unwrap());
    println!("aabb.low:{:?}", aabb.get_low().unwrap());
    println!(
        "delta:{:?}",
        aabb.get_high().unwrap() - aabb.get_low().unwrap()
    );
    let vertices: Vec<PointN<[f32; 3]>> = vertices.into_iter().map(|v| v * scale).collect();

    let mut sdfu_vec = Vec::<(
        PointN<[f32; 3]>,
        PointN<[f32; 3]>,
        Box<dyn Fn(Point3f) -> f32>,
    )>::new();
    if !edges.is_empty() {
        for (from, to) in edges.into_iter() {
            let f = vertices[from];
            let t = vertices[to];
            sdfu_vec.push((
                f,
                t,
                Box::new(move |p: Point3f| -> f32 { sdfu::Line::new(f, t, thickness).dist(p) }),
            ));
        }
    }

    let samples = {
        let main_extent = {
            let aabb_min = aabb.get_low().unwrap() * (scale as f64);
            let aabb_max = aabb.get_high().unwrap() * (scale as f64);
            Extent3i::from_min_and_max(
                PointN([aabb_min.x as i32, aabb_min.y as i32, aabb_min.z as i32]),
                PointN([aabb_max.x as i32, aabb_max.y as i32, aabb_max.z as i32]),
            )
            .padded(thickness as i32 + 2)
        };
        println!("extent:{:?}", main_extent);
        let now = time::Instant::now();

        let mut samples = Array3x1::fill(main_extent, thickness * 10.0_f32);
        for sample_sdf in sdfu_vec.into_iter() {
            let sample_extent = {
                let aabb_min = [
                    sample_sdf.0.x().min(sample_sdf.1.x()) as i32,
                    sample_sdf.0.y().min(sample_sdf.1.y()) as i32,
                    sample_sdf.0.z().min(sample_sdf.1.z()) as i32,
                ];
                let aabb_max = [
                    sample_sdf.0.x().max(sample_sdf.1.x()) as i32,
                    sample_sdf.0.y().max(sample_sdf.1.y()) as i32,
                    sample_sdf.0.z().max(sample_sdf.1.z()) as i32,
                ];
                Extent3i::from_min_and_max(PointN(aabb_min), PointN(aabb_max))
                    .padded(thickness as i32 + 2)
            };
            samples.for_each_mut(&sample_extent, |p: Point3i, dist| {
                *dist = dist.min(sample_sdf.2(Point3f::from(p)));
            });
        }
        println!("fill() duration: {:?}", now.elapsed());
        samples
    };

    let mut mesh_buffer = SurfaceNetsBuffer::default();
    let voxel_size = 1.0 / scale;
    let now = time::Instant::now();
    surface_nets(&samples, samples.extent(), voxel_size, &mut mesh_buffer);
    println!("surface_nets() duration: {:?}", now.elapsed());
    Ok(mesh_buffer.mesh)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_output_bp_model(a_command: &PB_Command, mesh: PosNormMesh) -> Result<PB_Model, TBError> {
    let input_pb_model = &a_command.models[0];

    let pb_vertices = mesh
        .positions
        .iter()
        .map(|[x, y, z]| PB_Vertex {
            x: *x as f64,
            y: *y as f64,
            z: *z as f64,
        })
        .collect();
    let pb_faces = mesh
        .indices
        .iter()
        .tuple_windows::<(_, _, _)>()
        .step_by(3)
        .map(|(a, b, c)| PB_Face {
            vertices: vec![*a as u64, *b as u64, *c as u64],
        })
        .collect();

    Ok(PB_Model {
        name: input_pb_model.name.clone(),
        world_orientation: input_pb_model.world_orientation.clone(),
        vertices: pb_vertices,
        faces: pb_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("Voxel got command: \"{}\"", a_command.command);
    if a_command.models.len() > 1 {
        return Err(TBError::InvalidInputData(format!(
            "This operation only supports one model as input:{}",
            a_command.models.len()
        )));
    }
    if a_command.models.is_empty() {
        return Err(TBError::InvalidInputData(
            "Model did not contain any data".to_string(),
        ));
    }
    let cmd_arg_radius_multiplier = options
        .get("RADIUS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the RADIUS parameter".to_string()))?
        .parse::<f64>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the RADIUS parameter".to_string())
        })?
        / 100.0;

    let cmd_arg_divisions = options
        .get("DIVISIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the DIVISIONS parameter".to_string()))?
        .parse::<f64>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the DIVISIONS parameter".to_string())
        })?;
    if !(9.9..1000.1).contains(&cmd_arg_divisions) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of DIVISIONS is [{}..{}[% :({})",
            10, 1000, cmd_arg_divisions
        )));
    }

    for model in a_command.models.iter() {
        println!("model.name:{:?}, ", model.name);
        println!("model.vertices:{:?}, ", model.vertices.len());
        println!("model.faces:{:?}, ", model.faces.len());
        println!(
            "model.world_orientation:{:?}, ",
            model.world_orientation.as_ref().map_or(0, |_| 16)
        );
        println!("Tube radius:{:?}% multiplier ", cmd_arg_radius_multiplier);
        println!("Divisions:{:?} ", cmd_arg_divisions);
        println!();
    }

    let (vertices, edges, aabb) = parse_input_pb_model(&a_command.models[0])?;
    let mesh = build_voxel(
        cmd_arg_radius_multiplier,
        cmd_arg_divisions,
        vertices,
        edges,
        aabb,
    )?;
    let model = build_output_bp_model(&a_command, mesh)?;

    let reply = PB_Reply {
        options: vec![PB_KeyValuePair {
            key: "ONLY_EDGES".to_string(),
            value: "False".to_string(),
        }],
        models: vec![model],
    };
    Ok(reply)
}
