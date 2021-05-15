use super::TBError;
use crate::toxicblend_pb::Command as PB_Command;
use crate::toxicblend_pb::Face as PB_Face;
use crate::toxicblend_pb::KeyValuePair as PB_KeyValuePair;
use crate::toxicblend_pb::Model as PB_Model;
use crate::toxicblend_pb::Reply as PB_Reply;
use crate::toxicblend_pb::Vertex as PB_Vertex;
use std::collections::HashMap;
use std::time;

use dcc_lsystem::renderer::DataRendererOptionsBuilder;
use dcc_lsystem::renderer::Renderer;
use dcc_lsystem::turtle::{TurtleAction, TurtleLSystemBuilder};

/// Build the voxel data from the input
fn build_dragon_curve(
    cmd_arg_iterations: usize,
) -> Result<Vec<(i32, i32, i32, i32)>, TBError> {

    let mut builder = TurtleLSystemBuilder::new();

    let _ = builder
        .token("X", TurtleAction::Nothing)
        .token("Y", TurtleAction::Nothing)
        .token("F", TurtleAction::Forward(30))
        .token("+", TurtleAction::Rotate(-90))
        .token("-", TurtleAction::Rotate(90))
        .axiom("F X")
        .rule("X => X + Y F +")
        .rule("Y => - F X - Y");

    let (mut system, renderer) = builder.finish();
    system.step_by(cmd_arg_iterations);

    let now = time::Instant::now();
    let options = DataRendererOptionsBuilder::default().build();
    let rv = renderer.render(&system, &options);
    println!("lsystem render() duration: {:?}", now.elapsed());

    Ok(rv)
}

/// Build the return model
/// lines contains a vector of (<first vertex index>,<a list of points><last vertex index>)
fn build_output_bp_model(_a_command: &PB_Command, lines: Vec<(i32, i32, i32, i32)>) -> Result<PB_Model, TBError> {
    let mut vertices_map = ahash::AHashMap::<(i32, i32), usize>::new();
    // Default capacity is probably a little bit too low
    let mut pb_vertices = Vec::<PB_Vertex>::with_capacity(lines.len());
    let mut pb_faces = Vec::<PB_Face>::with_capacity(lines.len());

    for (x0,y0,x1,y1) in lines.into_iter() {
        let key = (x0,y0);
        let i0 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            pb_vertices.push(PB_Vertex {
                x: x0 as f64,
                y: y0 as f64,
                z: 0.0,
            });
            n
        });

        let key = (x1,y1);
        let i1 = *vertices_map.entry(key).or_insert_with(|| {
            let n = pb_vertices.len();
            pb_vertices.push(PB_Vertex {
                x: x1 as f64,
                y: y1 as f64,
                z: 0.0,
            });
            n
        });

        pb_faces.push(PB_Face{vertices:vec!(i0 as u64,i1 as u64)});
    }

    Ok(PB_Model {
        name: "Dragon Curve".to_string(),
        world_orientation: None,
        vertices: pb_vertices,
        faces: pb_faces,
    })
}

pub fn command(
    a_command: &PB_Command,
    options: HashMap<String, String>,
) -> Result<PB_Reply, TBError> {
    println!("Dragon curve got command: \"{}\"", a_command.command);
    if a_command.models.len() > 0 {
        return Err(TBError::InvalidInputData(format!(
            "This operation does not use any models as input:{}",
            a_command.models.len()
        )));
    }
    let cmd_arg_iterations = options
        .get("ITERATIONS")
        .ok_or_else(|| TBError::InvalidInputData("Missing the ITERATIONS parameter".to_string()))?
        .parse::<usize>()
        .map_err(|_| {
            TBError::InvalidInputData("Could not parse the ITERATIONS parameter".to_string())
        })?;

    if !(1..21).contains(&cmd_arg_iterations) {
        return Err(TBError::InvalidInputData(format!(
            "The valid range of ITERATIONS is [{}..{}[ :({})",
            1, 21, cmd_arg_iterations
        )));
    }

    println!("Dragon curve iterations:{:?} ", cmd_arg_iterations);

    let mesh = build_dragon_curve(
        cmd_arg_iterations,
    )?;
    let packed_faces_model = build_output_bp_model(&a_command, mesh)?;
    println!(
        "packed_faces_model.vertices.len() {}",
        packed_faces_model.vertices.len()
    );
    println!(
        "packed_faces_model.faces.len() {}",
        packed_faces_model.faces.len()
    );
    println!(
        "packed_faces_model.faces[0].vertices.len() {}",
        packed_faces_model.faces[0].vertices.len()
    );

    let reply = PB_Reply {
        options: vec![
            PB_KeyValuePair {
                key: "ONLY_EDGES".to_string(),
                value: "True".to_string(),
            },
        ],
        models: vec![packed_faces_model],
    };
    Ok(reply)
}
