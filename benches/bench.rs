use toxicblend::{
    execute_command, PB_Command, PB_Face32, PB_KeyValuePair, PB_Model32, PB_Vertex32,
};

use criterion::{criterion_group, criterion_main, Criterion};

fn generate_command(command_variant: String) -> PB_Command {
    PB_Command {
        command: command_variant,
        options: vec![
            PB_KeyValuePair {
                key: "RADIUS".to_string(),
                value: "10.0".to_string(),
            },
            PB_KeyValuePair {
                key: "DIVISIONS".to_string(),
                value: "325.0".to_string(),
            },
        ],
        models: vec![],
        models32: vec![PB_Model32 {
            name: "Cube".to_string(),
            world_orientation: None,
            vertices: vec![
                PB_Vertex32::from([1.0, 1.0, 1.0]),
                PB_Vertex32 {
                    x: 1.0,
                    y: 1.0,
                    z: -1.0,
                },
                PB_Vertex32 {
                    x: 1.0,
                    y: -1.0,
                    z: 1.0,
                },
                PB_Vertex32 {
                    x: 1.0,
                    y: -1.0,
                    z: -1.0,
                },
                PB_Vertex32 {
                    x: -1.0,
                    y: 1.0,
                    z: 1.0,
                },
                PB_Vertex32 {
                    x: -1.0,
                    y: 1.0,
                    z: -1.0,
                },
                PB_Vertex32 {
                    x: -1.0,
                    y: -1.0,
                    z: 1.0,
                },
                PB_Vertex32 {
                    x: -1.0,
                    y: -1.0,
                    z: -1.0,
                },
            ],
            faces: vec![
                PB_Face32::from([5, 7]),
                PB_Face32::from([1, 5]),
                PB_Face32::from([0, 1]),
                PB_Face32::from([7, 6]),
                PB_Face32::from([2, 3]),
                PB_Face32::from([4, 5]),
                PB_Face32::from([2, 6]),
                PB_Face32::from([0, 2]),
                PB_Face32::from([7, 3]),
                PB_Face32::from([6, 4]),
                PB_Face32::from([4, 0]),
                PB_Face32::from([3, 1]),
            ],
        }],
    }
}

#[cfg(test)]
pub fn bench_fsn_voxel(c: &mut Criterion) {
    let command = generate_command("voxel_fsn".to_string());

    c.bench_function("bench_fsn_voxel", |b| {
        let command = command.clone();
        b.iter({
            move || {
                let _ = execute_command(command.clone(), false).unwrap();
            }
        })
    });
}

#[cfg(test)]
pub fn bench_saft_voxel(c: &mut Criterion) {
    let command = generate_command("voxel_saft".to_string());

    c.bench_function("bench_saft_voxel", |b| {
        let command = command.clone();
        b.iter({
            move || {
                let _ = execute_command(command.clone(), false).unwrap();
            }
        })
    });
}

criterion_group! {name=benches1; config = Criterion::default().sample_size(40); targets=bench_fsn_voxel, bench_saft_voxel}
criterion_main!(benches1);
