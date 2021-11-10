use toxicblend::{
    execute_command, PB_Command, PB_Face32, PB_KeyValuePair, PB_Model32, PB_Vertex32,
};

use criterion::{criterion_group, criterion_main, Criterion};

fn generate_voxel_command(command_variant: String) -> PB_Command {
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
    let command = generate_voxel_command("voxel_fsn".to_string());

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
    let command = generate_voxel_command("voxel_saft".to_string());

    c.bench_function("bench_saft_voxel", |b| {
        let command = command.clone();
        b.iter({
            move || {
                let _ = execute_command(command.clone(), false).unwrap();
            }
        })
    });
}

#[cfg(test)]
pub fn bench_fsn_mavoxel(c: &mut Criterion) {
    let command = PB_Command {
        command: "mavoxel_fsn".to_string(),
        options: vec![
            PB_KeyValuePair {
                key: "RADIUS_AXIS".to_string(),
                value: "XY".to_string(),
            },
            PB_KeyValuePair {
                key: "DIVISIONS".to_string(),
                value: "400.0".to_string(),
            },
        ],
        models: vec![],
        models32: vec![PB_Model32 {
            name: "Plane".to_string(),
            world_orientation: None,
            vertices: vec![
                PB_Vertex32::from([-1.000000000000, -1.000000000000, 0.922545969486]),
                PB_Vertex32::from([1.000000000000, -1.000000000000, 0.498110979795]),
                PB_Vertex32::from([-1.000000000000, 1.000000000000, 0.371810138226]),
                PB_Vertex32::from([1.000000000000, 1.000000000000, 0.529315829277]),
                PB_Vertex32::from([0.000000000000, 0.000000000000, 0.000000000000]),
                PB_Vertex32::from([0.913968265057, -1.411813497543, 0.922545969486]),
                PB_Vertex32::from([2.408138036728, -0.082356274128, 0.498110979795]),
                PB_Vertex32::from([-0.415488958359, 0.082356274128, 0.371810138226]),
                PB_Vertex32::from([1.078680753708, 1.411813497543, 0.529315829277]),
                PB_Vertex32::from([0.996324479580, -0.000000039621, 0.000000000000]),
            ],
            faces: vec![
                PB_Face32::from([0, 4]),
                PB_Face32::from([4, 2]),
                PB_Face32::from([1, 4]),
                PB_Face32::from([4, 3]),
                PB_Face32::from([5, 9]),
                PB_Face32::from([9, 7]),
                PB_Face32::from([6, 9]),
                PB_Face32::from([9, 8]),
            ],
        }],
    };

    c.bench_function("bench_fsn_mavoxel", |b| {
        let command = command.clone();
        b.iter({
            move || {
                let _ = execute_command(command.clone(), false).unwrap();
            }
        })
    });
}

criterion_group! {name=benches1; config = Criterion::default().sample_size(40); targets=bench_fsn_voxel, bench_saft_voxel, bench_fsn_mavoxel}
criterion_main!(benches1);
