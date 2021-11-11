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
                value: "600.0".to_string(),
            },
        ],
        models: vec![],
        models32: vec![PB_Model32 {
            name: "bench_fsn_mavoxel".to_string(),
            world_orientation: None,
            vertices: vec![
                PB_Vertex32::from([0.043814618140, 0.189448788762, 0.000000000000]),
                PB_Vertex32::from([0.062846317887, 0.240106359124, 0.025404101238]),
                PB_Vertex32::from([0.052431099117, 0.215173318982, 0.013593746349]),
                PB_Vertex32::from([0.245472669601, 0.399445831776, 0.025162346661]),
                PB_Vertex32::from([0.061210963875, 0.254962563515, 0.022829664871]),
                PB_Vertex32::from([0.061185382307, 0.272923052311, 0.019907215610]),
                PB_Vertex32::from([0.063211768866, 0.289423137903, 0.017014885321]),
                PB_Vertex32::from([0.067330196500, 0.305923670530, 0.014266284183]),
                PB_Vertex32::from([0.072132982314, 0.318078935146, 0.012460504659]),
                PB_Vertex32::from([0.079940162599, 0.331875771284, 0.010756342672]),
                PB_Vertex32::from([0.088479116559, 0.343224227428, 0.009804142639]),
                PB_Vertex32::from([0.103355690837, 0.358017235994, 0.009444915690]),
                PB_Vertex32::from([0.122647292912, 0.372300833464, 0.009637964889]),
                PB_Vertex32::from([0.138594955206, 0.381501138210, 0.010316053405]),
                PB_Vertex32::from([0.165110811591, 0.393128752708, 0.011501277797]),
                PB_Vertex32::from([0.195621743798, 0.402790278196, 0.012294668704]),
                PB_Vertex32::from([0.217992007732, 0.407828390598, 0.012549886480]),
                PB_Vertex32::from([0.224233090878, 0.408348292112, 0.013295399956]),
                PB_Vertex32::from([0.232639282942, 0.406586050987, 0.016319062561]),
                PB_Vertex32::from([0.241433709860, 0.402307897806, 0.021888431162]),
                PB_Vertex32::from([0.086002759635, 0.234689682722, 0.006024818867]),
                PB_Vertex32::from([0.333619862795, 0.192329093814, 0.025486689061]),
                PB_Vertex32::from([0.251815944910, 0.405123084784, 0.020189192146]),
                PB_Vertex32::from([0.257342010736, 0.408701628447, 0.017216335982]),
                PB_Vertex32::from([0.264132857323, 0.411298692226, 0.015043575317]),
                PB_Vertex32::from([0.271026194096, 0.412238985300, 0.014530743472]),
                PB_Vertex32::from([0.291575253010, 0.412193864584, 0.015127480961]),
                PB_Vertex32::from([0.310724437237, 0.410985231400, 0.015832142904]),
                PB_Vertex32::from([0.328594028950, 0.408691197634, 0.016807455570]),
                PB_Vertex32::from([0.343319147825, 0.405712425709, 0.017921067774]),
                PB_Vertex32::from([0.358567714691, 0.401254773140, 0.019451206550]),
                PB_Vertex32::from([0.370432287455, 0.396480023861, 0.021067380905]),
                PB_Vertex32::from([0.386206924915, 0.387538462877, 0.023334158584]),
                PB_Vertex32::from([0.393160670996, 0.382234573364, 0.024711312726]),
                PB_Vertex32::from([0.400823235512, 0.374975025654, 0.025914754719]),
                PB_Vertex32::from([0.406442284584, 0.367599844933, 0.026890464127]),
                PB_Vertex32::from([0.410828471184, 0.360094130039, 0.027290821075]),
                PB_Vertex32::from([0.414165049791, 0.351751863956, 0.027399133891]),
                PB_Vertex32::from([0.416642874479, 0.342006176710, 0.027117913589]),
                PB_Vertex32::from([0.418186992407, 0.330263525248, 0.026379071176]),
                PB_Vertex32::from([0.418235063553, 0.321639746428, 0.025864582509]),
                PB_Vertex32::from([0.416293621063, 0.304134428501, 0.023919524625]),
                PB_Vertex32::from([0.413984179497, 0.295173376799, 0.022680072114]),
                PB_Vertex32::from([0.406274050474, 0.276289999485, 0.019489295781]),
                PB_Vertex32::from([0.398121118546, 0.262711346149, 0.016842618585]),
                PB_Vertex32::from([0.384809672832, 0.246031939983, 0.013779270463]),
                PB_Vertex32::from([0.369562566280, 0.231558382511, 0.011199929751]),
                PB_Vertex32::from([0.349266916513, 0.216321647167, 0.009026985615]),
                PB_Vertex32::from([0.343782603741, 0.210546970367, 0.010310550220]),
                PB_Vertex32::from([0.339854985476, 0.204239130020, 0.014702289365]),
                PB_Vertex32::from([0.226685792208, 0.207206279039, 0.026782741770]),
                PB_Vertex32::from([0.245185166597, 0.392832010984, 0.024722676724]),
                PB_Vertex32::from([0.226235046983, 0.221475526690, 0.024762965739]),
                PB_Vertex32::from([0.225959360600, 0.214396432042, 0.025268735364]),
                PB_Vertex32::from([0.454468965530, 0.013122913428, 0.025361066684]),
                PB_Vertex32::from([0.350037813187, 0.159057572484, 0.026106500998]),
                PB_Vertex32::from([0.369901299477, 0.121338024735, 0.026235291734]),
                PB_Vertex32::from([0.389107137918, 0.087774232030, 0.025710737333]),
                PB_Vertex32::from([0.407851755619, 0.058442831039, 0.024785570800]),
                PB_Vertex32::from([0.420077890158, 0.041968833655, 0.024383071810]),
                PB_Vertex32::from([0.432154029608, 0.028599349782, 0.024497672915]),
                PB_Vertex32::from([0.445286154747, 0.017770389095, 0.025164512917]),
                PB_Vertex32::from([0.317419856787, 0.200835824013, 0.014994658530]),
                PB_Vertex32::from([0.310045212507, 0.202685892582, 0.012548433617]),
                PB_Vertex32::from([0.306377261877, 0.202850833535, 0.012167789973]),
                PB_Vertex32::from([0.277385175228, 0.202316403389, 0.013175206259]),
                PB_Vertex32::from([0.248815402389, 0.204775154591, 0.014131255448]),
                PB_Vertex32::from([0.156715780497, 0.016899963841, 0.021180691198]),
                PB_Vertex32::from([0.223167121410, 0.193664684892, 0.024777105078]),
                PB_Vertex32::from([0.214157521725, 0.118968665600, 0.024426016957]),
                PB_Vertex32::from([0.210985124111, 0.103756822646, 0.023961585015]),
                PB_Vertex32::from([0.207534387708, 0.092200480402, 0.023252112791]),
                PB_Vertex32::from([0.201637685299, 0.079523652792, 0.021280774847]),
                PB_Vertex32::from([0.192353218794, 0.066469453275, 0.017384568229]),
                PB_Vertex32::from([0.176443651319, 0.049817088991, 0.011450582184]),
                PB_Vertex32::from([0.165961071849, 0.041320119053, 0.008549008518]),
                PB_Vertex32::from([0.162783890963, 0.038229946047, 0.008305910975]),
                PB_Vertex32::from([0.158690124750, 0.030243493617, 0.011521312408]),
                PB_Vertex32::from([0.514037668705, 0.077754542232, 0.000000000000]),
                PB_Vertex32::from([0.465178996325, 0.014503707178, 0.021201808006]),
                PB_Vertex32::from([0.477948993444, 0.019134351984, 0.016303455457]),
                PB_Vertex32::from([0.488584280014, 0.025260532275, 0.012859232724]),
                PB_Vertex32::from([0.498801201582, 0.033881053329, 0.010556406341]),
                PB_Vertex32::from([0.504513204098, 0.040394168347, 0.009781785309]),
                PB_Vertex32::from([0.509759128094, 0.047936774790, 0.009189116769]),
                PB_Vertex32::from([0.513150155544, 0.054211873561, 0.008746384643]),
                PB_Vertex32::from([0.516300261021, 0.062621407211, 0.007771582343]),
                PB_Vertex32::from([0.516846835613, 0.067785978317, 0.006184345111]),
                PB_Vertex32::from([0.454721838236, 0.009003028274, 0.021280152723]),
                PB_Vertex32::from([0.242517963052, 0.029005005956, 0.000000000000]),
                PB_Vertex32::from([0.168478980660, 0.014395446517, 0.019276231527]),
                PB_Vertex32::from([0.198184877634, 0.012696962804, 0.019087130204]),
                PB_Vertex32::from([0.071893371642, 0.000001162986, 0.000000000000]),
                PB_Vertex32::from([0.149789541960, 0.015609146096, 0.019540118054]),
                PB_Vertex32::from([0.142925620079, 0.015545018949, 0.019127815962]),
                PB_Vertex32::from([0.118493705988, 0.017508855090, 0.019849514589]),
            ],
            faces: vec![
                PB_Face32::from([0, 2]),
                PB_Face32::from([1, 4]),
                PB_Face32::from([4, 5]),
                PB_Face32::from([5, 6]),
                PB_Face32::from([6, 7]),
                PB_Face32::from([7, 8]),
                PB_Face32::from([8, 9]),
                PB_Face32::from([9, 10]),
                PB_Face32::from([10, 11]),
                PB_Face32::from([11, 12]),
                PB_Face32::from([12, 13]),
                PB_Face32::from([13, 14]),
                PB_Face32::from([14, 15]),
                PB_Face32::from([15, 16]),
                PB_Face32::from([16, 17]),
                PB_Face32::from([17, 18]),
                PB_Face32::from([2, 1]),
                PB_Face32::from([1, 20]),
                PB_Face32::from([3, 22]),
                PB_Face32::from([22, 23]),
                PB_Face32::from([23, 24]),
                PB_Face32::from([24, 25]),
                PB_Face32::from([25, 26]),
                PB_Face32::from([26, 27]),
                PB_Face32::from([27, 28]),
                PB_Face32::from([28, 29]),
                PB_Face32::from([29, 30]),
                PB_Face32::from([30, 31]),
                PB_Face32::from([31, 32]),
                PB_Face32::from([32, 33]),
                PB_Face32::from([33, 34]),
                PB_Face32::from([34, 35]),
                PB_Face32::from([35, 36]),
                PB_Face32::from([36, 37]),
                PB_Face32::from([37, 38]),
                PB_Face32::from([38, 39]),
                PB_Face32::from([39, 40]),
                PB_Face32::from([40, 41]),
                PB_Face32::from([41, 42]),
                PB_Face32::from([42, 43]),
                PB_Face32::from([43, 44]),
                PB_Face32::from([44, 45]),
                PB_Face32::from([45, 46]),
                PB_Face32::from([46, 47]),
                PB_Face32::from([47, 48]),
                PB_Face32::from([48, 49]),
                PB_Face32::from([3, 51]),
                PB_Face32::from([51, 52]),
                PB_Face32::from([52, 53]),
                PB_Face32::from([21, 55]),
                PB_Face32::from([55, 56]),
                PB_Face32::from([56, 57]),
                PB_Face32::from([57, 58]),
                PB_Face32::from([58, 59]),
                PB_Face32::from([59, 60]),
                PB_Face32::from([60, 61]),
                PB_Face32::from([21, 62]),
                PB_Face32::from([62, 63]),
                PB_Face32::from([63, 64]),
                PB_Face32::from([64, 65]),
                PB_Face32::from([65, 66]),
                PB_Face32::from([50, 68]),
                PB_Face32::from([68, 69]),
                PB_Face32::from([69, 70]),
                PB_Face32::from([70, 71]),
                PB_Face32::from([71, 72]),
                PB_Face32::from([72, 73]),
                PB_Face32::from([73, 74]),
                PB_Face32::from([74, 75]),
                PB_Face32::from([75, 76]),
                PB_Face32::from([76, 77]),
                PB_Face32::from([54, 79]),
                PB_Face32::from([79, 80]),
                PB_Face32::from([80, 81]),
                PB_Face32::from([81, 82]),
                PB_Face32::from([82, 83]),
                PB_Face32::from([83, 84]),
                PB_Face32::from([84, 85]),
                PB_Face32::from([85, 86]),
                PB_Face32::from([86, 87]),
                PB_Face32::from([54, 88]),
                PB_Face32::from([67, 90]),
                PB_Face32::from([90, 91]),
                PB_Face32::from([67, 93]),
                PB_Face32::from([93, 94]),
                PB_Face32::from([94, 95]),
                PB_Face32::from([19, 3]),
                PB_Face32::from([91, 89]),
                PB_Face32::from([53, 50]),
                PB_Face32::from([77, 67]),
                PB_Face32::from([49, 21]),
                PB_Face32::from([95, 92]),
                PB_Face32::from([87, 78]),
                PB_Face32::from([66, 50]),
                PB_Face32::from([61, 54]),
                PB_Face32::from([18, 19]),
            ],
        }],
    };

    c.bench_function("bench_fsn_mavoxel", |b| {
        b.iter({
            let command = command.clone();
            move || {
                let _ = execute_command(command.clone(), false).unwrap();
            }
        })
    });
}

criterion_group! {name=benches1; config = Criterion::default().sample_size(40); targets=bench_fsn_voxel, bench_saft_voxel, bench_fsn_mavoxel}
criterion_main!(benches1);
