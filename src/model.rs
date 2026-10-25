//! A robot's body, for the book's 3D views: from drydock's robot file and
//! the MuJoCo model it names, the bodies (how each sits on its parent, the
//! joint that moves it), the motors (which joint each turns, through which
//! transmission) and the bodies' collision meshes, made lighter.
//!
//!     cargo run --release -- model ../drydock/examples/openarm/robot.yml book/models/openarm.js
//!
//! What's read of the MJCF is what OpenArm's model uses: bodies placed by
//! `pos` and `quat`, hinge and slide joints, mesh geoms of class
//! `collision`, joints that follow others through `<equality>`. Anything
//! else in a body (`euler`, `axisangle`, other geom types) is an error,
//! rather than a body drawn in the wrong place.

use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde_json::{json, Value};

use crate::Result;

#[derive(Deserialize)]
struct Robot {
    scene: Scene,
    buses: Vec<Bus>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum Scene {
    File(PathBuf),
    Git { git: String, commit: String, file: PathBuf },
}

#[derive(Deserialize)]
struct Bus {
    motors: Vec<Motor>,
}

#[derive(Deserialize)]
struct Motor {
    joint: String,
    id: u32,
    #[serde(default)]
    transmission: Option<Transmission>,
}

#[derive(Deserialize)]
struct Transmission {
    ratio: f64,
    offset: f64,
}

/// Meshes are thinned by merging the vertices that fall in one cell of a
/// grid this fine, as a share of the mesh's size: plenty for a view a few
/// hundred pixels wide.
const CELLS: f64 = 140.0;

pub fn write(robot_file: &Path, out: &Path) -> Result<()> {
    let robot: Robot = serde_yaml::from_str(&fs::read_to_string(robot_file)?)?;
    let scene = match &robot.scene {
        Scene::File(path) => robot_file.parent().unwrap_or(Path::new(".")).join(path),
        // Where drydock fetched it to (see drydock's `robot::fetch`).
        Scene::Git { git, commit, file } => {
            let name = git.trim_end_matches('/').trim_end_matches(".git").rsplit('/').next().unwrap_or(git);
            let cache = std::env::var_os("DRYDOCK_CACHE")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".cache/drydock"));
            cache.join("models").join(format!("{name}-{}", &commit[..commit.len().min(12)])).join(file)
        }
    };
    let xml = fs::read_to_string(&scene).map_err(|e| format!("{}: {e} (run drydock with this robot once, to fetch it)", scene.display()))?;
    let doc = roxmltree::Document::parse(&xml)?;
    let root = doc.root_element();
    if let Some(c) = child(root, "compiler") {
        if c.attribute("angle").is_some_and(|a| a != "radian") {
            return Err("the model's angles are in degrees; only radians are read".into());
        }
    }
    let meshdir = child(root, "compiler").and_then(|c| c.attribute("meshdir")).unwrap_or("");
    let dir = scene.parent().unwrap_or(Path::new(".")).join(meshdir);

    // Meshes, by name: their file and scale.
    let mut mesh_files = HashMap::new();
    if let Some(asset) = child(root, "asset") {
        for m in asset.children().filter(|c| c.has_tag_name("mesh")) {
            let (Some(name), Some(file)) = (m.attribute("name"), m.attribute("file")) else { continue };
            let scale = floats(m.attribute("scale").unwrap_or("1 1 1"))?;
            mesh_files.insert(name.to_owned(), (dir.join(file), scale));
        }
    }

    // Bodies, depth first, each after its parent.
    let mut bodies = Vec::new();
    let mut used = Vec::new();
    let world = child(root, "worldbody").ok_or("no <worldbody>")?;
    for b in world.children().filter(|c| c.has_tag_name("body")) {
        body(b, None, &mut bodies, &mut used)?;
    }

    let mut meshes = serde_json::Map::new();
    for name in used {
        if meshes.contains_key(&name) {
            continue;
        }
        let (file, scale) = mesh_files.get(&name).ok_or_else(|| format!("no mesh {name}"))?;
        meshes.insert(name, thin(&stl(file)?, scale)?);
    }

    // Joints that follow another: the gripper's second finger.
    let mut follows = serde_json::Map::new();
    if let Some(eq) = child(root, "equality") {
        for j in eq.children().filter(|c| c.has_tag_name("joint")) {
            if let (Some(a), Some(b)) = (j.attribute("joint1"), j.attribute("joint2")) {
                follows.insert(b.to_owned(), json!(a));
            }
        }
    }

    let motors: Vec<Value> = robot
        .buses
        .iter()
        .flat_map(|b| &b.motors)
        .map(|m| {
            let (ratio, offset) = m.transmission.as_ref().map_or((1.0, 0.0), |t| (t.ratio, t.offset));
            json!({ "id": m.id, "joint": m.joint, "ratio": ratio, "offset": offset })
        })
        .collect();
    let name = out.file_stem().and_then(|s| s.to_str()).ok_or("the out file needs a name")?;
    let data = json!({ "bodies": bodies, "motors": motors, "follows": follows, "meshes": meshes });
    let script = format!(
        "// Written by keel-book from {} and {}: Enactic's OpenArm model, Apache-2.0.\n\
         (window.KEEL_MODELS = window.KEEL_MODELS || {{}})[{}] = {};\n",
        robot_file.display(),
        scene.file_name().map_or(String::new(), |f| f.to_string_lossy().into_owned()),
        serde_json::to_string(name)?,
        serde_json::to_string(&data)?
    );
    fs::write(out, &script)?;
    eprintln!("wrote {}: {} bodies, {} meshes, {} KB", out.display(), bodies.len(), meshes.len(), script.len() / 1024);
    Ok(())
}

fn body(node: roxmltree::Node, parent: Option<usize>, bodies: &mut Vec<Value>, used: &mut Vec<String>) -> Result<()> {
    let name = node.attribute("name").unwrap_or("").to_owned();
    for unread in ["euler", "axisangle", "xyaxes", "zaxis"] {
        if node.attribute(unread).is_some() {
            return Err(format!("body {name}: `{unread}` isn't read, only pos and quat").into());
        }
    }
    let joints: Vec<_> = node.children().filter(|c| c.has_tag_name("joint")).collect();
    if joints.len() > 1 {
        return Err(format!("body {name}: more than one joint").into());
    }
    let joint = match joints.first() {
        Some(j) => {
            let kind = j.attribute("type").unwrap_or("hinge");
            if kind != "hinge" && kind != "slide" {
                return Err(format!("body {name}: a {kind} joint isn't read").into());
            }
            json!({ "name": j.attribute("name").unwrap_or(""), "type": kind, "axis": floats(j.attribute("axis").unwrap_or("0 0 1"))? })
        }
        None => Value::Null,
    };
    let mut geoms = Vec::new();
    for g in node.children().filter(|c| c.has_tag_name("geom") && c.attribute("class") == Some("collision")) {
        let mesh = g.attribute("mesh").ok_or_else(|| format!("body {name}: a collision geom that isn't a mesh"))?;
        used.push(mesh.to_owned());
        geoms.push(json!({
            "mesh": mesh,
            "pos": floats(g.attribute("pos").unwrap_or("0 0 0"))?,
            "quat": floats(g.attribute("quat").unwrap_or("1 0 0 0"))?,
        }));
    }
    let me = bodies.len();
    bodies.push(json!({
        "name": name,
        "parent": parent,
        "pos": floats(node.attribute("pos").unwrap_or("0 0 0"))?,
        "quat": floats(node.attribute("quat").unwrap_or("1 0 0 0"))?,
        "joint": joint,
        "geoms": geoms,
    }));
    for b in node.children().filter(|c| c.has_tag_name("body")) {
        body(b, Some(me), bodies, used)?;
    }
    Ok(())
}

fn child<'a, 'i>(n: roxmltree::Node<'a, 'i>, tag: &str) -> Option<roxmltree::Node<'a, 'i>> {
    n.children().find(|c| c.has_tag_name(tag))
}

fn floats(s: &str) -> Result<Vec<f64>> {
    Ok(s.split_whitespace().map(str::parse).collect::<std::result::Result<_, _>>()?)
}

/// A binary STL's triangles.
fn stl(path: &Path) -> Result<Vec<[[f32; 3]; 3]>> {
    let b = fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let n = b.get(80..84).map(|c| u32::from_le_bytes(c.try_into().unwrap()) as usize).unwrap_or(0);
    if b.len() != 84 + 50 * n {
        return Err(format!("{}: not a binary STL", path.display()).into());
    }
    let f = |at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
    Ok((0..n)
        .map(|t| {
            let at = 84 + 50 * t + 12;
            [0, 1, 2].map(|v| [0, 1, 2].map(|c| f(at + v * 12 + c * 4)))
        })
        .collect())
}

/// Scaled, its vertices merged on a grid, the triangles that collapse
/// dropped. Vertices as 16-bit cell indexes from the mesh's corner, in
/// base64, with the corner and the cell size; triangles as indexes.
fn thin(tris: &[[[f32; 3]; 3]], scale: &[f64]) -> Result<Value> {
    let p = |v: &[f32; 3]| [v[0] as f64 * scale[0], v[1] as f64 * scale[1], v[2] as f64 * scale[2]];
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    for v in tris.iter().flatten().map(p) {
        for c in 0..3 {
            lo[c] = lo[c].min(v[c]);
            hi[c] = hi[c].max(v[c]);
        }
    }
    let size = (0..3).map(|c| hi[c] - lo[c]).fold(0.0, f64::max);
    let cell = (size / CELLS).max(1e-4);
    let mut index: HashMap<[u16; 3], u32> = HashMap::new();
    let mut verts: Vec<u16> = Vec::new();
    let mut faces: Vec<u32> = Vec::new();
    let mut seen = std::collections::HashSet::new();
    for t in tris {
        let ids = t.map(|v| {
            let q = p(&v);
            let k = [0, 1, 2].map(|c| ((q[c] - lo[c]) / cell).round() as u16);
            *index.entry(k).or_insert_with(|| {
                verts.extend_from_slice(&k);
                (verts.len() / 3 - 1) as u32
            })
        });
        if ids[0] == ids[1] || ids[1] == ids[2] || ids[0] == ids[2] {
            continue;
        }
        let mut key = ids;
        key.sort();
        if seen.insert(key) {
            faces.extend_from_slice(&ids);
        }
    }
    let wide = verts.len() / 3 > u16::MAX as usize;
    let face_bytes: Vec<u8> = if wide {
        faces.iter().flat_map(|i| i.to_le_bytes()).collect()
    } else {
        faces.iter().flat_map(|&i| (i as u16).to_le_bytes()).collect()
    };
    Ok(json!({
        "min": lo, "cell": cell,
        "v": base64(&verts.iter().flat_map(|x| x.to_le_bytes()).collect::<Vec<_>>()),
        "f": base64(&face_bytes), "f32": wide,
    }))
}

fn base64(bytes: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for c in bytes.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for k in 0..4 {
            s.push(if k <= c.len() { A[(n >> (18 - 6 * k) & 63) as usize] as char } else { '=' });
        }
    }
    s
}
