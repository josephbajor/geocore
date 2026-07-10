//! Integration tests: parse and reconstruct the fixture corpus.
//!
//! `block.*` are hand-authored at exact schema 13006 (both encodings);
//! `sphere/disk_nat/plate` are real-world files written by Parasolid V27
//! and V28 (embedded schemas over base 13006); `longbar` is a V10 file
//! that must be rejected.

use kgeom::frame::Frame;
use kgeom::vec::Point3;
use ktopo::check::check_body;
use ktopo::entity::{Body, BodyKind, Edge, Face, Region, Shell, Vertex};
use ktopo::make::block;
use ktopo::store::Store;
use ktopo::transaction::MutationKind;
use kxt::parse::Value;
use kxt::schema::code;
use kxt::{XtError, import, read_xt, reconstruct};

fn fixture(name: &str) -> Vec<u8> {
    let path = format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
    std::fs::read(&path).unwrap_or_else(|e| panic!("reading fixture {path}: {e}"))
}

/// Import one fixture expecting a single body; run the checker on it.
fn import_one(name: &str) -> (Store, ktopo::entity::BodyId) {
    let mut store = Store::new();
    let recon = import(&fixture(name), &mut store).unwrap_or_else(|e| {
        panic!("importing {name}: {e}");
    });
    assert_eq!(recon.bodies.len(), 1, "{name}: expected one body");
    (store, recon.bodies[0])
}

#[test]
fn hand_authored_block_text_reconstructs_checker_clean() {
    let (store, body) = import_one("block.x_t");
    assert_eq!(store.get(body).unwrap().kind, BodyKind::Solid);
    assert_eq!(store.faces_of_body(body).unwrap().len(), 6);
    assert_eq!(store.edges_of_body(body).unwrap().len(), 12);
    assert_eq!(store.vertices_of_body(body).unwrap().len(), 8);
    assert_eq!(store.count::<Vertex>(), 8);
    assert_eq!(store.count::<Edge>(), 12);
    assert_eq!(store.count::<Face>(), 6);
    // Every edge is bounded, on a line, with both vertices.
    for e in store.edges_of_body(body).unwrap() {
        let edge = store.get(e).unwrap();
        assert!(edge.curve.is_some());
        let (t0, t1) = edge.bounds.expect("line edges are bounded");
        assert!(t1 > t0);
        assert_eq!(edge.fins.len(), 2);
    }
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "block.x_t faults: {faults:?}");
}

#[test]
fn successful_reconstruction_exposes_its_atomic_mutation_journal() {
    let mut store = Store::new();
    let reconstruction = import(&fixture("block.x_t"), &mut store).unwrap();
    let expected_created = store.count::<Body>()
        + store.count::<Region>()
        + store.count::<Shell>()
        + store.count::<Face>()
        + store.count::<ktopo::entity::Loop>()
        + store.count::<ktopo::entity::Fin>()
        + store.count::<Edge>()
        + store.count::<Vertex>()
        + store.count::<ktopo::geom::CurveGeom>()
        + store.count::<ktopo::geom::SurfaceGeom>()
        + store.count::<Point3>()
        + store.count::<ktopo::geom::Curve2dGeom>();
    assert_eq!(reconstruction.journal.mutations().len(), expected_created);
    assert!(
        reconstruction
            .journal
            .mutations()
            .iter()
            .all(|mutation| mutation.kind == MutationKind::Created)
    );
    assert!(reconstruction.journal.lineage().is_empty());
}

#[test]
fn neutral_binary_block_matches_text_block() {
    let (ts, tb) = import_one("block.x_t");
    let (bs, bb) = import_one("block.x_b");
    // Same topology counts and bit-identical geometry.
    assert_eq!(ts.count::<Face>(), bs.count::<Face>());
    assert_eq!(ts.count::<Edge>(), bs.count::<Edge>());
    let tv: Vec<_> = ts
        .vertices_of_body(tb)
        .unwrap()
        .into_iter()
        .map(|v| ts.vertex_position(v).unwrap())
        .collect();
    let bv: Vec<_> = bs
        .vertices_of_body(bb)
        .unwrap()
        .into_iter()
        .map(|v| bs.vertex_position(v).unwrap())
        .collect();
    assert_eq!(tv.len(), 8);
    for (a, b) in tv.iter().zip(&bv) {
        assert_eq!(a.x.to_bits(), b.x.to_bits());
        assert_eq!(a.y.to_bits(), b.y.to_bits());
        assert_eq!(a.z.to_bits(), b.z.to_bits());
    }
    let faults = check_body(&bs, bb).unwrap();
    assert!(faults.is_empty(), "block.x_b faults: {faults:?}");
}

#[test]
fn real_world_cut_sphere_v27_reconstructs() {
    let (store, body) = import_one("sphere.x_t");
    let b: &Body = store.get(body).unwrap();
    assert_eq!(b.kind, BodyKind::Solid);
    let faces = store.faces_of_body(body).unwrap();
    assert_eq!(faces.len(), 2, "cut sphere: spherical face + planar cap");
    let edges = store.edges_of_body(body).unwrap();
    assert_eq!(edges.len(), 1, "one circular rim");
    let rim = store.get(edges[0]).unwrap();
    assert_eq!(rim.vertices, [None, None], "rim is a ring edge");
    assert!(rim.bounds.is_none());
    assert_eq!(rim.fins.len(), 2);
    // Every face carries live surface geometry.
    for f in faces {
        let face = store.get(f).unwrap();
        assert!(store.get(face.surface).is_ok());
    }
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "sphere.x_t faults: {faults:?}");
}

#[test]
fn real_world_sheet_disk_v27_reconstructs() {
    let (store, body) = import_one("disk_nat.x_t");
    assert_eq!(store.get(body).unwrap().kind, BodyKind::Sheet);
    let faces = store.faces_of_body(body).unwrap();
    assert_eq!(faces.len(), 1);
    let edges = store.edges_of_body(body).unwrap();
    assert_eq!(edges.len(), 1);
    assert_eq!(store.get(edges[0]).unwrap().fins.len(), 1);
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "disk_nat.x_t faults: {faults:?}");
}

#[test]
fn real_world_plate_v28_reconstructs() {
    let (store, body) = import_one("plate.x_t");
    assert_eq!(store.get(body).unwrap().kind, BodyKind::Solid);
    let faces = store.faces_of_body(body).unwrap();
    assert!(!faces.is_empty());
    for f in faces {
        let face = store.get(f).unwrap();
        assert!(store.get(face.surface).is_ok());
    }
    let faults = check_body(&store, body).unwrap();
    assert!(faults.is_empty(), "plate.x_t faults: {faults:?}");
}

#[test]
fn pre_13006_files_are_rejected_with_unsupported_schema() {
    let mut store = Store::new();
    match import(&fixture("longbar.x_t"), &mut store) {
        Err(XtError::UnsupportedSchema { schema }) => {
            assert_eq!(schema, "SCH_1000230_10004");
        }
        other => panic!("expected UnsupportedSchema, got {other:?}"),
    }
}

#[test]
fn header_and_node_graph_are_exposed() {
    let file = read_xt(&fixture("sphere.x_t")).unwrap();
    assert_eq!(file.header.get("FORMAT"), Some("text"));
    assert_eq!(file.schema, "SCH_2700142_26105_13006");
    assert_eq!(file.usfld_size, 1);
    assert!(file.nodes.len() > 20);
}

#[test]
fn reconstruction_failure_leaves_store_unchanged() {
    let mut store = Store::new();
    let existing = block(&mut store, &Frame::world(), [1.0, 2.0, 3.0]).unwrap();
    let before = (
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
    );
    let mut control = store.clone();

    let mut file = read_xt(&fixture("block.x_t")).unwrap();
    let point_index = file
        .nodes
        .iter()
        .rev()
        .find(|(_, node)| node.code == code::POINT)
        .map(|(&index, _)| index)
        .unwrap();
    let point = file.nodes.get_mut(&point_index).unwrap();
    let def = file.defs.get(&code::POINT).unwrap();
    point.values[def.field_index("pvec").unwrap()] = Value::Vector(Some([501.0, 0.0, 0.0]));

    assert!(matches!(
        reconstruct(&file, &mut store),
        Err(XtError::OutsideSizeBox { value: 501.0 })
    ));
    assert_eq!(
        before,
        (
            store.count::<Body>(),
            store.count::<Region>(),
            store.count::<Shell>(),
            store.count::<Face>(),
            store.count::<Edge>(),
            store.count::<Vertex>(),
        )
    );
    assert!(store.get(existing).is_ok());
    assert!(check_body(&store, existing).unwrap().is_empty());
    assert_eq!(
        store.add(Point3::new(4.0, 5.0, 6.0)),
        control.add(Point3::new(4.0, 5.0, 6.0))
    );
}
