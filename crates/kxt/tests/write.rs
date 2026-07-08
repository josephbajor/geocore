//! M3b writer round trips for every self-authored analytic primitive.

use kgeom::frame::Frame;
use kgeom::vec::{Point3, Vec3};
use ktopo::btess::{TessOptions, check_watertight, tessellate_body};
use ktopo::check::check_body;
use ktopo::entity::{BodyId, Edge, Face, Vertex};
use ktopo::make;
use ktopo::store::Store;

fn tilted() -> Frame {
    Frame::new(
        Point3::new(0.3, -1.2, 2.1),
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
    )
    .unwrap()
}

fn assert_roundtrip(store: &Store, body: BodyId) {
    let text = kxt::export_text(store, body).unwrap();
    assert_eq!(text, kxt::export_text(store, body).unwrap());
    let parsed = kxt::read_xt(text.as_bytes()).unwrap();
    assert_eq!(parsed.schema, "SCH_1300000_13006");
    assert_eq!(parsed.usfld_size, 0);

    let mut imported = Store::new();
    let recon = kxt::import(text.as_bytes(), &mut imported).unwrap();
    assert_eq!(recon.bodies.len(), 1);
    let imported_body = recon.bodies[0];
    let faults = check_body(&imported, imported_body).unwrap();
    assert!(faults.is_empty(), "round-trip checker faults: {faults:?}");
    assert_eq!(store.count::<Face>(), imported.count::<Face>());
    assert_eq!(store.count::<Edge>(), imported.count::<Edge>());
    assert_eq!(store.count::<Vertex>(), imported.count::<Vertex>());

    let mesh = tessellate_body(
        &imported,
        imported_body,
        &TessOptions {
            chord_tol: 1e-3,
            max_edge_len: None,
        },
    )
    .unwrap();
    assert!(check_watertight(&mesh).is_empty());
}

#[test]
fn all_analytic_primitives_round_trip() {
    let frame = tilted();
    let constructors: [fn(&mut Store, &Frame) -> BodyId; 6] = [
        |store, frame| make::block(store, frame, [0.4, 0.3, 0.2]).unwrap(),
        |store, frame| make::cylinder(store, frame, 0.2, 0.5).unwrap(),
        |store, frame| make::cone(store, frame, 0.2, 0.35, 0.5).unwrap(),
        |store, frame| make::cone(store, frame, 0.35, 0.2, 0.5).unwrap(),
        |store, frame| make::sphere(store, frame, 0.25).unwrap(),
        |store, frame| make::torus(store, frame, 0.4, 0.1).unwrap(),
    ];
    for constructor in constructors {
        let mut store = Store::new();
        let body = constructor(&mut store, &frame);
        assert_roundtrip(&store, body);
    }
}
