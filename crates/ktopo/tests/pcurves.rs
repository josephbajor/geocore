//! Integration coverage for per-fin parameter-space curves.

use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Vec2};
use ktopo::check::{FaultKind, check_body};
use ktopo::entity::{FinPcurve, ParamMap1d};
use ktopo::geom::Curve2dGeom;
use ktopo::make::{block, cone, cylinder};
use ktopo::store::Store;

#[test]
fn authored_primitives_carry_checker_verified_pcurves() {
    let mut store = Store::new();
    let bodies = [
        block(&mut store, &Frame::world(), [2.0, 3.0, 4.0]).unwrap(),
        cylinder(&mut store, &Frame::world(), 1.25, 2.5).unwrap(),
        cone(&mut store, &Frame::world(), 1.0, 0.4, 2.0).unwrap(),
        cone(&mut store, &Frame::world(), 0.4, 1.0, 2.0).unwrap(),
    ];

    for body in bodies {
        assert!(check_body(&store, body).unwrap().is_empty());
        for edge in store.edges_of_body(body).unwrap() {
            for &fin in &store.get(edge).unwrap().fins {
                assert!(
                    store.get(fin).unwrap().pcurve.is_some(),
                    "authored face-edge uses must have explicit pcurves"
                );
            }
        }
    }
}

#[test]
fn the_two_uses_of_a_shared_edge_have_independent_pcurves() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    for edge in store.edges_of_body(body).unwrap() {
        let fins = &store.get(edge).unwrap().fins;
        assert_eq!(fins.len(), 2);
        let a = store.get(fins[0]).unwrap().pcurve.unwrap().curve();
        let b = store.get(fins[1]).unwrap().pcurve.unwrap().curve();
        assert_ne!(a, b, "each face use owns its own UV representation");
    }
}

#[test]
fn checker_rejects_a_parameter_map_that_does_not_cover_the_edge() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let edge = store.edges_of_body(body).unwrap()[0];
    let fin_id = store.get(edge).unwrap().fins[0];
    let current = store.get(fin_id).unwrap().pcurve.unwrap();
    let truncated = FinPcurve::new(
        current.curve(),
        ParamRange::new(0.0, current.range().hi / 2.0),
        ParamMap1d::identity(),
    )
    .unwrap();
    store.get_mut(fin_id).unwrap().pcurve = Some(truncated);

    let faults = check_body(&store, body).unwrap();
    assert!(faults.iter().any(|fault| {
        fault.entity == ktopo::entity::EntityRef::Fin(fin_id)
            && fault.kind == FaultKind::BadPcurveRange
    }));
}

#[test]
fn checker_rejects_a_pcurve_that_lifts_off_the_edge() {
    let mut store = Store::new();
    let body = block(&mut store, &Frame::world(), [2.0, 2.0, 2.0]).unwrap();
    let edge = store.edges_of_body(body).unwrap()[0];
    let fin_id = store.get(edge).unwrap().fins[0];
    let pcurve_id = store.get(fin_id).unwrap().pcurve.unwrap().curve();
    *store.get_mut(pcurve_id).unwrap() =
        Curve2dGeom::Line(Line2d::new(Point2::new(100.0, 100.0), Vec2::new(1.0, 0.0)).unwrap());

    let faults = check_body(&store, body).unwrap();
    assert!(faults.iter().any(|fault| {
        fault.entity == ktopo::entity::EntityRef::Fin(fin_id)
            && fault.kind == FaultKind::PcurveOffSurface
    }));
}

#[test]
fn reversed_affine_correspondence_is_invertible_and_explicit() {
    let map = ParamMap1d::affine(-2.0, 7.0).unwrap();
    assert_eq!(map.sense(), ktopo::entity::Sense::Reversed);
    let q = map.map(1.25);
    assert_eq!(map.inverse(q), 1.25);
    assert!(ParamMap1d::affine(0.0, 0.0).is_err());
}
