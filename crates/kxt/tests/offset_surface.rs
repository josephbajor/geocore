#![allow(
    deprecated,
    reason = "interchange compatibility coverage retains the deprecated v1 tessellation wrapper"
)]

//! G4a single-offset X_T transport contract.

use kcore::error::ErrorClass;
use kcore::tolerance::Tolerances;
use kgeom::curve::Line;
use kgeom::curve2d::Line2d;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::{EvalLimits, GeometryRef, OffsetSurfaceDescriptor, SurfaceDerivativeOrder};
use ktopo::btess::{TessOptions, tessellate_body};
use ktopo::check::{CheckLevel, CheckOutcome, VerificationGapKind, check_body_report};
use ktopo::entity::{
    Body, Edge, Face, Fin, FinPcurve, Loop, ParamMap1d, Region, Sense, Shell, SurfaceId, Vertex,
};
use ktopo::euler::FinPcurvePair;
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::make;
use ktopo::store::Store;
use kxt::parse::{Value, XtFile};
use kxt::schema::code;
use kxt::{XtError, export_text, import, read_xt, reconstruct};

const DISTANCE: f64 = 0.25;

fn offset_sheet() -> (Store, ktopo::entity::BodyId, SurfaceId, SurfaceId) {
    let mut store = Store::new();
    let frame = Frame::new(
        Point3::new(0.0, 0.0, DISTANCE),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let body = make::planar_sheet(
        &mut store,
        &frame,
        &[
            Point2::new(0.0, 0.0),
            Point2::new(2.0, 0.0),
            Point2::new(2.0, 1.0),
            Point2::new(0.0, 1.0),
        ],
    )
    .unwrap();
    let offset = store.faces_of_body(body).unwrap()[0];
    let offset = store.get(offset).unwrap().surface;
    let mut transaction = store.transaction().unwrap();
    let basis = {
        let mut assembly = transaction.assembly();
        let basis = assembly
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        assembly
            .replace_surface(
                offset,
                SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(basis, DISTANCE)),
            )
            .unwrap();
        basis
    };
    transaction.commit_checked_body(body).unwrap();
    (store, body, offset, basis)
}

fn node_indices(file: &XtFile, node_code: u16) -> Vec<u32> {
    file.nodes
        .iter()
        .filter_map(|(&index, node)| (node.code == node_code).then_some(index))
        .collect()
}

fn set_field(file: &mut XtFile, index: u32, name: &str, value: Value) {
    let node_code = file.nodes[&index].code;
    let field = file.defs[&node_code].field_index(name).unwrap();
    file.nodes.get_mut(&index).unwrap().values[field] = value;
}

fn reflect_authored_geometry_below_basis(file: &mut XtFile) {
    for node in file.nodes.values_mut() {
        for value in &mut node.values {
            if let Value::Vector(Some(vector)) = value
                && vector[2] == DISTANCE
            {
                vector[2] = -DISTANCE;
            }
        }
    }
}

fn offset_descriptor(store: &Store) -> (SurfaceId, OffsetSurfaceDescriptor) {
    store
        .geometry()
        .surfaces()
        .find_map(|(surface, descriptor)| descriptor.as_offset().copied().map(|d| (surface, d)))
        .unwrap()
}

fn store_counts(store: &Store) -> [usize; 12] {
    [
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Loop>(),
        store.count::<Fin>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
        store.count::<Point3>(),
        store.count::<ktopo::geom::CurveGeom>(),
        store.count::<SurfaceGeom>(),
        store.count::<ktopo::geom::Curve2dGeom>(),
    ]
}

fn assert_fin_pcurves_agree_with_edges(store: &Store, body: ktopo::entity::BodyId) {
    for fin_id in store
        .faces_of_body(body)
        .unwrap()
        .into_iter()
        .flat_map(|face| store.get(face).unwrap().loops.clone())
        .flat_map(|loop_id| store.get(loop_id).unwrap().fins.clone())
    {
        let fin = store.get(fin_id).unwrap();
        let pcurve = fin.pcurve.expect("offset-face pcurve retained");
        assert!(pcurve.chart().is_identity());
        let edge = store.get(fin.edge).unwrap();
        let (t0, t1) = edge.bounds.unwrap();
        let edge_curve = store.get(edge.curve.unwrap()).unwrap().as_curve();
        let face = store.get(store.get(fin.parent).unwrap().face).unwrap();
        let pcurve_geom = store.get(pcurve.curve()).unwrap().as_curve();
        let mut eval = store.eval_context(EvalLimits::default(), Tolerances::default());
        for t in [t0, 0.5 * (t0 + t1), t1] {
            let q = pcurve.parameter_at_edge(t);
            assert_eq!(pcurve.edge_to_pcurve().inverse(q), t);
            let uv = pcurve_geom.eval(q);
            let surface_point = eval
                .eval_surface(face.surface, [uv.x, uv.y], SurfaceDerivativeOrder::Position)
                .unwrap()
                .p;
            let edge_point = edge_curve.eval(t);
            assert!(
                surface_point.dist(edge_point) <= 32.0 * f64::EPSILON,
                "fin {fin_id:?} map disagrees at edge parameter {t}: {surface_point:?} vs {edge_point:?}"
            );
        }
    }
}

/// Re-pin the committed fixture after an intentional writer change:
/// `cargo test -p kxt --test offset_surface -- --ignored regenerate`.
/// Fixture re-pins are reviewed events; the byte-stable test below is the
/// gate that makes accidental drift fail.
#[test]
#[ignore = "rewrites the committed canonical fixture"]
fn regenerate_offset_plane_fixture() {
    let (store, body, _, _) = offset_sheet();
    std::fs::write(
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/offset_plane.x_t"
        ),
        export_text(&store, body).unwrap(),
    )
    .unwrap();
}

#[test]
fn single_offset_round_trip_is_class_preserving_and_byte_stable() {
    let (store, body, offset, basis) = offset_sheet();
    let text = export_text(&store, body).unwrap();
    assert_eq!(text, export_text(&store, body).unwrap());
    assert_eq!(
        text.as_bytes(),
        include_bytes!("fixtures/offset_plane.x_t"),
        "the committed synthetic fixture is the canonical writer output"
    );

    let file = read_xt(text.as_bytes()).unwrap();
    let planes = node_indices(&file, code::PLANE);
    let offsets = node_indices(&file, code::OFFSET_SURF);
    assert_eq!(planes.len(), 1);
    assert_eq!(offsets.len(), 1);
    assert!(planes[0] < offsets[0], "basis must be emitted first");
    let node = &file.nodes[&offsets[0]];
    assert_eq!(file.field(node, "sense"), Some(&Value::Char('+')));
    assert_eq!(file.field(node, "check"), Some(&Value::Char('U')));
    assert_eq!(
        file.field(node, "true_offset"),
        Some(&Value::Logical(false))
    );
    assert_eq!(file.field(node, "surface"), Some(&Value::Ptr(planes[0])));
    assert_eq!(file.field(node, "offset"), Some(&Value::Double(DISTANCE)));
    assert_eq!(file.field(node, "scale"), Some(&Value::Null));

    let mut imported = Store::new();
    let reconstruction = import(text.as_bytes(), &mut imported).unwrap();
    let imported_body = reconstruction.bodies[0];
    let (imported_offset, descriptor) = offset_descriptor(&imported);
    assert_eq!(descriptor.signed_distance(), DISTANCE);
    assert_ne!(imported_offset, descriptor.basis());
    assert_eq!(imported.count::<SurfaceGeom>(), 2);
    assert!(imported.iter::<Fin>().all(|(_, fin)| fin.pcurve.is_some()));
    assert_fin_pcurves_agree_with_edges(&imported, imported_body);

    let mut eval = imported.eval_context(EvalLimits::default(), Tolerances::default());
    let point = eval
        .eval_surface(
            imported_offset,
            [0.5, 0.5],
            SurfaceDerivativeOrder::Position,
        )
        .unwrap()
        .p;
    assert_eq!(point, Point3::new(0.5, 0.5, DISTANCE));
    let fast = check_body_report(&imported, imported_body, CheckLevel::Fast).unwrap();
    assert_eq!(fast.outcome(), CheckOutcome::Valid);
    let full = check_body_report(&imported, imported_body, CheckLevel::Full).unwrap();
    assert_eq!(full.outcome(), CheckOutcome::Indeterminate);
    assert!(
        full.gaps
            .iter()
            .any(|gap| gap.kind == VerificationGapKind::SurfaceRegularity)
    );
    let mesh = tessellate_body(
        &imported,
        imported_body,
        &TessOptions {
            chord_tol: 1e-4,
            max_edge_len: Some(0.2),
        },
    )
    .unwrap();
    assert!(mesh.positions.iter().all(|point| point.z == DISTANCE));
    assert_eq!(export_text(&imported, imported_body).unwrap(), text);

    // Source graph identity was not mutated by planning.
    assert_eq!(
        store.get(offset).unwrap().as_offset().unwrap().basis(),
        basis
    );
}

#[test]
fn minus_sense_negates_transport_offset_and_u_v_checks_are_permissive() {
    let (store, body, _, _) = offset_sheet();
    let text = export_text(&store, body).unwrap();
    for check in ['U', 'V'] {
        let mut file = read_xt(text.as_bytes()).unwrap();
        let offset = node_indices(&file, code::OFFSET_SURF)[0];
        let basis = file
            .field(&file.nodes[&offset], "surface")
            .unwrap()
            .as_ptr()
            .unwrap();
        set_field(&mut file, offset, "sense", Value::Char('-'));
        set_field(&mut file, basis, "sense", Value::Char('-'));
        set_field(&mut file, offset, "check", Value::Char(check));
        if check == 'V' {
            set_field(&mut file, offset, "true_offset", Value::Logical(true));
            set_field(&mut file, offset, "scale", Value::Double(37.0));
        }
        reflect_authored_geometry_below_basis(&mut file);
        let mut imported = Store::new();
        let reconstruction = reconstruct(&file, &mut imported).unwrap();
        let (_, descriptor) = offset_descriptor(&imported);
        assert_eq!(descriptor.signed_distance(), -DISTANCE);
        let canonical = export_text(&imported, reconstruction.bodies[0]).unwrap();
        assert_eq!(
            canonical,
            export_text(&imported, reconstruction.bodies[0]).unwrap()
        );
        let canonical = read_xt(canonical.as_bytes()).unwrap();
        let canonical_offset = node_indices(&canonical, code::OFFSET_SURF)[0];
        assert_eq!(
            canonical.field(&canonical.nodes[&canonical_offset], "sense"),
            Some(&Value::Char('+'))
        );
        assert_eq!(
            canonical.field(&canonical.nodes[&canonical_offset], "offset"),
            Some(&Value::Double(-DISTANCE))
        );
    }
}

#[test]
fn unsupported_offset_writer_shapes_have_precise_public_capabilities() {
    let (mut nested, nested_body, outer, basis) = offset_sheet();
    let inner = nested
        .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
            basis, 0.1,
        )))
        .unwrap();
    let mut transaction = nested.transaction().unwrap();
    transaction
        .assembly()
        .replace_surface(
            outer,
            SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(inner, 0.15)),
        )
        .unwrap();
    transaction.commit_checked_body(nested_body).unwrap();
    let nested_error = export_text(&nested, nested_body).unwrap_err();
    assert!(
        matches!(
            &nested_error,
            XtError::Unsupported {
                capability: kxt::XtCapability::NestedOffsetExport,
                ..
            }
        ),
        "{nested_error:?}"
    );

    let (mut shared, shared_body, first_offset, shared_basis) = offset_sheet();
    let face = shared.faces_of_body(shared_body).unwrap()[0];
    let loop_id = shared.get(face).unwrap().loops[0];
    let start = Point3::new(0.0, 0.0, DISTANCE);
    let direction = Vec3::new(2.0, 1.0, 0.0);
    let length = direction.norm();
    let diagonal = shared
        .insert_curve(CurveGeom::Line(Line::new(start, direction).unwrap()))
        .unwrap();
    let make_pcurve = |store: &mut Store| {
        let curve = store
            .insert_pcurve(Curve2dGeom::Line(
                Line2d::new(Point2::new(0.0, 0.0), Vec2::new(2.0, 1.0)).unwrap(),
            ))
            .unwrap();
        FinPcurve::new(curve, ParamRange::new(0.0, length), ParamMap1d::identity()).unwrap()
    };
    let pcurves = FinPcurvePair::new(make_pcurve(&mut shared), make_pcurve(&mut shared));
    let mut split = shared.transaction().unwrap();
    let made = split
        .split_face(
            loop_id,
            0,
            2,
            diagonal,
            (0.0, length),
            first_offset,
            Sense::Forward,
            pcurves,
        )
        .unwrap();
    split.commit_checked_body(shared_body).unwrap();
    let second_offset = shared
        .insert_surface(SurfaceGeom::Offset(OffsetSurfaceDescriptor::new(
            shared_basis,
            DISTANCE,
        )))
        .unwrap();
    let mut replace = shared.transaction().unwrap();
    replace.assembly().get_mut(made.face).unwrap().surface = second_offset;
    replace.commit_checked_body(shared_body).unwrap();
    let shared_error = export_text(&shared, shared_body).unwrap_err();
    assert!(
        matches!(
            &shared_error,
            XtError::Unsupported {
                capability: kxt::XtCapability::SharedOffsetBasisExport,
                ..
            }
        ),
        "{shared_error:?}"
    );
}

#[test]
fn malformed_offset_inputs_are_typed_and_failure_atomic() {
    let (source, body, _, _) = offset_sheet();
    let text = export_text(&source, body).unwrap();
    for mutation in ["cycle", "two_cycle", "sense", "check", "zero"] {
        let mut file = read_xt(text.as_bytes()).unwrap();
        let offset = node_indices(&file, code::OFFSET_SURF)[0];
        let expected_cycle = match mutation {
            "cycle" => {
                set_field(&mut file, offset, "surface", Value::Ptr(offset));
                Some(vec![offset, offset])
            }
            "two_cycle" => {
                let other = file.nodes.keys().next_back().copied().unwrap() + 1;
                file.nodes.insert(other, file.nodes[&offset].clone());
                set_field(&mut file, offset, "surface", Value::Ptr(other));
                set_field(&mut file, other, "surface", Value::Ptr(offset));
                Some(vec![offset, other, offset])
            }
            "sense" => {
                set_field(&mut file, offset, "sense", Value::Char('-'));
                None
            }
            "check" => {
                set_field(&mut file, offset, "check", Value::Char('I'));
                None
            }
            "zero" => {
                set_field(&mut file, offset, "offset", Value::Double(0.0));
                None
            }
            _ => unreachable!(),
        };
        let mut target = Store::new();
        let sentinel = make::planar_sheet(
            &mut target,
            &Frame::world(),
            &[
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(0.0, 1.0),
            ],
        )
        .unwrap();
        let mut control = target.clone();
        let before = store_counts(&target);
        let error = reconstruct(&file, &mut target).unwrap_err();
        assert_eq!(store_counts(&target), before);
        assert!(target.get(sentinel).is_ok());
        let target_descriptors: Vec<_> = target
            .geometry()
            .surfaces()
            .map(|(_, descriptor)| descriptor.clone())
            .collect();
        let control_descriptors: Vec<_> = control
            .geometry()
            .surfaces()
            .map(|(_, descriptor)| descriptor.clone())
            .collect();
        assert_eq!(target_descriptors, control_descriptors);
        for (surface, _) in target.geometry().surfaces() {
            assert!(
                target
                    .geometry()
                    .dependents(GeometryRef::Surface(surface))
                    .unwrap()
                    .is_empty()
            );
        }
        let target_next = target
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        let control_next = control
            .insert_surface(SurfaceGeom::Plane(Plane::new(Frame::world())))
            .unwrap();
        assert_eq!(
            target_next, control_next,
            "failed import consumed a graph slot"
        );
        match mutation {
            "cycle" | "two_cycle" => {
                assert_eq!(error.code(), kxt::error::code::SURFACE_DEPENDENCY_CYCLE);
                assert_eq!(error.class(), ErrorClass::InvalidInput);
                let XtError::SurfaceDependencyCycle { path } = error else {
                    unreachable!()
                };
                assert_eq!(path, expected_cycle.unwrap());
            }
            _ => assert!(matches!(error, XtError::BadField { .. })),
        }
    }
}

#[test]
fn evaluation_errors_preserve_source_and_classification() {
    use std::error::Error as _;

    let error = XtError::Evaluation(kgraph::EvalError::NodeVisitLimitExceeded {
        consumed: 2,
        limit: 1,
    });
    assert_eq!(error.class(), ErrorClass::ResourceLimit);
    assert_eq!(
        error.limit(),
        Some(kcore::operation::LimitSnapshot {
            stage: kgraph::eval_stage::NODE_VISITS,
            resource: kcore::operation::ResourceKind::Work,
            consumed: 2,
            allowed: 1,
        })
    );
    assert!(error.source().unwrap().is::<kgraph::EvalError>());
    let unsupported = XtError::Evaluation(kgraph::EvalError::DerivativeUnavailable {
        class: kgraph::SurfaceClass::Offset.key(),
        requested: 2,
    });
    assert_eq!(unsupported.class(), ErrorClass::Unsupported);
    assert_eq!(
        unsupported.capability_id(),
        Some(kgraph::eval_capability::DERIVATIVE_ORDER)
    );
    assert_eq!(
        XtError::Evaluation(kgraph::EvalError::InvalidParameter).class(),
        ErrorClass::InvalidInput
    );
    assert_eq!(
        XtError::Evaluation(kgraph::EvalError::NonFiniteResult {
            class: kgraph::SurfaceClass::Offset.key(),
        })
        .class(),
        ErrorClass::InternalInvariant
    );
}
