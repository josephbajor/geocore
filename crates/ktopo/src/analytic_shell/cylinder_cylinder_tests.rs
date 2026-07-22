use super::assemble::AnalyticShellAssemblyError;
use super::*;
use crate::check::CheckOutcome;
use crate::geom::{CurveGeom, SurfaceGeom};
use crate::transaction::FullCommitRequirement;
use kcore::error::Error;
use kgeom::curve::{Circle, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

const TOLERANCE: f64 = 1.0e-12;
const V0: AnalyticVertexKey = AnalyticVertexKey::new(0);
const V1: AnalyticVertexKey = AnalyticVertexKey::new(1);
const V2: AnalyticVertexKey = AnalyticVertexKey::new(2);
const V3: AnalyticVertexKey = AnalyticVertexKey::new(3);
const E0: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const E1: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const E2: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
const E3: AnalyticEdgeKey = AnalyticEdgeKey::new(3);
const E4: AnalyticEdgeKey = AnalyticEdgeKey::new(4);
const E5: AnalyticEdgeKey = AnalyticEdgeKey::new(5);
const E6: AnalyticEdgeKey = AnalyticEdgeKey::new(6);
const E7: AnalyticEdgeKey = AnalyticEdgeKey::new(7);

#[derive(Clone, Copy)]
struct LensGeometry {
    angle: f64,
    cylinders: [Cylinder; 2],
    circles: [Circle; 4],
    points: [Point3; 4],
}

fn map(scale: f64, offset: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, offset).unwrap()
}

fn lens_geometry() -> LensGeometry {
    let angle = core::f64::consts::PI / 3.0;
    let first_frame = Frame::world();
    let second_frame = first_frame.with_origin(Point3::new(1.0, 0.0, 0.0));
    let top_first = first_frame.with_origin(Point3::new(0.0, 0.0, 1.0));
    let top_second = second_frame.with_origin(Point3::new(1.0, 0.0, 1.0));
    let circles = [
        Circle::new(first_frame, 1.0).unwrap(),
        Circle::new(top_first, 1.0).unwrap(),
        Circle::new(second_frame, 1.0).unwrap(),
        Circle::new(top_second, 1.0).unwrap(),
    ];
    let points = [
        circles[0].eval(-angle),
        circles[0].eval(angle),
        circles[1].eval(-angle),
        circles[1].eval(angle),
    ];
    LensGeometry {
        angle,
        cylinders: [
            Cylinder::new(first_frame, 1.0).unwrap(),
            Cylinder::new(second_frame, 1.0).unwrap(),
        ],
        circles,
        points,
    }
}

fn cylinder_arc(edge: AnalyticEdgeKey, sense: Sense, height: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            map(1.0, 0.0),
        ),
    )
}

fn cylinder_ruling(edge: AnalyticEdgeKey, sense: Sense, longitude: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
            ),
            map(1.0, 0.0),
        ),
    )
}

fn cap_arc(
    edge: AnalyticEdgeKey,
    sense: Sense,
    center: Point2,
    parameter_scale: f64,
) -> AnalyticShellFin {
    AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Circle(Circle2d::new(center, 1.0, Vec2::new(1.0, 0.0)).unwrap()),
            map(parameter_scale, 0.0),
        ),
    )
}

fn cap_chord(
    edge: AnalyticEdgeKey,
    sense: Sense,
    origin: Point2,
    direction: Vec2,
) -> AnalyticShellFin {
    AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(Line2d::new(origin, direction).unwrap()),
            map(1.0, 0.0),
        ),
    )
}

fn lens_edges(geometry: LensGeometry) -> Vec<AnalyticShellEdge> {
    let a = geometry.angle;
    let [lower, upper, _, _] = geometry.points;
    vec![
        AnalyticShellEdge::new(
            E0,
            [V0, V1],
            AnalyticShellCurve::Circle(geometry.circles[0]),
            ParamRange::new(-a, a),
        ),
        AnalyticShellEdge::new(
            E1,
            [V2, V3],
            AnalyticShellCurve::Circle(geometry.circles[1]),
            ParamRange::new(-a, a),
        ),
        AnalyticShellEdge::new(
            E2,
            [V1, V0],
            AnalyticShellCurve::Circle(geometry.circles[2]),
            ParamRange::new(2.0 * a, 4.0 * a),
        ),
        AnalyticShellEdge::new(
            E3,
            [V3, V2],
            AnalyticShellCurve::Circle(geometry.circles[3]),
            ParamRange::new(2.0 * a, 4.0 * a),
        ),
        AnalyticShellEdge::new(
            E4,
            [V0, V2],
            AnalyticShellCurve::Line(Line::new(lower, Vec3::new(0.0, 0.0, 1.0)).unwrap()),
            ParamRange::new(0.0, 1.0),
        ),
        AnalyticShellEdge::new(
            E5,
            [V1, V3],
            AnalyticShellCurve::Line(Line::new(upper, Vec3::new(0.0, 0.0, 1.0)).unwrap()),
            ParamRange::new(0.0, 1.0),
        ),
        AnalyticShellEdge::new(
            E6,
            [V0, V1],
            AnalyticShellCurve::Line(Line::new(lower, upper - lower).unwrap()),
            ParamRange::new(0.0, (upper - lower).norm()),
        ),
        AnalyticShellEdge::new(
            E7,
            [V2, V3],
            AnalyticShellCurve::Line(
                Line::new(geometry.points[2], geometry.points[3] - geometry.points[2]).unwrap(),
            ),
            ParamRange::new(0.0, (geometry.points[3] - geometry.points[2]).norm()),
        ),
    ]
}

fn lens_faces(geometry: LensGeometry) -> Vec<AnalyticShellFace> {
    let a = geometry.angle;
    let bottom_frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let top_frame = Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0));
    vec![
        AnalyticShellFace::new(
            AnalyticFaceKey::new(0),
            AnalyticShellSurface::Cylinder(geometry.cylinders[0]),
            Sense::Forward,
            FaceDomain::from_bounds(-a, a, 0.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cylinder_arc(E0, Sense::Forward, 0.0),
                cylinder_ruling(E5, Sense::Forward, a),
                cylinder_arc(E1, Sense::Reversed, 1.0),
                cylinder_ruling(E4, Sense::Reversed, -a),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(1),
            AnalyticShellSurface::Cylinder(geometry.cylinders[1]),
            Sense::Forward,
            FaceDomain::from_bounds(2.0 * a, 4.0 * a, 0.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cylinder_arc(E2, Sense::Forward, 0.0),
                cylinder_ruling(E4, Sense::Forward, 4.0 * a),
                cylinder_arc(E3, Sense::Reversed, 1.0),
                cylinder_ruling(E5, Sense::Reversed, 2.0 * a),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(2),
            AnalyticShellSurface::Plane(Plane::new(bottom_frame)),
            Sense::Forward,
            FaceDomain::from_bounds(0.49, 1.1, -1.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cap_arc(E0, Sense::Reversed, Point2::new(0.0, 0.0), -1.0),
                cap_chord(
                    E6,
                    Sense::Forward,
                    Point2::new(0.5, 3.0_f64.sqrt() / 2.0),
                    Vec2::new(0.0, -1.0),
                ),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(3),
            AnalyticShellSurface::Plane(Plane::new(bottom_frame)),
            Sense::Forward,
            FaceDomain::from_bounds(-0.1, 0.51, -1.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cap_arc(E2, Sense::Reversed, Point2::new(1.0, 0.0), -1.0),
                cap_chord(
                    E6,
                    Sense::Reversed,
                    Point2::new(0.5, 3.0_f64.sqrt() / 2.0),
                    Vec2::new(0.0, -1.0),
                ),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(4),
            AnalyticShellSurface::Plane(Plane::new(top_frame)),
            Sense::Forward,
            FaceDomain::from_bounds(0.49, 1.1, -1.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cap_arc(E1, Sense::Forward, Point2::new(0.0, 0.0), 1.0),
                cap_chord(
                    E7,
                    Sense::Reversed,
                    Point2::new(0.5, -3.0_f64.sqrt() / 2.0),
                    Vec2::new(0.0, 1.0),
                ),
            ])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(5),
            AnalyticShellSurface::Plane(Plane::new(top_frame)),
            Sense::Forward,
            FaceDomain::from_bounds(-0.1, 0.51, -1.0, 1.0).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                cap_chord(
                    E7,
                    Sense::Forward,
                    Point2::new(0.5, -3.0_f64.sqrt() / 2.0),
                    Vec2::new(0.0, 1.0),
                ),
                cap_arc(E3, Sense::Forward, Point2::new(1.0, 0.0), 1.0),
            ])],
        ),
    ]
}

fn parallel_cylinder_lens_input() -> AnalyticShellInput {
    let geometry = lens_geometry();
    AnalyticShellInput::new(
        vec![
            AnalyticShellVertex::new(V0, geometry.points[0]),
            AnalyticShellVertex::new(V1, geometry.points[1]),
            AnalyticShellVertex::new(V2, geometry.points[2]),
            AnalyticShellVertex::new(V3, geometry.points[3]),
        ],
        lens_edges(geometry),
        lens_faces(geometry),
    )
}

#[test]
fn parallel_cylinder_rulings_are_source_ordered_and_permutation_invariant() {
    let input = parallel_cylinder_lens_input();
    let store = Store::new();
    let prepared = prepare_analytic_shell(&input, &store, TOLERANCE).unwrap();
    let geometry = lens_geometry();
    let rulings = prepared
        .edges()
        .iter()
        .filter_map(|edge| match edge.proof() {
            AnalyticEdgeProof::CylinderCylinderRuling(certificate) => Some(certificate),
            _ => None,
        })
        .collect::<Vec<_>>();
    assert_eq!(rulings.len(), 2);
    for certificate in rulings {
        assert_eq!(certificate.traces()[0].surface(), geometry.cylinders[0]);
        assert_eq!(certificate.traces()[1].surface(), geometry.cylinders[1]);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= TOLERANCE)
        );
    }

    let mut permuted = input.clone();
    permuted.vertices.reverse();
    permuted.edges.rotate_left(3);
    permuted.faces.reverse();
    for face in &mut permuted.faces {
        for loop_ in &mut face.loops {
            loop_.fins.rotate_left(1);
        }
    }
    let reparsed = prepare_analytic_shell(&permuted, &store, TOLERANCE).unwrap();
    assert_eq!(prepared, reparsed);
}

#[test]
fn parallel_cylinder_lens_materializes_and_is_full_check_compatible() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&parallel_cylinder_lens_input(), TOLERANCE)
        .unwrap();
    for key in [E4, E5] {
        let edge_id = output
            .edges()
            .iter()
            .find_map(|&(candidate, handle)| (candidate == key).then_some(handle))
            .unwrap();
        let edge = transaction.store().get(edge_id).unwrap();
        assert!(matches!(
            transaction.store().curve(edge.curve().unwrap()).unwrap(),
            CurveGeom::Line(_)
        ));
        assert_eq!(edge.fins().len(), 2);
        assert!(edge.fins().iter().all(|fin| {
            let fin = transaction.store().get(*fin).unwrap();
            let loop_ = transaction.store().get(fin.parent()).unwrap();
            let face = transaction.store().get(loop_.face()).unwrap();
            matches!(
                transaction.store().surface(face.surface()).unwrap(),
                SurfaceGeom::Cylinder(_)
            )
        }));
    }

    let decision = transaction
        .commit_full(&[output.body()], FullCommitRequirement::AllowIndeterminate)
        .unwrap();
    let report = decision.checks()[0].report();
    assert_eq!(report.outcome(), CheckOutcome::Indeterminate, "{report:#?}");
    assert!(report.faults.is_empty(), "{report:#?}");
    assert_eq!(report.gaps.len(), 2, "{report:#?}");
    assert!(decision.is_committed());
}

#[test]
fn sense_tampering_fails_closed_during_preflight() {
    let mut input = parallel_cylinder_lens_input();
    let fin = input.faces[1].loops[0]
        .fins
        .iter_mut()
        .find(|fin| fin.edge == E4)
        .unwrap();
    fin.sense = fin.sense.flipped();
    assert!(matches!(
        prepare_analytic_shell(&input, &Store::new(), TOLERANCE),
        Err(AnalyticShellPlanError::OpenLoop { .. })
            | Err(AnalyticShellPlanError::EdgeUsesNotOpposed(E4))
    ));
}

#[test]
fn materialization_rejects_swapped_proof_and_wrong_carrier_trace_or_source() {
    let store = Store::new();
    let prepared =
        prepare_analytic_shell(&parallel_cylinder_lens_input(), &store, TOLERANCE).unwrap();
    let lower = prepared
        .edges
        .binary_search_by_key(&E4, |edge| edge.edge.key)
        .unwrap();
    let upper = prepared
        .edges
        .binary_search_by_key(&E5, |edge| edge.edge.key)
        .unwrap();

    let mut swapped = prepared.clone();
    let first = swapped.edges[lower].proof;
    swapped.edges[lower].proof = swapped.edges[upper].proof;
    swapped.edges[upper].proof = first;
    assert_materialization_refuses(&swapped);

    let mut wrong_carrier = prepared.clone();
    wrong_carrier.edges[lower].edge.carrier = prepared.edges[upper].edge.carrier;
    assert_materialization_refuses(&wrong_carrier);

    let mut wrong_trace = prepared.clone();
    let trace = wrong_trace.faces[0].loops[0]
        .fins
        .iter_mut()
        .find(|fin| fin.edge == E4)
        .unwrap();
    trace.pcurve.curve = AnalyticShellPcurve::Line(
        Line2d::new(
            Point2::new(-lens_geometry().angle + 0.125, 0.0),
            Vec2::new(0.0, 1.0),
        )
        .unwrap(),
    );
    assert_materialization_refuses(&wrong_trace);

    let mut wrong_source = prepared;
    wrong_source.faces[1].surface = wrong_source.faces[0].surface;
    assert_materialization_refuses(&wrong_source);
}

fn assert_materialization_refuses(prepared: &PreparedAnalyticShell) {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let error = transaction
        .allocate_prepared_analytic_shell_for_test(prepared)
        .unwrap_err();
    assert!(matches!(
        error,
        AnalyticShellAssemblyError::Store(Error::InvalidGeometry { .. })
    ));
    transaction.rollback().unwrap();
}
