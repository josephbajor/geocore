use super::*;
use crate::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellOutput, AnalyticShellPcurve, AnalyticShellSurface,
    AnalyticShellVertex, AnalyticVertexKey,
};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::{Body, Edge, Face, FaceDomain, Fin, Loop, Region, Shell, Vertex};
use crate::transaction::FullCommitRequirement;
use kcore::operation::{
    ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Vec2};
use kgraph::AffineParamMap1d;

const TOLERANCE: f64 = 1.0e-12;
const TANGENT: AnalyticVertexKey = AnalyticVertexKey::new(0);
const OUTER_FAR: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const OUTER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const INNER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
const INNER_FAR: AnalyticEdgeKey = AnalyticEdgeKey::new(3);

fn map(scale: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, 0.0).unwrap()
}

fn side_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    axial_parameter: f64,
    closed: bool,
) -> AnalyticShellFin {
    let use_ = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(
            Line2d::new(Point2::new(0.0, axial_parameter), Vec2::new(1.0, 0.0)).unwrap(),
        ),
        map(1.0),
    );
    AnalyticShellFin::new(
        edge,
        sense,
        if closed {
            use_.with_closure_winding([1, 0])
        } else {
            use_
        },
    )
}

fn plane_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    plane: Plane,
    circle: Circle,
    closed: bool,
) -> AnalyticShellFin {
    let center = plane.frame().to_local(circle.frame().origin());
    let x = Vec2::new(
        circle.frame().x().dot(plane.frame().x()),
        circle.frame().x().dot(plane.frame().y()),
    );
    let y = Vec2::new(
        circle.frame().y().dot(plane.frame().x()),
        circle.frame().y().dot(plane.frame().y()),
    );
    let use_ = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Circle(
            Circle2d::new(Point2::new(center.x, center.y), circle.radius(), x).unwrap(),
        ),
        map(if x.perp().dot(y) > 0.0 { 1.0 } else { -1.0 }),
    );
    AnalyticShellFin::new(
        edge,
        sense,
        if closed {
            use_.with_closure_winding([0, 0])
        } else {
            use_
        },
    )
}

fn oblique_frame() -> Frame {
    Frame::new(
        Point3::new(0.5, 0.0, 0.0),
        Vec3::new(0.0, 0.28, 0.96),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn axis_frame(frame: Frame, origin: Point3, reversed: bool) -> Frame {
    if reversed {
        Frame::new(origin, -frame.z(), frame.x()).unwrap()
    } else {
        frame.with_origin(origin)
    }
}

fn compact_internal_tangent_input(
    frame: Frame,
    outer_reversed: bool,
    inner_reversed: bool,
    permuted: bool,
) -> AnalyticShellInput {
    let outer_radius = 2.0;
    let inner_radius = 1.0;
    let outer_contact_origin = frame.origin();
    let outer_far_origin = frame.point_at(0.0, 0.0, -2.0);
    let inner_contact_origin = frame.point_at(1.0, 0.0, 0.0);
    let inner_far_origin = frame.point_at(1.0, 0.0, 1.0);
    let outer_frame = axis_frame(frame, outer_contact_origin, outer_reversed);
    let inner_frame = axis_frame(frame, inner_contact_origin, inner_reversed);
    let outer = Cylinder::new(outer_frame, outer_radius).unwrap();
    let inner = Cylinder::new(inner_frame, inner_radius).unwrap();
    let outer_far_circle =
        Circle::new(outer_frame.with_origin(outer_far_origin), outer_radius).unwrap();
    let outer_contact_circle = Circle::new(outer_frame, outer_radius).unwrap();
    let inner_contact_circle = Circle::new(inner_frame, inner_radius).unwrap();
    let inner_far_circle =
        Circle::new(inner_frame.with_origin(inner_far_origin), inner_radius).unwrap();
    let outer_far_plane = Plane::new(Frame::new(outer_far_origin, -frame.z(), frame.x()).unwrap());
    let inner_far_plane = Plane::new(frame.with_origin(inner_far_origin));
    let shoulder_plane = Plane::new(frame.with_origin(outer_contact_origin));
    let tau = core::f64::consts::TAU;
    let outer_far_v: f64 = if outer_reversed { 2.0 } else { -2.0 };
    let inner_far_v: f64 = if inner_reversed { -1.0 } else { 1.0 };
    let outer_far_sense = if outer_reversed {
        Sense::Reversed
    } else {
        Sense::Forward
    };
    let outer_contact_sense = outer_far_sense.flipped();
    let inner_contact_sense = if inner_reversed {
        Sense::Reversed
    } else {
        Sense::Forward
    };
    let inner_far_sense = inner_contact_sense.flipped();

    let mut faces = vec![
        AnalyticShellFace::new(
            AnalyticFaceKey::new(0),
            AnalyticShellSurface::Cylinder(outer),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, tau, outer_far_v.min(0.0), outer_far_v.max(0.0)).unwrap(),
            vec![
                AnalyticShellLoop::new(vec![side_fin(
                    OUTER_FAR,
                    outer_far_sense,
                    outer_far_v,
                    true,
                )]),
                AnalyticShellLoop::new(vec![side_fin(
                    OUTER_CONTACT,
                    outer_contact_sense,
                    0.0,
                    false,
                )]),
            ],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(1),
            AnalyticShellSurface::Cylinder(inner),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, tau, inner_far_v.min(0.0), inner_far_v.max(0.0)).unwrap(),
            vec![
                AnalyticShellLoop::new(vec![side_fin(
                    INNER_CONTACT,
                    inner_contact_sense,
                    0.0,
                    false,
                )]),
                AnalyticShellLoop::new(vec![side_fin(
                    INNER_FAR,
                    inner_far_sense,
                    inner_far_v,
                    true,
                )]),
            ],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(2),
            AnalyticShellSurface::Plane(outer_far_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![plane_fin(
                OUTER_FAR,
                outer_far_sense.flipped(),
                outer_far_plane,
                outer_far_circle,
                true,
            )])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(3),
            AnalyticShellSurface::Plane(inner_far_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-1.1, 1.1, -1.1, 1.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![plane_fin(
                INNER_FAR,
                inner_far_sense.flipped(),
                inner_far_plane,
                inner_far_circle,
                true,
            )])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(4),
            AnalyticShellSurface::Plane(shoulder_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                plane_fin(
                    OUTER_CONTACT,
                    outer_contact_sense.flipped(),
                    shoulder_plane,
                    outer_contact_circle,
                    false,
                ),
                plane_fin(
                    INNER_CONTACT,
                    inner_contact_sense.flipped(),
                    shoulder_plane,
                    inner_contact_circle,
                    false,
                ),
            ])],
        ),
    ];
    let tangent = outer_contact_circle.eval(0.0);
    let vertices = vec![AnalyticShellVertex::new(TANGENT, tangent)];
    let edges = vec![
        AnalyticShellEdge::new(
            OUTER_CONTACT,
            [TANGENT, TANGENT],
            AnalyticShellCurve::Circle(outer_contact_circle),
            outer_contact_circle.param_range(),
        ),
        AnalyticShellEdge::new(
            INNER_CONTACT,
            [TANGENT, TANGENT],
            AnalyticShellCurve::Circle(inner_contact_circle),
            inner_contact_circle.param_range(),
        ),
    ];
    let mut closed_edges = vec![
        AnalyticShellClosedEdge::new(
            OUTER_FAR,
            AnalyticShellCurve::Circle(outer_far_circle),
            outer_far_circle.param_range(),
        ),
        AnalyticShellClosedEdge::new(
            INNER_FAR,
            AnalyticShellCurve::Circle(inner_far_circle),
            inner_far_circle.param_range(),
        ),
    ];
    if permuted {
        faces.reverse();
        closed_edges.reverse();
    }
    AnalyticShellInput::new(vertices, edges, faces).with_closed_edges(closed_edges)
}

fn face_for_key(output: &AnalyticShellOutput, key: AnalyticFaceKey) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (*candidate == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &AnalyticShellOutput, key: AnalyticEdgeKey) -> EdgeId {
    output
        .edges()
        .iter()
        .find_map(|(candidate, edge)| (*candidate == key).then_some(*edge))
        .unwrap()
}

fn positive() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Positive,
    })
}

fn invalid() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Invalid,
    })
}

fn assert_not_full_certified(store: &Store, output: &AnalyticShellOutput) {
    assert_ne!(
        crate::shell_proof::certify_shell(
            store,
            output.shell(),
            crate::entity::BodyKind::Solid,
            crate::entity::RegionKind::Solid,
        )
        .unwrap(),
        positive().unwrap(),
        "a fallback theorem must not certify the tampered shell",
    );
    let report = check_body_report(store, output.body(), CheckLevel::Full).unwrap();
    assert_ne!(
        report.outcome(),
        CheckOutcome::Valid,
        "Full checking must not certify the tampered shell: {report:#?}",
    );
}

#[test]
fn compact_internal_tangent_topology_is_full_valid_across_frames_and_axis_directions() {
    for frame in [Frame::world(), oblique_frame()] {
        for outer_reversed in [false, true] {
            for inner_reversed in [false, true] {
                for permuted in [false, true] {
                    let mut store = Store::new();
                    let mut transaction = store.transaction().unwrap();
                    let output = transaction
                        .assemble_analytic_shell(
                            &compact_internal_tangent_input(
                                frame,
                                outer_reversed,
                                inner_reversed,
                                permuted,
                            ),
                            TOLERANCE,
                        )
                        .unwrap();
                    assert_eq!(output.faces().len(), 5);
                    assert_eq!(output.edges().len(), 4);
                    assert_eq!(output.vertices().len(), 1);
                    let fast =
                        check_body_report(transaction.store(), output.body(), CheckLevel::Fast)
                            .unwrap();
                    assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:#?}");
                    let full =
                        check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                            .unwrap();
                    assert_eq!(
                        full.outcome(),
                        CheckOutcome::Valid,
                        "frame={frame:?}, outer_reversed={outer_reversed}, inner_reversed={inner_reversed}, permuted={permuted}: {full:#?}",
                    );
                    transaction
                        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                        .unwrap();
                }
            }
        }
    }
}

#[test]
fn adjacent_circle_center_tamper_fails_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &compact_internal_tangent_input(Frame::world(), false, false, false),
            TOLERANCE,
        )
        .unwrap();
    let inner_contact = edge_for_key(&output, INNER_CONTACT);

    let mut tampered = transaction.store().clone();
    let mut edit = tampered.transaction().unwrap();
    let curve_id = edit.store().get(inner_contact).unwrap().curve.unwrap();
    let CurveGeom::Circle(circle) = *edit.store().get(curve_id).unwrap() else {
        panic!("contact edge must retain a circle")
    };
    let origin = circle.frame().origin();
    let shifted = Circle::new(
        circle
            .frame()
            .with_origin(Point3::new(origin.x.next_up(), origin.y, origin.z)),
        circle.radius(),
    )
    .unwrap();
    edit.store_mut()
        .replace_curve(curve_id, CurveGeom::Circle(shifted))
        .unwrap();

    assert_ne!(
        certify_parallel_cylinder_contact_shell(edit.store(), output.shell(), None).unwrap(),
        positive(),
    );
    assert_not_full_certified(edit.store(), &output);
}

#[test]
fn coincident_but_distinct_tangent_vertices_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &compact_internal_tangent_input(Frame::world(), false, false, false),
            TOLERANCE,
        )
        .unwrap();
    let tangent = output.vertices()[0].1;
    let position = transaction.store().vertex_position(tangent).unwrap();

    let mut tampered = transaction.store().clone();
    let mut edit = tampered.transaction().unwrap();
    let point = edit.store_mut().insert_point(position).unwrap();
    let duplicate = edit.store_mut().add(Vertex {
        point,
        tolerance: None,
    });
    edit.store_mut()
        .get_mut(edge_for_key(&output, INNER_CONTACT))
        .unwrap()
        .vertices = [Some(duplicate), Some(duplicate)];

    assert_ne!(
        certify_parallel_cylinder_contact_shell(edit.store(), output.shell(), None).unwrap(),
        positive(),
    );
    assert_not_full_certified(edit.store(), &output);
}

#[test]
fn recognized_face_sense_tampers_preserve_embedding_and_invalidate_orientation() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &compact_internal_tangent_input(Frame::world(), false, false, false),
            TOLERANCE,
        )
        .unwrap();

    for key in [
        AnalyticFaceKey::new(0),
        AnalyticFaceKey::new(2),
        AnalyticFaceKey::new(4),
    ] {
        let mut tampered = transaction.store().clone();
        let face = face_for_key(&output, key);
        tampered.get_mut(face).unwrap().sense = tampered.get(face).unwrap().sense.flipped();
        assert_eq!(
            certify_parallel_cylinder_contact_shell(&tampered, output.shell(), None).unwrap(),
            invalid(),
            "face key {}",
            key.value(),
        );
        assert_eq!(
            crate::shell_proof::certify_shell(
                &tampered,
                output.shell(),
                crate::entity::BodyKind::Solid,
                crate::entity::RegionKind::Solid,
            )
            .unwrap(),
            invalid().unwrap(),
            "face key {} must not bypass through a fallback theorem",
            key.value(),
        );
        let report = check_body_report(&tampered, output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Invalid, "{report:#?}");
    }
}

fn session_with_work(allowed: u64) -> SessionPolicy {
    let budget = BudgetPlan::new([LimitSpec::new(
        PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        budget,
        PolicyVersion::V1,
    )
}

fn store_counts(store: &Store) -> [usize; 8] {
    [
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Loop>(),
        store.count::<Fin>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
    ]
}

fn consumed(scope: &OperationScope<'_, '_>) -> u64 {
    scope
        .ledger()
        .snapshots()
        .into_iter()
        .find(|snapshot| snapshot.stage == PARALLEL_CYLINDER_CONTACT_SHELL_WORK)
        .unwrap()
        .consumed
}

#[test]
fn internal_tangent_work_accepts_1113_rejects_1112_and_inapplicable_is_free() {
    // Independent structural oracle: N = 1 shell + 5 faces + 7 loops + 8 uses.
    // The theorem contract is N^2 + 32N = 21^2 + 32*21 = 1,113.
    const SIZE: u64 = 1 + 5 + 7 + 8;
    const REQUIRED: u64 = SIZE * SIZE + 32 * SIZE;
    assert_eq!(SIZE, 21);
    assert_eq!(REQUIRED, 1_113);

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &compact_internal_tangent_input(Frame::world(), false, false, true),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        proof_work(transaction.store(), output.shell()).unwrap(),
        Some(REQUIRED),
    );
    let before = store_counts(transaction.store());

    let accepted_policy = session_with_work(REQUIRED);
    let accepted_context = OperationContext::new(&accepted_policy, Tolerances::default()).unwrap();
    let mut accepted_scope = OperationScope::new(&accepted_context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(
            transaction.store(),
            output.shell(),
            Some(&mut accepted_scope),
        )
        .unwrap(),
        positive(),
    );
    assert_eq!(consumed(&accepted_scope), REQUIRED);
    assert_eq!(store_counts(transaction.store()), before);

    let denied_policy = session_with_work(REQUIRED - 1);
    let denied_context = OperationContext::new(&denied_policy, Tolerances::default()).unwrap();
    let mut denied_scope = OperationScope::new(&denied_context);
    let error = certify_parallel_cylinder_contact_shell(
        transaction.store(),
        output.shell(),
        Some(&mut denied_scope),
    )
    .unwrap_err();
    assert_eq!(
        error.limit(),
        Some(kcore::operation::LimitSnapshot {
            stage: PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
            resource: ResourceKind::Work,
            consumed: REQUIRED,
            allowed: REQUIRED - 1,
        }),
    );
    assert_eq!(consumed(&denied_scope), 0);
    assert_eq!(store_counts(transaction.store()), before);

    let mut inapplicable = transaction.store().clone();
    inapplicable
        .get_mut(output.shell())
        .unwrap()
        .edges
        .push(edge_for_key(&output, OUTER_FAR));
    let zero_policy = session_with_work(0);
    let zero_context = OperationContext::new(&zero_policy, Tolerances::default()).unwrap();
    let mut zero_scope = OperationScope::new(&zero_context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(
            &inapplicable,
            output.shell(),
            Some(&mut zero_scope),
        )
        .unwrap(),
        None,
    );
    assert_eq!(consumed(&zero_scope), 0);
    assert!(zero_scope.ledger().limit_events().is_empty());
}
