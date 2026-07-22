use super::*;
use crate::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
    AnalyticShellOutput, AnalyticShellPcurve, AnalyticShellSurface,
};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::{Body, Edge, Face, FaceDomain, Fin, Loop, Region, Shell, Vertex};
use crate::transaction::FullCommitRequirement;
use kcore::operation::{
    ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::{ANGULAR_RESOLUTION, Tolerances};
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::surface::Plane;
use kgeom::vec::Vec2;
use kgraph::AffineParamMap1d;

const TOLERANCE: f64 = 1.0e-12;
const OUTER_RADIUS: f64 = 2.0;
const INNER_RADIUS: f64 = 1.0;
const DEFAULT_INNER_OFFSET: f64 = 0.25;

const OUTER_FAR_EDGE: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const OUTER_CONTACT_EDGE: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const INNER_CONTACT_EDGE: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
const INNER_FAR_EDGE: AnalyticEdgeKey = AnalyticEdgeKey::new(3);

fn affine_map(scale: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, 0.0).unwrap()
}

fn oblique_frame() -> Frame {
    Frame::new(
        Point3::new(2.5, -1.75, 0.625),
        Vec3::new(0.48, 0.64, 0.6),
        Vec3::new(0.8, -0.6, 0.0),
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

fn side_fin(edge: AnalyticEdgeKey, sense: Sense, axial_parameter: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        edge,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(
                    kgeom::vec::Point2::new(0.0, axial_parameter),
                    Vec2::new(1.0, 0.0),
                )
                .unwrap(),
            ),
            affine_map(1.0),
        )
        .with_closure_winding([1, 0]),
    )
}

fn cap_fin(
    edge: AnalyticEdgeKey,
    side_sense: Sense,
    plane: Plane,
    circle: Circle,
) -> AnalyticShellFin {
    let center = plane.frame().to_local(circle.frame().origin());
    let local_x = Vec2::new(
        circle.frame().x().dot(plane.frame().x()),
        circle.frame().x().dot(plane.frame().y()),
    );
    let local_y = Vec2::new(
        circle.frame().y().dot(plane.frame().x()),
        circle.frame().y().dot(plane.frame().y()),
    );
    let scale = if local_x.perp().dot(local_y) > 0.0 {
        1.0
    } else {
        -1.0
    };
    AnalyticShellFin::new(
        edge,
        side_sense.flipped(),
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Circle(
                Circle2d::new(
                    kgeom::vec::Point2::new(center.x, center.y),
                    circle.radius(),
                    local_x,
                )
                .unwrap(),
            ),
            affine_map(scale),
        )
        .with_closure_winding([0, 0]),
    )
}

fn nested_contact_input(
    frame: Frame,
    outer_axis_reversed: bool,
    inner_axis_reversed: bool,
    permuted: bool,
    inner_offset: f64,
) -> AnalyticShellInput {
    let outer_contact_origin = frame.origin();
    let outer_far_origin = frame.point_at(0.0, 0.0, -2.0);
    let inner_contact_origin = frame.point_at(inner_offset, 0.0, 0.0);
    let inner_far_origin = frame.point_at(inner_offset, 0.0, 1.0);
    let outer_frame = axis_frame(frame, outer_contact_origin, outer_axis_reversed);
    let inner_frame = axis_frame(frame, inner_contact_origin, inner_axis_reversed);
    let outer = Cylinder::new(outer_frame, OUTER_RADIUS).unwrap();
    let inner = Cylinder::new(inner_frame, INNER_RADIUS).unwrap();

    let outer_far = Circle::new(outer_frame.with_origin(outer_far_origin), OUTER_RADIUS).unwrap();
    let outer_contact = Circle::new(outer_frame, OUTER_RADIUS).unwrap();
    let inner_contact = Circle::new(inner_frame, INNER_RADIUS).unwrap();
    let inner_far = Circle::new(inner_frame.with_origin(inner_far_origin), INNER_RADIUS).unwrap();

    let outer_far_plane = Plane::new(Frame::new(outer_far_origin, -frame.z(), frame.x()).unwrap());
    let inner_far_plane = Plane::new(frame.with_origin(inner_far_origin));
    let annulus_plane = Plane::new(frame.with_origin(outer_contact_origin));

    let outer_far_v = if outer_axis_reversed { 2.0 } else { -2.0 };
    let outer_contact_v = 0.0;
    let inner_contact_v = 0.0;
    let inner_far_v = if inner_axis_reversed { -1.0 } else { 1.0 };
    let outer_far_sense = if outer_axis_reversed {
        Sense::Reversed
    } else {
        Sense::Forward
    };
    let outer_contact_sense = outer_far_sense.flipped();
    let inner_contact_sense = if inner_axis_reversed {
        Sense::Reversed
    } else {
        Sense::Forward
    };
    let inner_far_sense = inner_contact_sense.flipped();

    let outer_far_loop =
        AnalyticShellLoop::new(vec![side_fin(OUTER_FAR_EDGE, outer_far_sense, outer_far_v)]);
    let outer_contact_loop = AnalyticShellLoop::new(vec![side_fin(
        OUTER_CONTACT_EDGE,
        outer_contact_sense,
        outer_contact_v,
    )]);
    let outer_loops = if outer_axis_reversed {
        vec![outer_contact_loop, outer_far_loop]
    } else {
        vec![outer_far_loop, outer_contact_loop]
    };
    let inner_contact_loop = AnalyticShellLoop::new(vec![side_fin(
        INNER_CONTACT_EDGE,
        inner_contact_sense,
        inner_contact_v,
    )]);
    let inner_far_loop =
        AnalyticShellLoop::new(vec![side_fin(INNER_FAR_EDGE, inner_far_sense, inner_far_v)]);
    let inner_loops = if inner_axis_reversed {
        vec![inner_far_loop, inner_contact_loop]
    } else {
        vec![inner_contact_loop, inner_far_loop]
    };

    let mut faces = vec![
        AnalyticShellFace::new(
            AnalyticFaceKey::new(0),
            AnalyticShellSurface::Cylinder(outer),
            Sense::Forward,
            FaceDomain::from_bounds(
                0.0,
                core::f64::consts::TAU,
                outer_far_v.min(outer_contact_v),
                outer_far_v.max(outer_contact_v),
            )
            .unwrap(),
            outer_loops,
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(1),
            AnalyticShellSurface::Cylinder(inner),
            Sense::Forward,
            FaceDomain::from_bounds(
                0.0,
                core::f64::consts::TAU,
                inner_far_v.min(inner_contact_v),
                inner_far_v.max(inner_contact_v),
            )
            .unwrap(),
            inner_loops,
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(2),
            AnalyticShellSurface::Plane(outer_far_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![cap_fin(
                OUTER_FAR_EDGE,
                outer_far_sense,
                outer_far_plane,
                outer_far,
            )])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(3),
            AnalyticShellSurface::Plane(inner_far_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-1.1, 1.1, -1.1, 1.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![cap_fin(
                INNER_FAR_EDGE,
                inner_far_sense,
                inner_far_plane,
                inner_far,
            )])],
        ),
        AnalyticShellFace::new(
            AnalyticFaceKey::new(4),
            AnalyticShellSurface::Plane(annulus_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![
                AnalyticShellLoop::new(vec![cap_fin(
                    OUTER_CONTACT_EDGE,
                    outer_contact_sense,
                    annulus_plane,
                    outer_contact,
                )]),
                AnalyticShellLoop::new(vec![cap_fin(
                    INNER_CONTACT_EDGE,
                    inner_contact_sense,
                    annulus_plane,
                    inner_contact,
                )]),
            ],
        ),
    ];
    let mut closed_edges = vec![
        AnalyticShellClosedEdge::new(
            OUTER_FAR_EDGE,
            crate::analytic_shell::AnalyticShellCurve::Circle(outer_far),
            outer_far.param_range(),
        ),
        AnalyticShellClosedEdge::new(
            OUTER_CONTACT_EDGE,
            crate::analytic_shell::AnalyticShellCurve::Circle(outer_contact),
            outer_contact.param_range(),
        ),
        AnalyticShellClosedEdge::new(
            INNER_CONTACT_EDGE,
            crate::analytic_shell::AnalyticShellCurve::Circle(inner_contact),
            inner_contact.param_range(),
        ),
        AnalyticShellClosedEdge::new(
            INNER_FAR_EDGE,
            crate::analytic_shell::AnalyticShellCurve::Circle(inner_far),
            inner_far.param_range(),
        ),
    ];
    if permuted {
        faces.reverse();
        closed_edges.rotate_left(1);
    }
    AnalyticShellInput::new(Vec::new(), Vec::new(), faces).with_closed_edges(closed_edges)
}

fn face_for_key(output: &AnalyticShellOutput, key: u64) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &AnalyticShellOutput, key: u64) -> EdgeId {
    output
        .edges()
        .iter()
        .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
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
fn nested_contact_is_full_valid_across_frames_axis_directions_and_permutations() {
    for frame in [Frame::world(), oblique_frame()] {
        for outer_axis_reversed in [false, true] {
            for inner_axis_reversed in [false, true] {
                for permuted in [false, true] {
                    let mut store = Store::new();
                    let mut transaction = store.transaction().unwrap();
                    let output = transaction
                        .assemble_analytic_shell(
                            &nested_contact_input(
                                frame,
                                outer_axis_reversed,
                                inner_axis_reversed,
                                permuted,
                                DEFAULT_INNER_OFFSET,
                            ),
                            TOLERANCE,
                        )
                        .unwrap();
                    assert_eq!(output.faces().len(), 5);
                    assert_eq!(output.edges().len(), 4);
                    assert!(output.vertices().is_empty());
                    assert_eq!(
                        certify_parallel_cylinder_contact_shell(
                            transaction.store(),
                            output.shell(),
                            None,
                        )
                        .unwrap(),
                        positive(),
                        "frame={frame:?}, outer_reversed={outer_axis_reversed}, inner_reversed={inner_axis_reversed}, permuted={permuted}",
                    );
                    assert_eq!(
                        crate::shell_proof::certify_shell(
                            transaction.store(),
                            output.shell(),
                            crate::entity::BodyKind::Solid,
                            crate::entity::RegionKind::Solid,
                        )
                        .unwrap(),
                        positive().unwrap(),
                    );
                    let report =
                        check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                            .unwrap();
                    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
                    transaction
                        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                        .unwrap();
                }
            }
        }
    }
}

#[test]
fn recognized_face_sense_tampers_preserve_embedding_and_invalidate_orientation() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &nested_contact_input(Frame::world(), false, false, false, DEFAULT_INNER_OFFSET),
            TOLERANCE,
        )
        .unwrap();

    for key in [0, 2, 4] {
        let mut tampered = transaction.store().clone();
        let face = face_for_key(&output, key);
        tampered.get_mut(face).unwrap().sense = tampered.get(face).unwrap().sense.flipped();
        assert_eq!(
            certify_parallel_cylinder_contact_shell(&tampered, output.shell(), None).unwrap(),
            invalid(),
            "face key {key}",
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
            "face key {key} must not bypass through a fallback theorem",
        );
        let report = check_body_report(&tampered, output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Invalid, "{report:#?}");
    }
}

fn replace_circle_origin(
    transaction: &mut crate::transaction::Transaction<'_>,
    edge: EdgeId,
    delta: Vec3,
) {
    let curve_id = transaction.store().get(edge).unwrap().curve.unwrap();
    let CurveGeom::Circle(circle) = *transaction.store().get(curve_id).unwrap() else {
        panic!("contact edge must retain a circle")
    };
    let shifted = Circle::new(
        circle.frame().with_origin(circle.frame().origin() + delta),
        circle.radius(),
    )
    .unwrap();
    transaction
        .store_mut()
        .replace_curve(curve_id, CurveGeom::Circle(shifted))
        .unwrap();
}

#[test]
fn whole_ring_incidence_accepts_only_the_linear_envelope() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &nested_contact_input(Frame::world(), false, false, false, DEFAULT_INNER_OFFSET),
            TOLERANCE,
        )
        .unwrap();
    let inner_contact = edge_for_key(&output, INNER_CONTACT_EDGE.value());

    let mut inside = transaction.store().clone();
    let mut inside_edit = inside.transaction().unwrap();
    replace_circle_origin(
        &mut inside_edit,
        inner_contact,
        Frame::world().x() * (0.25 * LINEAR_RESOLUTION),
    );
    assert_eq!(
        certify_parallel_cylinder_contact_shell(inside_edit.store(), output.shell(), None).unwrap(),
        positive(),
    );

    let mut outside = transaction.store().clone();
    let mut outside_edit = outside.transaction().unwrap();
    replace_circle_origin(
        &mut outside_edit,
        inner_contact,
        Frame::world().x() * (2.0 * LINEAR_RESOLUTION),
    );
    assert_ne!(
        certify_parallel_cylinder_contact_shell(outside_edit.store(), output.shell(), None)
            .unwrap(),
        positive(),
    );
    assert_not_full_certified(outside_edit.store(), &output);
}

#[test]
fn cylinder_support_containment_is_independent_of_ring_containment() {
    let ring_offset = 1.0 - 2.5 * LINEAR_RESOLUTION;
    let cylinder_offset = 1.0 - 1.6 * LINEAR_RESOLUTION;
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &nested_contact_input(Frame::world(), false, false, false, ring_offset),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        certify_parallel_cylinder_contact_shell(transaction.store(), output.shell(), None).unwrap(),
        positive(),
    );

    let mut disagreement = transaction.store().clone();
    let mut disagreement_edit = disagreement.transaction().unwrap();
    let inner_face = face_for_key(&output, 1);
    let surface_id = disagreement_edit.store().get(inner_face).unwrap().surface;
    let SurfaceGeom::Cylinder(inner) = *disagreement_edit.store().get(surface_id).unwrap() else {
        panic!("inner face must retain its cylinder")
    };
    let moved = Cylinder::new(
        inner
            .frame()
            .with_origin(Point3::new(cylinder_offset, 0.0, 0.0)),
        inner.radius(),
    )
    .unwrap();
    disagreement_edit
        .store_mut()
        .replace_surface(surface_id, SurfaceGeom::Cylinder(moved))
        .unwrap();
    assert_ne!(
        certify_parallel_cylinder_contact_shell(disagreement_edit.store(), output.shell(), None,)
            .unwrap(),
        positive(),
    );
    assert_not_full_certified(disagreement_edit.store(), &output);
}

#[test]
fn axis_distance_encloses_cross_product_oracle_for_ill_conditioned_frame() {
    let axis = Vec3::new(1.0, 1.0, 1.0).normalized().unwrap();
    let frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        axis,
        axis + Vec3::new(1.0e-6, -1.0e-6, 0.0),
    )
    .unwrap();
    let radial = Vec3::new(1.0, -1.0, 0.0).normalized().unwrap();
    let displacement = frame.z() * 400.0 + radial;
    let point = frame.origin() + displacement;

    let distance = axis_distance_squared(point, frame.origin(), frame.z()).unwrap();
    let cross = displacement.cross(frame.z());
    let oracle = cross.norm_sq() / frame.z().norm_sq();
    let legacy_projection =
        displacement.dot(frame.x()).powi(2) + displacement.dot(frame.y()).powi(2);

    assert!(frame.x().dot(frame.z()).abs() > 1.0e-10);
    assert!(oracle - legacy_projection > 1.0e-8);
    assert!(distance.lo() <= oracle && oracle <= distance.hi());
}

#[test]
fn geometry_topology_incidence_extra_face_and_near_parallel_tampers_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &nested_contact_input(Frame::world(), false, false, false, DEFAULT_INNER_OFFSET),
            TOLERANCE,
        )
        .unwrap();
    let baseline = transaction.store();
    let outer_side = face_for_key(&output, 0);
    let inner_side = face_for_key(&output, 1);

    let mut wrong_geometry = baseline.clone();
    let mut wrong_geometry_edit = wrong_geometry.transaction().unwrap();
    let inner_contact = edge_for_key(&output, INNER_CONTACT_EDGE.value());
    let curve_id = wrong_geometry_edit
        .store()
        .get(inner_contact)
        .unwrap()
        .curve
        .unwrap();
    let CurveGeom::Circle(circle) = *wrong_geometry_edit.store().get(curve_id).unwrap() else {
        unreachable!()
    };
    wrong_geometry_edit
        .store_mut()
        .replace_curve(
            curve_id,
            CurveGeom::Circle(Circle::new(*circle.frame(), circle.radius() + 0.1).unwrap()),
        )
        .unwrap();
    assert_ne!(
        certify_parallel_cylinder_contact_shell(wrong_geometry_edit.store(), output.shell(), None,)
            .unwrap(),
        positive(),
    );
    assert_not_full_certified(wrong_geometry_edit.store(), &output);

    let mut wrong_topology = baseline.clone();
    let loop_id = wrong_topology.get(outer_side).unwrap().loops[0];
    let duplicate = wrong_topology.get(loop_id).unwrap().fins[0];
    wrong_topology
        .get_mut(loop_id)
        .unwrap()
        .fins
        .push(duplicate);
    assert_ne!(
        certify_parallel_cylinder_contact_shell(&wrong_topology, output.shell(), None).unwrap(),
        positive(),
    );
    assert_not_full_certified(&wrong_topology, &output);

    let mut wrong_incidence = baseline.clone();
    let contact_fin = wrong_incidence
        .get(inner_contact)
        .unwrap()
        .fins
        .iter()
        .copied()
        .find(|fin| {
            let loop_id = wrong_incidence.get(*fin).unwrap().parent;
            wrong_incidence.get(loop_id).unwrap().face == inner_side
        })
        .unwrap();
    wrong_incidence.get_mut(contact_fin).unwrap().sense =
        wrong_incidence.get(contact_fin).unwrap().sense.flipped();
    assert_ne!(
        certify_parallel_cylinder_contact_shell(&wrong_incidence, output.shell(), None).unwrap(),
        positive(),
    );
    assert_not_full_certified(&wrong_incidence, &output);

    let mut extra_face = baseline.clone();
    extra_face
        .get_mut(output.shell())
        .unwrap()
        .faces
        .push(outer_side);
    assert_ne!(
        certify_parallel_cylinder_contact_shell(&extra_face, output.shell(), None).unwrap(),
        positive(),
    );
    assert_not_full_certified(&extra_face, &output);

    let mut near_parallel = baseline.clone();
    let mut near_parallel_edit = near_parallel.transaction().unwrap();
    let surface_id = near_parallel_edit.store().get(inner_side).unwrap().surface;
    let SurfaceGeom::Cylinder(inner) = *near_parallel_edit.store().get(surface_id).unwrap() else {
        unreachable!()
    };
    let tilted_frame = Frame::new(
        inner.frame().origin(),
        inner.frame().z() + inner.frame().y() * (0.25 * ANGULAR_RESOLUTION),
        inner.frame().x(),
    )
    .unwrap();
    near_parallel_edit
        .store_mut()
        .replace_surface(
            surface_id,
            SurfaceGeom::Cylinder(Cylinder::new(tilted_frame, inner.radius()).unwrap()),
        )
        .unwrap();
    assert_ne!(
        certify_parallel_cylinder_contact_shell(near_parallel_edit.store(), output.shell(), None,)
            .unwrap(),
        positive(),
    );
    assert_not_full_certified(near_parallel_edit.store(), &output);
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
fn contact_work_accepts_1188_rejects_1187_and_zero_charges_inapplicable() {
    // Independent structural oracle: N = 1 shell + 5 faces + 8 loops + 8 uses.
    // The theorem contract is N^2 + 32N = 22^2 + 32*22 = 1,188.
    const SIZE: u64 = 1 + 5 + 8 + 8;
    const REQUIRED: u64 = SIZE * SIZE + 32 * SIZE;
    assert_eq!(SIZE, 22);
    assert_eq!(REQUIRED, 1_188);

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &nested_contact_input(Frame::world(), false, true, true, DEFAULT_INNER_OFFSET),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        proof_work(transaction.store(), output.shell()).unwrap(),
        Some(REQUIRED)
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
        .push(edge_for_key(&output, OUTER_FAR_EDGE.value()));
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
