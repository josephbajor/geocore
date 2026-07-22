use super::*;
use crate::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex,
    AnalyticVertexKey,
};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::FaceDomain;
use crate::transaction::FullCommitRequirement;
use kgeom::curve::{Circle, Line};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::surface::Plane;
use kgraph::AffineParamMap1d;

fn map(scale: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, 0.0).unwrap()
}

fn plane_circle_use(edge: u64, sense: Sense, plane: Plane, circle: Circle) -> AnalyticShellFin {
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
        AnalyticEdgeKey::new(edge),
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Circle(
                Circle2d::new(Point2::new(center.x, center.y), circle.radius(), local_x).unwrap(),
            ),
            map(scale),
        ),
    )
}

fn cylinder_arc_use(edge: u64, sense: Sense, height: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        AnalyticEdgeKey::new(edge),
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            map(1.0),
        ),
    )
}

fn cylinder_ruling_use(edge: u64, sense: Sense, longitude: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        AnalyticEdgeKey::new(edge),
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
            ),
            map(1.0),
        ),
    )
}

fn ring_cylinder_use(edge: u64, sense: Sense, height: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        AnalyticEdgeKey::new(edge),
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            map(1.0),
        )
        .with_closure_winding([1, 0]),
    )
}

fn ring_plane_use(edge: u64, sense: Sense, plane: Plane, circle: Circle) -> AnalyticShellFin {
    let use_ = plane_circle_use(edge, sense, plane, circle);
    AnalyticShellFin::new(
        use_.edge(),
        use_.sense(),
        use_.pcurve().with_closure_winding([0, 0]),
    )
}

fn six_face_parallel_cylinder_attachment(side: RadialSide) -> AnalyticShellInput {
    let radius = 1.0;
    let host_low = -2.0;
    let boss_low = -1.0;
    let boss_high = 1.0;
    let host_high = 2.0;
    let angle = core::f64::consts::PI / 3.0;
    let world = Frame::world();
    let host_frame = Frame::new(Point3::new(-0.5, 0.0, 0.0), world.z(), -world.x()).unwrap();
    let boss_frame = Frame::new(Point3::new(0.5, 0.0, 0.0), world.z(), -world.x()).unwrap();
    let host = Cylinder::new(host_frame, radius).unwrap();
    let boss = Cylinder::new(boss_frame, radius).unwrap();
    let host_circle = |height| {
        Circle::new(
            host_frame.with_origin(host_frame.point_at(0.0, 0.0, height)),
            radius,
        )
        .unwrap()
    };
    let boss_circle = |height| {
        Circle::new(
            boss_frame.with_origin(boss_frame.point_at(0.0, 0.0, height)),
            radius,
        )
        .unwrap()
    };
    let host_bottom = host_circle(boss_low);
    let host_top = host_circle(boss_high);
    let boss_bottom = boss_circle(boss_low);
    let boss_top = boss_circle(boss_high);
    let points = [
        host_bottom.eval(2.0 * angle),
        host_bottom.eval(4.0 * angle),
        host_top.eval(2.0 * angle),
        host_top.eval(4.0 * angle),
    ];
    let vertices = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), *point)
        })
        .collect::<Vec<_>>();

    let (attachment_vertices, attachment_range) = match side {
        RadialSide::Outside => ([[0, 1], [2, 3]], ParamRange::new(angle, 5.0 * angle)),
        RadialSide::Inside => ([[1, 0], [3, 2]], ParamRange::new(-angle, angle)),
    };
    let mut edges = vec![
        AnalyticShellEdge::new(
            AnalyticEdgeKey::new(0),
            [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
            AnalyticShellCurve::Circle(host_bottom),
            ParamRange::new(2.0 * angle, 4.0 * angle),
        ),
        AnalyticShellEdge::new(
            AnalyticEdgeKey::new(1),
            [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
            AnalyticShellCurve::Circle(host_top),
            ParamRange::new(2.0 * angle, 4.0 * angle),
        ),
        AnalyticShellEdge::new(
            AnalyticEdgeKey::new(4),
            attachment_vertices[0].map(AnalyticVertexKey::new),
            AnalyticShellCurve::Circle(boss_bottom),
            attachment_range,
        ),
        AnalyticShellEdge::new(
            AnalyticEdgeKey::new(5),
            attachment_vertices[1].map(AnalyticVertexKey::new),
            AnalyticShellCurve::Circle(boss_top),
            attachment_range,
        ),
    ];
    for (key, vertices) in [(2, [0, 2]), (3, [1, 3])] {
        let origin = points[vertices[0] as usize] - world.z() * boss_low;
        edges.push(AnalyticShellEdge::new(
            AnalyticEdgeKey::new(key),
            vertices.map(AnalyticVertexKey::new),
            AnalyticShellCurve::Line(Line::new(origin, world.z()).unwrap()),
            ParamRange::new(boss_low, boss_high),
        ));
    }

    let host_low_plane = Plane::new(
        Frame::new(
            host_frame.point_at(0.0, 0.0, host_low),
            -world.z(),
            -world.x(),
        )
        .unwrap(),
    );
    let host_high_plane =
        Plane::new(host_frame.with_origin(host_frame.point_at(0.0, 0.0, host_high)));
    let attachment_low_normal = match side {
        RadialSide::Outside => -world.z(),
        RadialSide::Inside => world.z(),
    };
    let attachment_high_normal = -attachment_low_normal;
    let boss_low_plane = Plane::new(
        Frame::new(
            host_frame.point_at(0.0, 0.0, boss_low),
            attachment_low_normal,
            -world.x(),
        )
        .unwrap(),
    );
    let boss_high_plane = Plane::new(
        Frame::new(
            host_frame.point_at(0.0, 0.0, boss_high),
            attachment_high_normal,
            -world.x(),
        )
        .unwrap(),
    );
    let (low_plane_attachment_sense, high_plane_attachment_sense) = match side {
        RadialSide::Outside => (Sense::Reversed, Sense::Forward),
        RadialSide::Inside => (Sense::Forward, Sense::Reversed),
    };
    let (attachment_low_sense, attachment_high_sense, attachment_face_sense) = match side {
        RadialSide::Outside => (Sense::Forward, Sense::Reversed, Sense::Forward),
        RadialSide::Inside => (Sense::Reversed, Sense::Forward, Sense::Reversed),
    };
    let attachment_domain = match side {
        RadialSide::Outside => {
            FaceDomain::from_bounds(0.0, core::f64::consts::TAU, boss_low, boss_high)
        }
        RadialSide::Inside => FaceDomain::from_bounds(-angle, angle, boss_low, boss_high),
    }
    .unwrap();
    let planar_domain = || FaceDomain::from_bounds(-3.0, 3.0, -3.0, 3.0).unwrap();

    AnalyticShellInput::new(
        vertices,
        edges,
        vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Cylinder(host),
                Sense::Forward,
                FaceDomain::from_bounds(0.0, core::f64::consts::TAU, host_low, host_high).unwrap(),
                vec![
                    AnalyticShellLoop::new(vec![
                        cylinder_arc_use(0, Sense::Reversed, boss_low),
                        cylinder_ruling_use(2, Sense::Forward, 2.0 * angle),
                        cylinder_arc_use(1, Sense::Forward, boss_high),
                        cylinder_ruling_use(3, Sense::Reversed, 4.0 * angle),
                    ]),
                    AnalyticShellLoop::new(vec![ring_cylinder_use(100, Sense::Forward, host_low)]),
                    AnalyticShellLoop::new(vec![ring_cylinder_use(
                        101,
                        Sense::Reversed,
                        host_high,
                    )]),
                ],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Plane(host_low_plane),
                Sense::Forward,
                planar_domain(),
                vec![AnalyticShellLoop::new(vec![ring_plane_use(
                    100,
                    Sense::Reversed,
                    host_low_plane,
                    host_circle(host_low),
                )])],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(host_high_plane),
                Sense::Forward,
                planar_domain(),
                vec![AnalyticShellLoop::new(vec![ring_plane_use(
                    101,
                    Sense::Forward,
                    host_high_plane,
                    host_circle(host_high),
                )])],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(3),
                AnalyticShellSurface::Plane(boss_low_plane),
                Sense::Forward,
                planar_domain(),
                vec![AnalyticShellLoop::new(vec![
                    plane_circle_use(0, Sense::Forward, boss_low_plane, host_bottom),
                    plane_circle_use(4, low_plane_attachment_sense, boss_low_plane, boss_bottom),
                ])],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(4),
                AnalyticShellSurface::Plane(boss_high_plane),
                Sense::Forward,
                planar_domain(),
                vec![AnalyticShellLoop::new(vec![
                    plane_circle_use(1, Sense::Reversed, boss_high_plane, host_top),
                    plane_circle_use(5, high_plane_attachment_sense, boss_high_plane, boss_top),
                ])],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(5),
                AnalyticShellSurface::Cylinder(boss),
                attachment_face_sense,
                attachment_domain,
                vec![AnalyticShellLoop::new(vec![
                    cylinder_arc_use(4, attachment_low_sense, boss_low),
                    cylinder_ruling_use(
                        3,
                        Sense::Forward,
                        match side {
                            RadialSide::Outside => 5.0 * angle,
                            RadialSide::Inside => -angle,
                        },
                    ),
                    cylinder_arc_use(5, attachment_high_sense, boss_high),
                    cylinder_ruling_use(2, Sense::Reversed, angle),
                ])],
            ),
        ],
    )
    .with_closed_edges(vec![
        AnalyticShellClosedEdge::new(
            AnalyticEdgeKey::new(100),
            AnalyticShellCurve::Circle(host_circle(host_low)),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ),
        AnalyticShellClosedEdge::new(
            AnalyticEdgeKey::new(101),
            AnalyticShellCurve::Circle(host_circle(host_high)),
            ParamRange::new(0.0, core::f64::consts::TAU),
        ),
    ])
}

fn six_face_parallel_cylinder_union() -> AnalyticShellInput {
    six_face_parallel_cylinder_attachment(RadialSide::Outside)
}

fn six_face_parallel_cylinder_outer_minus_inner() -> AnalyticShellInput {
    six_face_parallel_cylinder_attachment(RadialSide::Inside)
}

fn face_for_key(output: &crate::analytic_shell::AnalyticShellOutput, key: u64) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &crate::analytic_shell::AnalyticShellOutput, key: u64) -> EdgeId {
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

#[test]
fn six_face_parallel_cylinder_union_is_certified_and_checked() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&six_face_parallel_cylinder_union(), 1.0e-12)
        .unwrap();
    assert_eq!(
        transaction.store().get(output.shell()).unwrap().faces.len(),
        6
    );
    assert_eq!(
        certify_portal_cylinder_shell(transaction.store(), output.shell(), None).unwrap(),
        positive()
    );
    let report = check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:?}");
    transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
}

#[test]
fn six_face_parallel_cylinder_outer_minus_inner_is_certified_checked_and_order_independent() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&six_face_parallel_cylinder_outer_minus_inner(), 1.0e-12)
        .unwrap();
    assert_eq!(
        transaction.store().get(output.shell()).unwrap().faces.len(),
        6
    );
    assert_eq!(
        certify_portal_cylinder_shell(transaction.store(), output.shell(), None).unwrap(),
        positive()
    );
    let report = check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
    assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:?}");

    let mut reordered = transaction.store().clone();
    reordered.get_mut(output.shell()).unwrap().faces.reverse();
    assert_eq!(
        certify_portal_cylinder_shell(&reordered, output.shell(), None).unwrap(),
        positive()
    );

    transaction
        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
        .unwrap();
}

#[test]
fn cylindrical_host_selection_is_independent_of_face_order() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&six_face_parallel_cylinder_union(), 1.0e-12)
        .unwrap();
    transaction
        .store_mut()
        .get_mut(output.shell())
        .unwrap()
        .faces
        .reverse();
    assert_eq!(
        certify_portal_cylinder_shell(transaction.store(), output.shell(), None).unwrap(),
        positive()
    );
}

#[test]
fn circular_boss_tampers_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&six_face_parallel_cylinder_union(), 1.0e-12)
        .unwrap();

    let mut wrong_sense = transaction.store().clone();
    wrong_sense.get_mut(face_for_key(&output, 5)).unwrap().sense = Sense::Reversed;
    assert_eq!(
        certify_portal_cylinder_shell(&wrong_sense, output.shell(), None).unwrap(),
        Some(ShellCertification {
            embedding: ShellEmbedding::Certified,
            orientation: ShellOrientation::Invalid,
        })
    );

    let mut minor_span = transaction.store().clone();
    let angle = core::f64::consts::PI / 3.0;
    for key in [4, 5] {
        minor_span
            .get_mut(edge_for_key(&output, key))
            .unwrap()
            .bounds = Some((-angle, angle));
    }
    assert_ne!(
        certify_portal_cylinder_shell(&minor_span, output.shell(), None).unwrap(),
        positive()
    );

    let mut duplicate_side = transaction.store().clone();
    duplicate_side
        .get_mut(output.shell())
        .unwrap()
        .faces
        .push(face_for_key(&output, 5));
    assert_ne!(
        certify_portal_cylinder_shell(&duplicate_side, output.shell(), None).unwrap(),
        positive()
    );
}

fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
    let budget = BudgetPlan::new([LimitSpec::new(
        PORTAL_CYLINDER_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    kcore::operation::SessionPolicy::new(
        kcore::operation::SessionPrecision::parasolid(),
        kcore::operation::NumericalPolicy::v1(),
        kcore::operation::ExecutionPolicy::Serial,
        budget,
        kcore::operation::PolicyVersion::V1,
    )
}

#[test]
fn two_cylinder_host_scan_accepts_exact_work_and_rejects_n_minus_one() {
    // Independent fixture oracle: N = 1 + F + L + U + E + V = 43,
    // H = 2 Cylinder host candidates, and P = 4 Plane cap candidates.
    // H * (N^2 + 64N) * (C(P, 2) + 1) = 64_414.
    const REQUIRED_WORK: u64 = 64_414;
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(&six_face_parallel_cylinder_union(), 1.0e-12)
        .unwrap();
    assert_eq!(
        proof_work(transaction.store(), output.shell(), 2, 4).unwrap(),
        Some(REQUIRED_WORK)
    );

    let exact_policy = session_with_work(REQUIRED_WORK);
    let exact_context = kcore::operation::OperationContext::new(
        &exact_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut exact_scope = OperationScope::new(&exact_context);
    assert_eq!(
        certify_portal_cylinder_shell(transaction.store(), output.shell(), Some(&mut exact_scope),)
            .unwrap(),
        positive()
    );

    let denied_policy = session_with_work(REQUIRED_WORK - 1);
    let denied_context = kcore::operation::OperationContext::new(
        &denied_policy,
        kcore::tolerance::Tolerances::default(),
    )
    .unwrap();
    let mut denied_scope = OperationScope::new(&denied_context);
    let error =
        certify_portal_cylinder_shell(transaction.store(), output.shell(), Some(&mut denied_scope))
            .unwrap_err();
    assert_eq!(
        error.limit().map(|limit| limit.stage),
        Some(PORTAL_CYLINDER_SHELL_WORK)
    );
}
