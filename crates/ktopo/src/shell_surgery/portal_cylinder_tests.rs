//! Regression fixtures for periodic hosts with rectangular surgery portals.

use super::super::shell_lemmas::RadialSide;
use super::*;
use kgeom::param::ParamRange;
use kgeom::vec::Vec2;

const PORTAL_CYLINDER_SHELL_WORK: StageId = SHELL_SURGERY_WORK;

fn proof_work(
    store: &Store,
    shell_id: ShellId,
    host_count: usize,
    plane_count: usize,
) -> Result<Option<u64>> {
    let Some(size) = shell_proof_size(store, shell_id)? else {
        return Ok(None);
    };
    let (Some(hosts), Some(planes)) = (
        u64::try_from(host_count).ok(),
        u64::try_from(plane_count).ok(),
    ) else {
        return Ok(None);
    };
    let Some(pairs) = planes
        .checked_mul(planes.saturating_sub(1))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    let Some(pair_groups) = pairs.checked_add(1) else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 64, 0, pair_groups)
        .and_then(|per_host| per_host.checked_mul(hosts)))
}

fn certify_portal_cylinder_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    certify_shell_surgery(store, shell_id, scope)
}

#[path = "portal_cylinder_analytic_boss_tests.rs"]
mod analytic_boss_tests;

mod tests {
    use super::*;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
        AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin,
        AnalyticShellInput, AnalyticShellLoop, AnalyticShellPcurve, AnalyticShellSurface,
        AnalyticShellVertex, AnalyticVertexKey,
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

    fn plane_line_use(edge: u64, sense: Sense, plane: Plane, line: Line) -> AnalyticShellFin {
        let origin = plane.frame().to_local(line.origin());
        let direction = line.dir();
        AnalyticShellFin::new(
            AnalyticEdgeKey::new(edge),
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(
                        Point2::new(origin.x, origin.y),
                        Vec2::new(
                            direction.dot(plane.frame().x()),
                            direction.dot(plane.frame().y()),
                        ),
                    )
                    .unwrap(),
                ),
                map(1.0),
            ),
        )
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
                    Circle2d::new(Point2::new(center.x, center.y), circle.radius(), local_x)
                        .unwrap(),
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

    fn portal_shell_input() -> AnalyticShellInput {
        let radius: f64 = 1.5;
        let low = 0.0;
        let cut_low = 0.5;
        let cut_high = 1.5;
        let high = 2.0;
        let angle = kcore::math::atan2((radius * radius - 1.0).sqrt(), 1.0);
        let opposite = core::f64::consts::PI - angle;
        let lower_left = core::f64::consts::PI + angle;
        let lower_right = core::f64::consts::TAU - angle;
        let frame = Frame::world();
        let cylinder = Cylinder::new(frame, radius).unwrap();
        let circle_at = |height| {
            Circle::new(frame.with_origin(frame.point_at(0.0, 0.0, height)), radius).unwrap()
        };
        let cut_low_circle = circle_at(cut_low);
        let cut_high_circle = circle_at(cut_high);
        let points = [
            cut_low_circle.eval(angle),
            cut_low_circle.eval(opposite),
            cut_low_circle.eval(lower_left),
            cut_low_circle.eval(lower_right),
            cut_high_circle.eval(angle),
            cut_high_circle.eval(opposite),
            cut_high_circle.eval(lower_left),
            cut_high_circle.eval(lower_right),
        ];
        let vertices = points
            .iter()
            .enumerate()
            .map(|(index, point)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), *point)
            })
            .collect::<Vec<_>>();

        let mut edges = Vec::new();
        for (key, vertices, circle, range) in [
            (0, [0, 1], cut_low_circle, ParamRange::new(angle, opposite)),
            (
                1,
                [2, 3],
                cut_low_circle,
                ParamRange::new(lower_left, lower_right),
            ),
            (2, [4, 5], cut_high_circle, ParamRange::new(angle, opposite)),
            (
                3,
                [6, 7],
                cut_high_circle,
                ParamRange::new(lower_left, lower_right),
            ),
        ] {
            edges.push(AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Circle(circle),
                range,
            ));
        }
        for (key, vertices) in [
            (4, [0, 4]),
            (5, [1, 5]),
            (6, [2, 6]),
            (7, [3, 7]),
            (8, [1, 2]),
            (9, [3, 0]),
            (10, [5, 6]),
            (11, [7, 4]),
        ] {
            let start = points[vertices[0] as usize];
            let end = points[vertices[1] as usize];
            let direction = end - start;
            let length = direction.norm();
            let (line, range) = if key < 8 {
                (
                    Line::new(start - frame.z() * cut_low, frame.z()).unwrap(),
                    ParamRange::new(cut_low, cut_high),
                )
            } else {
                (
                    Line::new(start, direction).unwrap(),
                    ParamRange::new(0.0, length),
                )
            };
            edges.push(AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Line(line),
                range,
            ));
        }

        let low_plane =
            Plane::new(Frame::new(frame.point_at(0.0, 0.0, low), -frame.z(), frame.x()).unwrap());
        let high_plane = Plane::new(frame.with_origin(frame.point_at(0.0, 0.0, high)));
        let cut_low_plane = Plane::new(frame.with_origin(frame.point_at(0.0, 0.0, cut_low)));
        let cut_high_plane = Plane::new(
            Frame::new(frame.point_at(0.0, 0.0, cut_high), -frame.z(), frame.x()).unwrap(),
        );
        let right_plane = Plane::new(Frame::new(points[0], -frame.x(), frame.y()).unwrap());
        let left_plane = Plane::new(Frame::new(points[1], frame.x(), -frame.y()).unwrap());
        let line = |edge: usize| match edges[edge].carrier() {
            AnalyticShellCurve::Line(line) => line,
            _ => unreachable!(),
        };

        let host_loops = vec![
            AnalyticShellLoop::new(vec![
                cylinder_arc_use(0, Sense::Reversed, cut_low),
                cylinder_ruling_use(4, Sense::Forward, angle),
                cylinder_arc_use(2, Sense::Forward, cut_high),
                cylinder_ruling_use(5, Sense::Reversed, opposite),
            ]),
            AnalyticShellLoop::new(vec![
                cylinder_arc_use(1, Sense::Reversed, cut_low),
                cylinder_ruling_use(6, Sense::Forward, lower_left),
                cylinder_arc_use(3, Sense::Forward, cut_high),
                cylinder_ruling_use(7, Sense::Reversed, lower_right),
            ]),
            AnalyticShellLoop::new(vec![ring_cylinder_use(100, Sense::Forward, low)]),
            AnalyticShellLoop::new(vec![ring_cylinder_use(101, Sense::Reversed, high)]),
        ];
        let cut_low_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(0, Sense::Forward, cut_low_plane, cut_low_circle),
            plane_line_use(8, Sense::Forward, cut_low_plane, line(8)),
            plane_circle_use(1, Sense::Forward, cut_low_plane, cut_low_circle),
            plane_line_use(9, Sense::Forward, cut_low_plane, line(9)),
        ]);
        let cut_high_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(2, Sense::Reversed, cut_high_plane, cut_high_circle),
            plane_line_use(11, Sense::Reversed, cut_high_plane, line(11)),
            plane_circle_use(3, Sense::Reversed, cut_high_plane, cut_high_circle),
            plane_line_use(10, Sense::Reversed, cut_high_plane, line(10)),
        ]);
        let right_loop = AnalyticShellLoop::new(vec![
            plane_line_use(9, Sense::Reversed, right_plane, line(9)),
            plane_line_use(7, Sense::Forward, right_plane, line(7)),
            plane_line_use(11, Sense::Forward, right_plane, line(11)),
            plane_line_use(4, Sense::Reversed, right_plane, line(4)),
        ]);
        let left_loop = AnalyticShellLoop::new(vec![
            plane_line_use(8, Sense::Reversed, left_plane, line(8)),
            plane_line_use(5, Sense::Forward, left_plane, line(5)),
            plane_line_use(10, Sense::Forward, left_plane, line(10)),
            plane_line_use(6, Sense::Reversed, left_plane, line(6)),
        ]);
        let domain = || FaceDomain::from_bounds(-4.0, 4.0, -4.0, 4.0).unwrap();
        AnalyticShellInput::new(
            vertices,
            edges,
            vec![
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(0),
                    AnalyticShellSurface::Cylinder(cylinder),
                    Sense::Forward,
                    FaceDomain::from_bounds(0.0, core::f64::consts::TAU, low, high).unwrap(),
                    host_loops,
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(1),
                    AnalyticShellSurface::Plane(low_plane),
                    Sense::Forward,
                    domain(),
                    vec![AnalyticShellLoop::new(vec![ring_plane_use(
                        100,
                        Sense::Reversed,
                        low_plane,
                        circle_at(low),
                    )])],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(2),
                    AnalyticShellSurface::Plane(high_plane),
                    Sense::Forward,
                    domain(),
                    vec![AnalyticShellLoop::new(vec![ring_plane_use(
                        101,
                        Sense::Forward,
                        high_plane,
                        circle_at(high),
                    )])],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(3),
                    AnalyticShellSurface::Plane(cut_low_plane),
                    Sense::Forward,
                    domain(),
                    vec![cut_low_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(4),
                    AnalyticShellSurface::Plane(cut_high_plane),
                    Sense::Forward,
                    domain(),
                    vec![cut_high_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(5),
                    AnalyticShellSurface::Plane(right_plane),
                    Sense::Forward,
                    domain(),
                    vec![right_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(6),
                    AnalyticShellSurface::Plane(left_plane),
                    Sense::Forward,
                    domain(),
                    vec![left_loop],
                ),
            ],
        )
        .with_closed_edges(vec![
            AnalyticShellClosedEdge::new(
                AnalyticEdgeKey::new(100),
                AnalyticShellCurve::Circle(circle_at(low)),
                ParamRange::new(0.0, core::f64::consts::TAU),
            ),
            AnalyticShellClosedEdge::new(
                AnalyticEdgeKey::new(101),
                AnalyticShellCurve::Circle(circle_at(high)),
                ParamRange::new(0.0, core::f64::consts::TAU),
            ),
        ])
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

    #[test]
    fn interior_two_portal_component_is_certified_and_checked() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_portal_cylinder_shell(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
        let report =
            check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Valid, "{report:?}");
        transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
    }

    #[test]
    fn portal_shell_face_sense_tamper_is_orientation_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        let mut tampered = transaction.store().clone();
        tampered.get_mut(face_for_key(&output, 5)).unwrap().sense = Sense::Reversed;
        assert_eq!(
            certify_portal_cylinder_shell(&tampered, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );
    }

    #[test]
    fn portal_shell_ring_direction_and_host_geometry_tampering_fail_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();

        let mut wrong_ring = transaction.store().clone();
        let high_ring = edge_for_key(&output, 101);
        let fins = wrong_ring.get(high_ring).unwrap().fins.clone();
        let host = face_for_key(&output, 0);
        for fin in fins {
            let face = wrong_ring
                .get(wrong_ring.get(fin).unwrap().parent)
                .unwrap()
                .face;
            wrong_ring.get_mut(fin).unwrap().sense = if face == host {
                Sense::Forward
            } else {
                Sense::Reversed
            };
        }
        assert_eq!(
            certify_portal_cylinder_shell(&wrong_ring, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );

        let mut wrong_host = transaction.store().clone();
        let surface = wrong_host.get(host).unwrap().surface;
        let SurfaceGeom::Cylinder(cylinder) = *wrong_host.get(surface).unwrap() else {
            unreachable!()
        };
        let changed = Cylinder::new(*cylinder.frame(), 1.6).unwrap();
        let mut edit = wrong_host.transaction().unwrap();
        edit.assembly()
            .replace_surface(surface, SurfaceGeom::Cylinder(changed))
            .unwrap();
        assert_eq!(
            certify_portal_cylinder_shell(edit.store(), output.shell(), None).unwrap(),
            None
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
    fn portal_shell_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&portal_shell_input(), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell(), 1, 6)
            .unwrap()
            .unwrap();

        let exact_policy = session_with_work(required);
        let exact_context = kcore::operation::OperationContext::new(
            &exact_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut exact_scope = OperationScope::new(&exact_context);
        assert_eq!(
            certify_portal_cylinder_shell(
                transaction.store(),
                output.shell(),
                Some(&mut exact_scope),
            )
            .unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let denied_policy = session_with_work(required - 1);
        let denied_context = kcore::operation::OperationContext::new(
            &denied_policy,
            kcore::tolerance::Tolerances::default(),
        )
        .unwrap();
        let mut denied_scope = OperationScope::new(&denied_context);
        let error = certify_portal_cylinder_shell(
            transaction.store(),
            output.shell(),
            Some(&mut denied_scope),
        )
        .unwrap_err();
        assert_eq!(
            error.limit().map(|limit| limit.stage),
            Some(PORTAL_CYLINDER_SHELL_WORK)
        );
    }
}
