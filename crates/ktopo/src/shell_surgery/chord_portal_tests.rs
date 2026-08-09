//! Regression fixtures for convex planar hosts with translated chord portals.

use super::*;

const CHORD_PORTAL_SHELL_WORK: StageId = SHELL_SURGERY_WORK;

fn proof_work(store: &Store, shell_id: ShellId) -> Result<Option<u64>> {
    let Some(size) = shell_proof_size(store, shell_id)? else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 32, 0, 1))
}

fn certify_chord_portal_shell(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    certify_shell_surgery(store, shell_id, scope)
}

mod tests {
    use super::*;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellEdge,
        AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
        AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::FaceDomain;
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve::{Circle, Curve, Line};
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::param::ParamRange;
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;
    use kgraph::AffineParamMap1d;

    fn parameter_map(scale: f64) -> AffineParamMap1d {
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
                parameter_map(1.0),
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
                parameter_map(scale),
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
                parameter_map(1.0),
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
                parameter_map(1.0),
            ),
        )
    }

    fn reverse_loop(loop_: AnalyticShellLoop) -> AnalyticShellLoop {
        AnalyticShellLoop::new(
            loop_
                .fins()
                .iter()
                .rev()
                .map(|fin| AnalyticShellFin::new(fin.edge(), fin.sense().flipped(), fin.pcurve()))
                .collect(),
        )
    }

    fn line_edge(
        key: u64,
        vertices: [u64; 2],
        start: Point3,
        end: Point3,
    ) -> (AnalyticShellEdge, Line) {
        let displacement = end - start;
        let line = Line::new(start, displacement).unwrap();
        (
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(key),
                vertices.map(AnalyticVertexKey::new),
                AnalyticShellCurve::Line(line),
                ParamRange::new(0.0, displacement.norm()),
            ),
            line,
        )
    }

    /// Fixture B/C cap crossing without retaining Boolean provenance. `pocket`
    /// chooses the reversed minor segment used by B-C; otherwise the major
    /// exterior segment is the attachment used by B union C.
    fn cap_crossing_input(pocket: bool) -> AnalyticShellInput {
        let x_low = 0.5;
        let x_high = 2.5;
        let y_low = -3.0;
        let y_high = 3.0;
        let z_low = -1.0;
        let z_high = 3.0;
        let host_points = [
            Point3::new(x_low, y_low, z_low),
            Point3::new(x_high, y_low, z_low),
            Point3::new(x_high, y_high, z_low),
            Point3::new(x_low, y_high, z_low),
            Point3::new(x_low, y_low, z_high),
            Point3::new(x_high, y_low, z_high),
            Point3::new(x_high, y_high, z_high),
            Point3::new(x_low, y_high, z_high),
        ];
        let cylinder_frame = Frame::world();
        let radius = 1.5;
        let cylinder = Cylinder::new(cylinder_frame, radius).unwrap();
        let bottom_circle = Circle::new(cylinder_frame, radius).unwrap();
        let top_frame = cylinder_frame.with_origin(Point3::new(0.0, 0.0, 2.0));
        let top_circle = Circle::new(top_frame, radius).unwrap();
        let alpha = kcore::math::atan2(2.0_f64.sqrt(), 0.5);
        let arc = if pocket {
            ParamRange::new(-alpha, alpha)
        } else {
            ParamRange::new(alpha, 2.0 * core::f64::consts::PI - alpha)
        };
        let feature_points = [
            bottom_circle.eval(arc.lo),
            bottom_circle.eval(arc.hi),
            top_circle.eval(arc.lo),
            top_circle.eval(arc.hi),
        ];
        let vertices = host_points
            .into_iter()
            .chain(feature_points)
            .enumerate()
            .map(|(index, position)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), position)
            })
            .collect::<Vec<_>>();

        let host_edge_vertices = [
            [0, 1],
            [1, 2],
            [2, 3],
            [3, 0],
            [4, 5],
            [5, 6],
            [6, 7],
            [7, 4],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ];
        let mut edges = Vec::new();
        let mut lines = Vec::new();
        for (index, endpoints) in host_edge_vertices.into_iter().enumerate() {
            let (edge, line) = line_edge(
                index as u64,
                endpoints,
                host_points[endpoints[0] as usize],
                host_points[endpoints[1] as usize],
            );
            edges.push(edge);
            lines.push(line);
        }
        edges.push(AnalyticShellEdge::new(
            AnalyticEdgeKey::new(12),
            [AnalyticVertexKey::new(8), AnalyticVertexKey::new(9)],
            AnalyticShellCurve::Circle(bottom_circle),
            arc,
        ));
        edges.push(AnalyticShellEdge::new(
            AnalyticEdgeKey::new(13),
            [AnalyticVertexKey::new(10), AnalyticVertexKey::new(11)],
            AnalyticShellCurve::Circle(top_circle),
            arc,
        ));
        let (ruling_first, line_14) = line_edge(14, [8, 10], feature_points[0], feature_points[2]);
        let (ruling_second, line_15) = line_edge(15, [9, 11], feature_points[1], feature_points[3]);
        let (bottom_chord, line_16) = line_edge(16, [8, 9], feature_points[0], feature_points[1]);
        let (top_chord, line_17) = line_edge(17, [10, 11], feature_points[2], feature_points[3]);
        edges.extend([ruling_first, ruling_second, bottom_chord, top_chord]);

        let bottom_host = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let top_host = Plane::new(
            Frame::new(
                host_points[4],
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let x_low_plane = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(-1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        );
        let x_high_plane = Plane::new(
            Frame::new(
                host_points[1],
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let y_low_plane = Plane::new(
            Frame::new(
                host_points[0],
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let y_high_plane = Plane::new(
            Frame::new(
                host_points[3],
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            )
            .unwrap(),
        );
        let bottom_cap = Plane::new(
            Frame::new(
                feature_points[0],
                Vec3::new(0.0, 0.0, -1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        );
        let top_cap = Plane::new(top_frame);

        let host_loop = |plane: Plane, uses: &[(u64, Sense)]| {
            AnalyticShellLoop::new(
                uses.iter()
                    .map(|&(edge, sense)| plane_line_use(edge, sense, plane, lines[edge as usize]))
                    .collect(),
            )
        };
        let bottom_outer = host_loop(
            bottom_host,
            &[
                (3, Sense::Reversed),
                (2, Sense::Reversed),
                (1, Sense::Reversed),
                (0, Sense::Reversed),
            ],
        );
        let top_outer = host_loop(
            top_host,
            &[
                (4, Sense::Forward),
                (5, Sense::Forward),
                (6, Sense::Forward),
                (7, Sense::Forward),
            ],
        );
        let x_low_outer = host_loop(
            x_low_plane,
            &[
                (8, Sense::Forward),
                (7, Sense::Reversed),
                (11, Sense::Reversed),
                (3, Sense::Forward),
            ],
        );
        let x_high_outer = host_loop(
            x_high_plane,
            &[
                (1, Sense::Forward),
                (10, Sense::Forward),
                (5, Sense::Reversed),
                (9, Sense::Reversed),
            ],
        );
        let y_low_outer = host_loop(
            y_low_plane,
            &[
                (0, Sense::Forward),
                (9, Sense::Forward),
                (4, Sense::Reversed),
                (8, Sense::Reversed),
            ],
        );
        let y_high_outer = host_loop(
            y_high_plane,
            &[
                (11, Sense::Forward),
                (6, Sense::Reversed),
                (10, Sense::Reversed),
                (2, Sense::Forward),
            ],
        );

        let cylinder_loop = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(14, Sense::Reversed, arc.lo),
            cylinder_arc_use(12, Sense::Forward, 0.0),
            cylinder_ruling_use(15, Sense::Forward, arc.hi),
            cylinder_arc_use(13, Sense::Reversed, 2.0),
        ]);
        let bottom_cap_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(12, Sense::Reversed, bottom_cap, bottom_circle),
            plane_line_use(16, Sense::Forward, bottom_cap, line_16),
        ]);
        let top_cap_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(13, Sense::Forward, top_cap, top_circle),
            plane_line_use(17, Sense::Reversed, top_cap, line_17),
        ]);
        let portal_loop = AnalyticShellLoop::new(vec![
            plane_line_use(16, Sense::Reversed, x_low_plane, line_16),
            plane_line_use(14, Sense::Forward, x_low_plane, line_14),
            plane_line_use(17, Sense::Forward, x_low_plane, line_17),
            plane_line_use(15, Sense::Reversed, x_low_plane, line_15),
        ]);
        let (cylinder_loop, bottom_cap_loop, top_cap_loop, portal_loop, patch_sense) = if pocket {
            (
                reverse_loop(cylinder_loop),
                reverse_loop(bottom_cap_loop),
                reverse_loop(top_cap_loop),
                reverse_loop(portal_loop),
                Sense::Reversed,
            )
        } else {
            (
                cylinder_loop,
                bottom_cap_loop,
                top_cap_loop,
                portal_loop,
                Sense::Forward,
            )
        };
        let wide = || FaceDomain::from_bounds(-10.0, 10.0, -10.0, 10.0).unwrap();
        let faces = vec![
            AnalyticShellFace::new(
                AnalyticFaceKey::new(0),
                AnalyticShellSurface::Plane(bottom_host),
                Sense::Forward,
                wide(),
                vec![bottom_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(1),
                AnalyticShellSurface::Plane(top_host),
                Sense::Forward,
                wide(),
                vec![top_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(2),
                AnalyticShellSurface::Plane(x_low_plane),
                Sense::Forward,
                wide(),
                vec![x_low_outer, portal_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(3),
                AnalyticShellSurface::Plane(x_high_plane),
                Sense::Forward,
                wide(),
                vec![x_high_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(4),
                AnalyticShellSurface::Plane(y_low_plane),
                Sense::Forward,
                wide(),
                vec![y_low_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(5),
                AnalyticShellSurface::Plane(y_high_plane),
                Sense::Forward,
                wide(),
                vec![y_high_outer],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(6),
                AnalyticShellSurface::Cylinder(cylinder),
                patch_sense,
                FaceDomain::from_bounds(arc.lo, arc.hi, 0.0, 2.0).unwrap(),
                vec![cylinder_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(7),
                AnalyticShellSurface::Plane(bottom_cap),
                patch_sense,
                wide(),
                vec![bottom_cap_loop],
            ),
            AnalyticShellFace::new(
                AnalyticFaceKey::new(8),
                AnalyticShellSurface::Plane(top_cap),
                patch_sense,
                wide(),
                vec![top_cap_loop],
            ),
        ];
        AnalyticShellInput::new(vertices, edges, faces)
    }

    fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            CHORD_PORTAL_SHELL_WORK,
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
    fn cap_crossing_attachment_and_pocket_are_full_certified() {
        for pocket in [false, true] {
            let mut store = Store::new();
            let mut transaction = store.transaction().unwrap();
            let output = transaction
                .assemble_analytic_shell(&cap_crossing_input(pocket), 1.0e-12)
                .unwrap();
            assert_eq!(
                certify_chord_portal_shell(transaction.store(), output.shell(), None).unwrap(),
                Some(ShellCertification {
                    embedding: ShellEmbedding::Certified,
                    orientation: ShellOrientation::Positive,
                }),
                "pocket={pocket}"
            );
            super::super::assert_chord_portal_evidence_claims_are_rechecked(
                transaction.store(),
                output.shell(),
            );
            let report =
                check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
            assert_eq!(
                report.outcome(),
                CheckOutcome::Valid,
                "pocket={pocket}: {report:#?}"
            );
            transaction
                .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                .unwrap();
        }
    }

    #[test]
    fn chord_portal_tampering_fails_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&cap_crossing_input(false), 1.0e-12)
            .unwrap();
        let baseline = transaction.store().clone();
        let face = |key: u64| {
            output
                .faces()
                .iter()
                .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                .unwrap()
        };

        let mut sense = baseline.clone();
        sense.get_mut(face(6)).unwrap().sense = Sense::Reversed;
        assert_ne!(
            certify_chord_portal_shell(&sense, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let mut geometry = baseline.clone();
        let mut geometry_edit = geometry.transaction().unwrap();
        let cylinder_surface = geometry_edit.store().get(face(6)).unwrap().surface;
        let SurfaceGeom::Cylinder(cylinder) = *geometry_edit.store().get(cylinder_surface).unwrap()
        else {
            unreachable!()
        };
        geometry_edit
            .store_mut()
            .replace_surface(
                cylinder_surface,
                SurfaceGeom::Cylinder(
                    Cylinder::new(*cylinder.frame(), cylinder.radius() + 0.1).unwrap(),
                ),
            )
            .unwrap();
        assert_ne!(
            certify_chord_portal_shell(geometry_edit.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );

        let mut topology = baseline;
        let portal_loop = topology.get(face(2)).unwrap().loops[1];
        let duplicate = topology.get(portal_loop).unwrap().fins[0];
        topology.get_mut(portal_loop).unwrap().fins.push(duplicate);
        assert_ne!(
            certify_chord_portal_shell(&topology, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
    }

    #[test]
    fn chord_portal_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&cap_crossing_input(false), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell())
            .unwrap()
            .unwrap();
        for allowed in [required, required - 1] {
            let session = session_with_work(allowed);
            let context = kcore::operation::OperationContext::new(
                &session,
                kcore::tolerance::Tolerances::default(),
            )
            .unwrap();
            let mut scope = OperationScope::new(&context);
            let result =
                certify_chord_portal_shell(transaction.store(), output.shell(), Some(&mut scope));
            if allowed == required {
                assert_eq!(
                    result.unwrap().unwrap(),
                    ShellCertification {
                        embedding: ShellEmbedding::Certified,
                        orientation: ShellOrientation::Positive,
                    }
                );
            } else {
                assert_eq!(
                    result.unwrap_err().limit().map(|limit| limit.stage),
                    Some(CHORD_PORTAL_SHELL_WORK)
                );
            }
        }
    }
}
