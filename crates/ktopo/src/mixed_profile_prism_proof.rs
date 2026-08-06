//! Full shell theorem for transverse translation sweeps of analytic profiles.
//!
//! R2 decomposition: certified simple Plane loops with certified strict hole
//! containment bound one Jordan material region `D`; a second cap must be its
//! bijective nonzero transverse translation; every remaining face must be
//! exactly one four-edge product strip over one cap edge. Bounded Line edges
//! require Plane strips and bounded Circle edges require Cylinder strips,
//! while the two other strip edges must be complete translation rulings.
//! Whole-fin incidence proves the authored pcurves over every complete edge range.
//! These local witnesses identify the shell with `boundary(D x [0,1])`, so
//! global embedding follows without convexity, layout tags, constructor
//! provenance, or sampled sidedness. Any unsupported carrier, ambiguous
//! correspondence, incomplete strip, or inconclusive interval comparison
//! returns no theorem.

use super::shell_lemmas::{
    certified_parallel, certify_sweep_support, edge_has_vertices, mapped_vertex, oriented_dot_sign,
    peer_face, prepare_profile_cap, prepare_side, ruling_connects, translated_carrier,
    translated_profile_vertices,
};
use super::shell_lemmas::{indeterminate, proof_work_budget};
use super::shell_lemmas::{proof_work as quadratic_proof_work, shell_proof_size};
use super::*;

/// Cumulative deterministic work for mixed analytic profile-prism proofs.
pub(crate) const MIXED_PROFILE_PRISM_WORK: StageId =
    match StageId::new("ktopo.check.mixed-profile-prism-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid mixed profile-prism work stage"),
    };

// X_T reconstruction intentionally retains representation rather than
// operation provenance. A reconstructed multi-portal result can therefore
// reach this general product-shell theorem instead of the more specialized
// portal-surgery theorem used during its original checked commit. The first
// such public five-support result charges 12_418_560 exact work units here,
// so keep the shared v1 checker default at the smallest power-of-two ceiling
// that admits both representation paths without changing the size-derived
// work formula or weakening caller overrides.
const DEFAULT_MIXED_PROFILE_PRISM_WORK: u64 = 16_777_216;

pub(super) fn mixed_profile_prism_proof_budget() -> BudgetPlan {
    proof_work_budget(
        MIXED_PROFILE_PRISM_WORK,
        DEFAULT_MIXED_PROFILE_PRISM_WORK,
        "built-in mixed profile-prism proof budget is valid",
    )
}

/// Attempt the representation-independent product-shell theorem.
pub(super) fn certify_mixed_profile_prism(
    store: &Store,
    shell_id: ShellId,
    scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 4 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    let mut planar_faces = Vec::new();
    let mut planar_cap_candidates = Vec::new();
    let mut has_cylinder = false;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Plane(_) => {
                planar_faces.push(face_id);
                let mut boundary_uses = 0_usize;
                for &loop_id in &face.loops {
                    let Some(total) = boundary_uses.checked_add(store.get(loop_id)?.fins.len())
                    else {
                        return Ok(None);
                    };
                    boundary_uses = total;
                }
                if boundary_uses.checked_add(2) == Some(shell.faces.len()) {
                    planar_cap_candidates.push(face_id);
                }
            }
            SurfaceGeom::Cylinder(_) => has_cylinder = true,
            _ => return Ok(None),
        }
    }
    let cap_candidates = if has_cylinder {
        &planar_faces
    } else {
        &planar_cap_candidates
    };
    if cap_candidates.len() < 2 {
        return Ok(None);
    }

    if let Some(scope) = scope {
        scope.ledger().require_limit(
            MIXED_PROFILE_PRISM_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        let Some(work) = mixed_profile_proof_work(store, shell_id, cap_candidates.len())? else {
            return Ok(Some(indeterminate()));
        };
        scope.ledger_mut().charge(MIXED_PROFILE_PRISM_WORK, work)?;
    }

    for (index, &first) in cap_candidates.iter().enumerate() {
        for &second in &cap_candidates[index + 1..] {
            if let Some(candidate) = certify_cap_pair(store, shell_id, first, second)? {
                // Existence of one complete D x [0,1] decomposition is the
                // embedding witness. Highly symmetric all-planar prisms can
                // have several equally authoritative sweep axes.
                return Ok(Some(candidate));
            }
        }
    }
    Ok(None)
}

/// Checked upper bound for cap-pair search and all structural comparisons.
///
/// With `N = 1 + F + L + U + E + V`, every planar face pair performs at
/// most `N^2 + 16N` visits/comparisons. Multiplying by the exact unordered
/// planar-pair count bounds vertex matching, edge/side bijections, loop scans,
/// and carrier checks before the search allocates any topology.
fn mixed_profile_proof_work(
    store: &Store,
    shell_id: ShellId,
    plane_count: usize,
) -> Result<Option<u64>> {
    let Some(size) = shell_proof_size(store, shell_id)? else {
        return Ok(None);
    };
    let Some(planes) = u64::try_from(plane_count).ok() else {
        return Ok(None);
    };
    let Some(pair_count) = planes
        .checked_sub(1)
        .and_then(|less| planes.checked_mul(less))
        .map(|ordered| ordered / 2)
    else {
        return Ok(None);
    };
    Ok(quadratic_proof_work(size, 16, 0, pair_count))
}

#[cfg(test)]
use mixed_profile_proof_work as proof_work;
fn certify_cap_pair(
    store: &Store,
    shell_id: ShellId,
    first: FaceId,
    second: FaceId,
) -> Result<Option<ShellCertification>> {
    let Some(first) = prepare_profile_cap(store, first)? else {
        return Ok(None);
    };
    let Some(second) = prepare_profile_cap(store, second)? else {
        return Ok(None);
    };
    let shell = store.get(shell_id)?;
    if first.uses.len() != second.uses.len()
        || first.vertices.len() != second.vertices.len()
        || shell.faces.len() != first.uses.len() + 2
    {
        return Ok(None);
    }
    let Some(translation) = translated_profile_vertices(store, &first, &second)? else {
        return Ok(None);
    };
    if !certified_parallel(first.plane.frame().z(), second.plane.frame().z()) {
        return Ok(None);
    }

    let first_sign = oriented_dot_sign(
        first.plane.frame().z() * sense_factor(store.get(first.face)?.sense),
        -translation.vector,
    );
    let second_sign = oriented_dot_sign(
        second.plane.frame().z() * sense_factor(store.get(second.face)?.sense),
        translation.vector,
    );
    let (Some(first_sign), Some(second_sign)) = (first_sign, second_sign) else {
        return Ok(None);
    };
    let mut orientation_signs = vec![first_sign, second_sign];
    let mut orientation_invalid = !first.local_orientation_valid || !second.local_orientation_valid;

    let second_edges = second.uses.iter().map(|use_| use_.edge).collect::<Vec<_>>();
    let side_faces = shell
        .faces
        .iter()
        .copied()
        .filter(|face| *face != first.face && *face != second.face)
        .collect::<Vec<_>>();
    let mut used_sides = Vec::with_capacity(side_faces.len());
    let mut used_second_edges = Vec::with_capacity(second_edges.len());
    for boundary in &first.uses {
        let Some(side_face_id) = peer_face(store, *boundary)? else {
            return Ok(None);
        };
        if !side_faces.contains(&side_face_id) || used_sides.contains(&side_face_id) {
            return Ok(None);
        }
        let Some(side) = prepare_side(store, side_face_id)? else {
            return Ok(None);
        };
        let Some(mapped_tail) = mapped_vertex(&translation.vertices, boundary.tail) else {
            return Ok(None);
        };
        let Some(mapped_head) = mapped_vertex(&translation.vertices, boundary.head) else {
            return Ok(None);
        };
        let mut matching_top = Vec::new();
        for candidate in &second.uses {
            if edge_has_vertices(store, candidate.edge, mapped_tail, mapped_head)?
                && translated_carrier(*boundary, *candidate, translation.vector)
            {
                matching_top.push(candidate);
            }
        }
        let [mapped_top] = matching_top.as_slice() else {
            return Ok(None);
        };
        if used_second_edges.contains(&mapped_top.edge)
            || !side.fins.iter().any(|(_, edge)| *edge == boundary.edge)
            || !side.fins.iter().any(|(_, edge)| *edge == mapped_top.edge)
        {
            return Ok(None);
        }
        let rulings = side
            .fins
            .iter()
            .copied()
            .filter(|(_, edge)| *edge != boundary.edge && *edge != mapped_top.edge)
            .collect::<Vec<_>>();
        let [first_ruling, second_ruling] = rulings.as_slice() else {
            return Ok(None);
        };
        let valid_rulings = (ruling_connects(
            store,
            first_ruling.1,
            boundary.tail,
            mapped_tail,
            translation.vector,
        )? && ruling_connects(
            store,
            second_ruling.1,
            boundary.head,
            mapped_head,
            translation.vector,
        )?) || (ruling_connects(
            store,
            first_ruling.1,
            boundary.head,
            mapped_head,
            translation.vector,
        )? && ruling_connects(
            store,
            second_ruling.1,
            boundary.tail,
            mapped_tail,
            translation.vector,
        )?);
        if !valid_rulings {
            return Ok(None);
        }
        let Some(side_sign) =
            certify_sweep_support(store, &side, *boundary, **mapped_top, translation.vector)?
        else {
            return Ok(None);
        };
        orientation_signs.push(side_sign);
        used_sides.push(side.face);
        used_second_edges.push(mapped_top.edge);
    }
    if used_sides.len() != side_faces.len() || used_second_edges.len() != second_edges.len() {
        return Ok(None);
    }
    orientation_invalid |= orientation_signs
        .iter()
        .any(|sign| *sign != orientation_signs[0]);
    let orientation = if orientation_invalid {
        ShellOrientation::Invalid
    } else if orientation_signs[0] > 0 {
        ShellOrientation::Positive
    } else {
        ShellOrientation::Negative
    };
    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::analytic_shell::cylinder_cylinder_tests::{
        source_ring_parallel_cylinder_lens_input, unsplit_parallel_cylinder_lens_input,
    };
    use crate::analytic_shell::tests::half_cylinder_input;
    use crate::analytic_shell::{
        AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellEdge,
        AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
        AnalyticShellPcurve, AnalyticShellSurface, AnalyticShellVertex, AnalyticVertexKey,
    };
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::{BodyId, FaceDomain, RegionKind};
    use crate::make::extrude_profile_along;
    use crate::profile::PlanarProfile;
    use crate::transaction::FullCommitRequirement;
    use kgeom::curve::{Circle, Line};
    use kgeom::curve2d::{Circle2d, Line2d};
    use kgeom::param::ParamRange;
    use kgeom::surface::Plane;
    use kgeom::vec::Vec2;
    use kgraph::AffineParamMap1d;

    fn parameter_map(scale: f64) -> AffineParamMap1d {
        AffineParamMap1d::new(scale, 0.0).unwrap()
    }

    fn oblique_frame() -> Frame {
        let origin = Point3::new(2.5, -1.75, 0.625);
        let axis = Vec3::new(0.48, 0.64, 0.6);
        let x = Vec3::new(0.8, -0.6, 0.0);
        Frame::new(origin, axis, x).unwrap()
    }

    fn solid_shell(store: &Store, body: BodyId) -> ShellId {
        let region = store
            .get(body)
            .unwrap()
            .regions
            .iter()
            .copied()
            .find(|&region| store.get(region).unwrap().kind == RegionKind::Solid)
            .unwrap();
        store.get(region).unwrap().shells[0]
    }

    fn concave_holed_profile(frame: Frame) -> PlanarProfile {
        let outer = [
            Point2::new(-3.0, -3.0),
            Point2::new(3.0, -3.0),
            Point2::new(3.0, 3.0),
            Point2::new(0.5, 3.0),
            Point2::new(0.5, 0.5),
            Point2::new(-3.0, 0.5),
        ];
        let first_hole = [
            Point2::new(-2.25, -2.25),
            Point2::new(-1.25, -2.25),
            Point2::new(-1.25, -1.25),
            Point2::new(-2.25, -1.25),
        ];
        let second_hole = [
            Point2::new(1.25, -1.75),
            Point2::new(2.25, -1.75),
            Point2::new(2.25, -0.75),
            Point2::new(1.25, -0.75),
        ];
        PlanarProfile::from_polygon_with_holes(frame, &outer, &[&first_hole, &second_hole]).unwrap()
    }

    fn plane_line_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        plane: Plane,
        line: Line,
    ) -> AnalyticShellFin {
        let origin = plane.frame().to_local(line.origin());
        let direction = line.dir();
        let local_direction = Vec2::new(
            direction.dot(plane.frame().x()),
            direction.dot(plane.frame().y()),
        );
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(origin.x, origin.y), local_direction).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn plane_circle_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
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

    fn cylinder_ruling_use(
        edge: AnalyticEdgeKey,
        sense: Sense,
        longitude: f64,
    ) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(longitude, 0.0), Vec2::new(0.0, 1.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    fn cylinder_arc_use(edge: AnalyticEdgeKey, sense: Sense, height: f64) -> AnalyticShellFin {
        AnalyticShellFin::new(
            edge,
            sense,
            AnalyticPcurveUse::new(
                AnalyticShellPcurve::Line(
                    Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
                ),
                parameter_map(1.0),
            ),
        )
    }

    /// A major circular segment is non-convex: its chord removes a strict
    /// circular cap. The translated frame also refuses world-axis shortcuts.
    fn concave_oblique_profile_input() -> AnalyticShellInput {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.6, 0.0, 0.8),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap();
        let height = 1.25;
        let translation = frame.z() * height;
        let top_frame = frame.with_origin(frame.origin() + translation);
        let cylinder = Cylinder::new(frame, 1.0).unwrap();
        let bottom_circle = Circle::new(frame, 1.0).unwrap();
        let top_circle = Circle::new(top_frame, 1.0).unwrap();
        let arc = ParamRange::new(0.25 * core::f64::consts::PI, 1.75 * core::f64::consts::PI);
        let points = [
            bottom_circle.eval(arc.lo),
            bottom_circle.eval(arc.hi),
            top_circle.eval(arc.lo),
            top_circle.eval(arc.hi),
        ];
        let vertices = points
            .into_iter()
            .enumerate()
            .map(|(index, point)| {
                AnalyticShellVertex::new(AnalyticVertexKey::new(index as u64), point)
            })
            .collect::<Vec<_>>();

        let chord_direction = points[1] - points[0];
        let chord_length = chord_direction.norm();
        let chord_line = Line::new(points[0], chord_direction).unwrap();
        let top_chord_line = Line::new(points[2], chord_line.dir()).unwrap();
        let first_ruling = Line::new(points[0], frame.z()).unwrap();
        let second_ruling = Line::new(points[1], frame.z()).unwrap();
        let edges = vec![
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(0),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
                AnalyticShellCurve::Circle(bottom_circle),
                arc,
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(1),
                [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Circle(top_circle),
                arc,
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(2),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(2)],
                AnalyticShellCurve::Line(first_ruling),
                ParamRange::new(0.0, height),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(3),
                [AnalyticVertexKey::new(1), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Line(second_ruling),
                ParamRange::new(0.0, height),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(4),
                [AnalyticVertexKey::new(0), AnalyticVertexKey::new(1)],
                AnalyticShellCurve::Line(chord_line),
                ParamRange::new(0.0, chord_length),
            ),
            AnalyticShellEdge::new(
                AnalyticEdgeKey::new(5),
                [AnalyticVertexKey::new(2), AnalyticVertexKey::new(3)],
                AnalyticShellCurve::Line(top_chord_line),
                ParamRange::new(0.0, chord_length),
            ),
        ];

        let bottom_plane = Plane::new(Frame::new(frame.origin(), -frame.z(), frame.x()).unwrap());
        let top_plane = Plane::new(top_frame);
        let cut_plane = Plane::new(Frame::new(points[0], frame.x(), chord_line.dir()).unwrap());
        let cylinder_loop = AnalyticShellLoop::new(vec![
            cylinder_ruling_use(AnalyticEdgeKey::new(2), Sense::Reversed, arc.lo),
            cylinder_arc_use(AnalyticEdgeKey::new(0), Sense::Forward, 0.0),
            cylinder_ruling_use(AnalyticEdgeKey::new(3), Sense::Forward, arc.hi),
            cylinder_arc_use(AnalyticEdgeKey::new(1), Sense::Reversed, height),
        ]);
        let bottom_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(0),
                Sense::Reversed,
                bottom_plane,
                bottom_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(4),
                Sense::Forward,
                bottom_plane,
                chord_line,
            ),
        ]);
        let top_loop = AnalyticShellLoop::new(vec![
            plane_circle_use(
                AnalyticEdgeKey::new(1),
                Sense::Forward,
                top_plane,
                top_circle,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(5),
                Sense::Reversed,
                top_plane,
                top_chord_line,
            ),
        ]);
        let cut_loop = AnalyticShellLoop::new(vec![
            plane_line_use(
                AnalyticEdgeKey::new(4),
                Sense::Reversed,
                cut_plane,
                chord_line,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(2),
                Sense::Forward,
                cut_plane,
                first_ruling,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(5),
                Sense::Forward,
                cut_plane,
                top_chord_line,
            ),
            plane_line_use(
                AnalyticEdgeKey::new(3),
                Sense::Reversed,
                cut_plane,
                second_ruling,
            ),
        ]);
        let wide_domain = || FaceDomain::from_bounds(-2.0, 2.0, -2.0, 2.0).unwrap();
        AnalyticShellInput::new(
            vertices,
            edges,
            vec![
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(0),
                    AnalyticShellSurface::Cylinder(cylinder),
                    Sense::Forward,
                    FaceDomain::from_bounds(arc.lo, arc.hi, 0.0, height).unwrap(),
                    vec![cylinder_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(1),
                    AnalyticShellSurface::Plane(bottom_plane),
                    Sense::Forward,
                    wide_domain(),
                    vec![bottom_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(2),
                    AnalyticShellSurface::Plane(top_plane),
                    Sense::Forward,
                    wide_domain(),
                    vec![top_loop],
                ),
                AnalyticShellFace::new(
                    AnalyticFaceKey::new(3),
                    AnalyticShellSurface::Plane(cut_plane),
                    Sense::Forward,
                    FaceDomain::from_bounds(-1.0, chord_length + 1.0, -height - 1.0, 1.0).unwrap(),
                    vec![cut_loop],
                ),
            ],
        )
    }

    #[test]
    fn half_cylinder_is_a_mixed_profile_prism() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&half_cylinder_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_mixed_profile_prism(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
    }

    #[test]
    fn concave_oblique_profile_is_full_certified() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        assert_eq!(
            certify_mixed_profile_prism(transaction.store(), output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Positive,
            })
        );
        assert!(matches!(
            check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        ));
        transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
    }

    #[test]
    fn concave_multi_loop_planar_profiles_use_the_general_sweep_theorem() {
        for translation in [Vec3::new(0.0, 0.0, 1.75), Vec3::new(0.375, -0.25, 1.75)] {
            let profile = concave_holed_profile(Frame::world());
            let mut store = Store::new();
            let body = extrude_profile_along(&mut store, &profile, translation).unwrap();
            let shell = solid_shell(&store, body);
            let cap_faces = &store.get(shell).unwrap().faces[..2];
            for &face in cap_faces {
                assert_eq!(
                    certify_loop_containment(&store, &store.get(face).unwrap().loops).unwrap(),
                    LoopContainment::Certified
                );
            }
            assert!(prepare_profile_cap(&store, cap_faces[0]).unwrap().is_some());
            assert!(prepare_profile_cap(&store, cap_faces[1]).unwrap().is_some());
            assert_eq!(
                store
                    .get(store.get(shell).unwrap().faces[0])
                    .unwrap()
                    .loops
                    .len(),
                3
            );
            assert_eq!(
                certify_mixed_profile_prism(&store, shell, None).unwrap(),
                Some(ShellCertification {
                    embedding: ShellEmbedding::Certified,
                    orientation: ShellOrientation::Positive,
                })
            );
            assert_eq!(
                check_body_report(&store, body, CheckLevel::Full)
                    .unwrap()
                    .outcome(),
                CheckOutcome::Valid
            );
        }
    }

    #[test]
    fn mixed_profile_tampering_fails_closed_and_live_senses_remain_decidable() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        let baseline = transaction.store().clone();
        let edge = |key: u64| {
            output
                .edges()
                .iter()
                .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
                .unwrap()
        };
        let face = |key: u64| {
            output
                .faces()
                .iter()
                .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                .unwrap()
        };
        let vertex = |key: u64| {
            output
                .vertices()
                .iter()
                .find_map(|(candidate, vertex)| (candidate.value() == key).then_some(*vertex))
                .unwrap()
        };

        for case in [
            "unsupported",
            "mapping",
            "simple",
            "radius",
            "axis",
            "range",
            "pcurve",
            "partial",
        ] {
            let mut copy = baseline.clone();
            let mut edit = copy.transaction().unwrap();
            match case {
                "unsupported" => {
                    let edge_id = edge(0);
                    let curve_id = edit.store().get(edge_id).unwrap().curve.unwrap();
                    let [Some(first), Some(second)] = edit.store().get(edge_id).unwrap().vertices
                    else {
                        unreachable!()
                    };
                    let start = edit.store().vertex_position(first).unwrap();
                    let end = edit.store().vertex_position(second).unwrap();
                    edit.store_mut()
                        .replace_curve(
                            curve_id,
                            CurveGeom::Line(Line::new(start, end - start).unwrap()),
                        )
                        .unwrap();
                }
                "mapping" => {
                    let point = edit.store().get(vertex(2)).unwrap().point;
                    edit.store_mut().get_mut(point).unwrap().y += 0.1;
                }
                "simple" => {
                    let loop_id = edit.store().get(face(1)).unwrap().loops[0];
                    let duplicate = edit.store().get(loop_id).unwrap().fins[0];
                    edit.store_mut()
                        .get_mut(loop_id)
                        .unwrap()
                        .fins
                        .push(duplicate);
                }
                "radius" | "axis" => {
                    let surface_id = edit.store().get(face(0)).unwrap().surface;
                    let SurfaceGeom::Cylinder(cylinder) = *edit.store().get(surface_id).unwrap()
                    else {
                        unreachable!()
                    };
                    let changed = if case == "radius" {
                        Cylinder::new(*cylinder.frame(), cylinder.radius() + 0.1).unwrap()
                    } else {
                        Cylinder::new(
                            Frame::new(
                                cylinder.frame().origin(),
                                cylinder.frame().z() + cylinder.frame().x() * 0.1,
                                cylinder.frame().x(),
                            )
                            .unwrap(),
                            cylinder.radius(),
                        )
                        .unwrap()
                    };
                    edit.store_mut()
                        .replace_surface(surface_id, SurfaceGeom::Cylinder(changed))
                        .unwrap();
                }
                "range" => {
                    let edge = edit.store_mut().get_mut(edge(1)).unwrap();
                    edge.bounds = edge.bounds.map(|(lo, hi)| (lo, hi - 0.1));
                }
                "pcurve" => {
                    let loop_id = edit.store().get(face(1)).unwrap().loops[0];
                    let fin = edit.store().get(loop_id).unwrap().fins[0];
                    edit.store_mut().get_mut(fin).unwrap().pcurve = None;
                }
                "partial" => {
                    let loop_id = edit.store().get(face(0)).unwrap().loops[0];
                    edit.store_mut().get_mut(loop_id).unwrap().fins.pop();
                }
                _ => unreachable!(),
            }
            assert_eq!(
                certify_mixed_profile_prism(edit.store(), output.shell(), None).unwrap(),
                None,
                "{case} tamper must not retain the theorem"
            );
        }

        let mut wrong = baseline.clone();
        wrong.get_mut(face(0)).unwrap().sense = Sense::Reversed;
        assert_eq!(
            certify_mixed_profile_prism(&wrong, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            })
        );
    }

    fn session_with_work(allowed: u64) -> kcore::operation::SessionPolicy {
        let budget = BudgetPlan::new([LimitSpec::new(
            MIXED_PROFILE_PRISM_WORK,
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
    fn mixed_profile_work_accepts_exact_n_and_rejects_n_minus_one() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&concave_oblique_profile_input(), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell(), 3)
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
                certify_mixed_profile_prism(transaction.store(), output.shell(), Some(&mut scope));
            if allowed == required {
                assert_eq!(
                    result.unwrap().unwrap().embedding,
                    ShellEmbedding::Certified
                );
            } else {
                assert_eq!(
                    result.unwrap_err().limit().map(|limit| limit.stage),
                    Some(MIXED_PROFILE_PRISM_WORK)
                );
            }
        }
    }

    #[test]
    fn strict_parallel_cylinder_profile_uses_the_general_sweep_theorem_across_authored_frames() {
        for frame in [Frame::world(), oblique_frame()] {
            for second_axis_reversed in [false, true] {
                for permuted in [false, true] {
                    let mut store = Store::new();
                    let mut transaction = store.transaction().unwrap();
                    let output = transaction
                        .assemble_analytic_shell(
                            &source_ring_parallel_cylinder_lens_input(
                                frame,
                                second_axis_reversed,
                                permuted,
                            ),
                            1.0e-12,
                        )
                        .unwrap();
                    assert_eq!(output.faces().len(), 4);
                    assert_eq!(output.edges().len(), 6);
                    assert_eq!(output.vertices().len(), 4);
                    let axes = [0, 1].map(|key| {
                        let face = output
                            .faces()
                            .iter()
                            .find_map(|(candidate, face)| {
                                (candidate.value() == key).then_some(*face)
                            })
                            .unwrap();
                        let face = transaction.store().get(face).unwrap();
                        let SurfaceGeom::Cylinder(cylinder) =
                            transaction.store().get(face.surface).unwrap()
                        else {
                            panic!("source-ring side must retain its cylinder support")
                        };
                        cylinder.frame().z()
                    });
                    assert_eq!(axes[0].dot(axes[1]) < 0.0, second_axis_reversed);
                    assert_eq!(
                        certify_mixed_profile_prism(transaction.store(), output.shell(), None,)
                            .unwrap(),
                        Some(ShellCertification {
                            embedding: ShellEmbedding::Certified,
                            orientation: ShellOrientation::Positive,
                        })
                    );
                    let decision = transaction
                        .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                        .unwrap();
                    assert_eq!(decision.checks()[0].report().outcome(), CheckOutcome::Valid);
                }
            }
        }
    }

    #[test]
    fn strict_parallel_cylinder_profile_tampering_fails_closed() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(
                &source_ring_parallel_cylinder_lens_input(oblique_frame(), true, true),
                1.0e-12,
            )
            .unwrap();
        let baseline = transaction.store().clone();
        let edge = |key: u64| {
            output
                .edges()
                .iter()
                .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
                .unwrap()
        };
        let face = |key: u64| {
            output
                .faces()
                .iter()
                .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
                .unwrap()
        };

        for case in ["carrier", "pcurve", "source", "sense"] {
            let mut copy = baseline.clone();
            let mut edit = copy.transaction().unwrap();
            match case {
                "carrier" => {
                    let curve_id = edit.store().get(edge(2)).unwrap().curve.unwrap();
                    let CurveGeom::Circle(circle) = *edit.store().curve(curve_id).unwrap() else {
                        unreachable!()
                    };
                    edit.store_mut()
                        .replace_curve(
                            curve_id,
                            CurveGeom::Circle(
                                Circle::new(*circle.frame(), circle.radius() + 0.125).unwrap(),
                            ),
                        )
                        .unwrap();
                }
                "pcurve" => {
                    let loop_id = edit.store().get(face(2)).unwrap().loops[0];
                    let fin_id = edit.store().get(loop_id).unwrap().fins[0];
                    let pcurve_id = edit.store().get(fin_id).unwrap().pcurve.unwrap().curve();
                    edit.store_mut()
                        .replace_pcurve(
                            pcurve_id,
                            Curve2dGeom::Circle(
                                Circle2d::new(Point2::new(1.125, 0.0), 1.0, Vec2::new(1.0, 0.0))
                                    .unwrap(),
                            ),
                        )
                        .unwrap();
                }
                "source" => {
                    let surface_id = edit.store().get(face(1)).unwrap().surface;
                    let SurfaceGeom::Cylinder(cylinder) =
                        *edit.store().surface(surface_id).unwrap()
                    else {
                        unreachable!()
                    };
                    edit.store_mut()
                        .replace_surface(
                            surface_id,
                            SurfaceGeom::Cylinder(
                                Cylinder::new(*cylinder.frame(), cylinder.radius() + 0.125)
                                    .unwrap(),
                            ),
                        )
                        .unwrap();
                }
                "sense" => {
                    let loop_id = edit.store().get(face(2)).unwrap().loops[0];
                    let fin_id = edit.store().get(loop_id).unwrap().fins[0];
                    let fin = edit.store_mut().get_mut(fin_id).unwrap();
                    fin.sense = fin.sense.flipped();
                }
                _ => unreachable!(),
            }
            assert_eq!(
                certify_mixed_profile_prism(edit.store(), output.shell(), None).unwrap(),
                None,
                "{case} tamper must not retain the sweep theorem"
            );
        }

        let mut wrong_orientation = baseline;
        wrong_orientation.get_mut(face(0)).unwrap().sense = Sense::Reversed;
        assert_eq!(
            certify_mixed_profile_prism(&wrong_orientation, output.shell(), None).unwrap(),
            Some(ShellCertification {
                embedding: ShellEmbedding::Certified,
                orientation: ShellOrientation::Invalid,
            }),
        );
    }

    #[test]
    fn strict_parallel_cylinder_profile_work_is_exactly_budgeted() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_analytic_shell(&unsplit_parallel_cylinder_lens_input(), 1.0e-12)
            .unwrap();
        let required = proof_work(transaction.store(), output.shell(), 2)
            .unwrap()
            .unwrap();
        assert_eq!(required, 1_457);
        for allowed in [required, required - 1] {
            let session = session_with_work(allowed);
            let context = kcore::operation::OperationContext::new(
                &session,
                kcore::tolerance::Tolerances::default(),
            )
            .unwrap();
            let mut scope = OperationScope::new(&context);
            let result =
                certify_mixed_profile_prism(transaction.store(), output.shell(), Some(&mut scope));
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
                    Some(MIXED_PROFILE_PRISM_WORK)
                );
            }
        }
    }
}
