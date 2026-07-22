//! Section publication for graph-certified Cylinder/Cylinder rulings.
//!
//! This module only adapts graph-owned carrier and pcurve evidence. Bounded
//! face membership, source-root identity, proof-range expansion, and fragment
//! stitching remain owned by the existing ruling publication pipeline.
//!
//! Orientation uses metric projections against the stored cylinder axes. A
//! [`Frame`] axis is semantically unit length but its stored components are
//! rounded, so the radial theorem retains the outward `axis · axis`
//! denominator instead of silently replacing it by one. Carrier and normal
//! magnitudes remain irrelevant: only a strict interval sign is published.

use super::*;

enum CylinderCylinderBranchAdaptation {
    Adapted(Box<SectionBranch>),
    OrientationIndeterminate,
    Unsupported,
}

/// Adapt and topology-clip one graph-certified Cylinder/Cylinder ruling.
#[allow(clippy::too_many_arguments)]
pub(super) fn append_branch(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    root_identity: &mut root_identity::RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    let branch = match adapt_branch(facades, edge, vertices, surfaces, senses) {
        CylinderCylinderBranchAdaptation::Adapted(branch) => *branch,
        CylinderCylinderBranchAdaptation::OrientationIndeterminate => {
            acc.gaps.push(SectionGap {
                reason: GAP_CARRIER_ORIENTATION,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
        CylinderCylinderBranchAdaptation::Unsupported => {
            acc.gaps.push(SectionGap {
                reason: GAP_PAIR_UNRESOLVED,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
    };
    ruling_publish::append_branch(store, raw_faces, facades, branch, root_identity, scope, acc)
}

fn adapt_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
) -> CylinderCylinderBranchAdaptation {
    if edge.kind != ContactKind::Transverse {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(certificate) = edge.certificate.as_cylinder_cylinder_ruling() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let CurveDescriptor::Line(carrier) = edge.carrier else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    if edge.topology != IntersectionBranchTopology::Open
        || edge.endpoint_vertices[0] == edge.endpoint_vertices[1]
    {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(flipped) = canonical_flip(
        surfaces[0],
        senses[0],
        surfaces[1],
        senses[1],
        carrier.origin(),
        carrier.dir(),
    ) else {
        return CylinderCylinderBranchAdaptation::OrientationIndeterminate;
    };
    let Some(pcurves) = branch_publish::adapt_branch_pcurves(edge, flipped) else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    if !matches!(pcurves, [SectionUvCurve::Line(_), SectionUvCurve::Line(_)]) {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(low_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(flipped)])
        .copied()
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let Some(high_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(!flipped)])
        .copied()
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: low_surfaces,
    } = low_vertex.event
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: high_surfaces,
    } = high_vertex.event
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let range = if flipped {
        ParamRange {
            lo: -edge.carrier_range.hi,
            hi: -edge.carrier_range.lo,
        }
    } else {
        edge.carrier_range
    };
    CylinderCylinderBranchAdaptation::Adapted(Box::new(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::Line {
            origin: carrier.origin(),
            direction: if flipped {
                -carrier.dir()
            } else {
                carrier.dir()
            },
        },
        range,
        topology: SectionBranchTopology::Open,
        pcurves,
        fragment_sites: vec![
            SectionFragmentSite {
                point: low_vertex.point,
                surface_parameters: low_vertex.surface_parameters,
                surface_window_boundaries: low_surfaces,
            },
            SectionFragmentSite {
                point: high_vertex.point,
                surface_parameters: high_vertex.surface_parameters,
                surface_window_boundaries: high_surfaces,
            },
        ],
        endpoint_sites: [0, 1],
        evidence: SectionBranchEvidence {
            residual_bounds: certificate.residual_bounds(),
            tolerance: certificate.tolerance(),
        },
        ruling_recertification: Some(RulingRecertification::CylinderCylinderGraph(certificate)),
        ruling_parameter_flipped: flipped,
    }))
}

/// Decide whether the graph-canonical ruling must be reversed to follow
/// Section's `n_A x n_B` convention.
///
/// Both cylinder normals are evaluated as outward-rounded, unnormalized
/// radial vectors at the carrier origin. A strict sign is required; tangent
/// or arithmetically ambiguous candidates remain structured gaps.
fn canonical_flip(
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
    origin: Point3,
    direction: Vec3,
) -> Option<bool> {
    let (SurfaceGeom::Cylinder(cylinder_a), SurfaceGeom::Cylinder(cylinder_b)) =
        (surface_a, surface_b)
    else {
        return None;
    };
    let normal_a = cylinder_normal(cylinder_a, sense_a, origin)?;
    let normal_b = cylinder_normal(cylinder_b, sense_b, origin)?;
    certified_carrier_sign_intervals(
        direction.to_array().map(Interval::point),
        normal_a,
        normal_b,
    )
    .map(|positive| !positive)
}

fn cylinder_normal(
    cylinder: &kgeom::surface::Cylinder,
    sense: Sense,
    point: Point3,
) -> Option<[Interval; 3]> {
    let axis = cylinder.frame().z().to_array().map(Interval::point);
    let axis_origin = cylinder.frame().origin().to_array();
    let point = point.to_array();
    let offset: [Interval; 3] = core::array::from_fn(|index| {
        Interval::point(point[index]) - Interval::point(axis_origin[index])
    });
    metric_radial(offset, axis, sense)
}

/// Outward metric projection onto the plane perpendicular to `axis`.
///
/// The division is essential for a general stored vector. In the certified
/// parallel-ruling family, omitting it adds only an axis-parallel component
/// whose exact contribution to `direction · (n_a × n_b)` vanishes. Retaining
/// it here removes that family-specific dependency and avoids needlessly wide
/// normal enclosures for rounded nonunit frame axes.
fn metric_radial(
    offset: [Interval; 3],
    axis: [Interval; 3],
    sense: Sense,
) -> Option<[Interval; 3]> {
    let axial_numerator = interval_dot(offset, axis);
    let axis_squared = interval_dot(axis, axis);
    let axial = axial_numerator.checked_div(axis_squared)?;
    if !finite(axial) {
        return None;
    }
    let sign = Interval::point(if sense.is_forward() { 1.0 } else { -1.0 });
    let radial = core::array::from_fn(|index| sign * (offset[index] - axis[index] * axial));
    radial.into_iter().all(finite).then_some(radial)
}

fn interval_dot(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, index| sum + a[index] * b[index])
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;

    use super::*;

    fn point_intervals(values: [f64; 3]) -> [Interval; 3] {
        values.map(Interval::point)
    }

    #[test]
    fn metric_radial_divides_by_the_outward_axis_gram_term() {
        let radial = metric_radial(
            point_intervals([3.0, 4.0, 5.0]),
            point_intervals([2.0, 0.0, 0.0]),
            Sense::Forward,
        )
        .unwrap();
        assert!(radial[0].contains(0.0));
        assert!(radial[1].contains(4.0));
        assert!(radial[2].contains(5.0));
        assert!(interval_dot(radial, point_intervals([2.0, 0.0, 0.0])).contains_zero());
        assert!(
            metric_radial(
                point_intervals([1.0, 2.0, 3.0]),
                point_intervals([0.0, 0.0, 0.0]),
                Sense::Forward,
            )
            .is_none()
        );
    }

    #[test]
    fn canonical_orientation_reverses_under_operand_swap_and_one_face_reversal() {
        let first = SurfaceGeom::Cylinder(Cylinder::new(Frame::world(), 1.0).unwrap());
        let second = SurfaceGeom::Cylinder(
            Cylinder::new(Frame::world().with_origin(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap(),
        );
        let origin = Point3::new(0.5, 3.0_f64.sqrt() * 0.5, 0.0);
        let direction = Vec3::new(0.0, 0.0, 1.0);
        let forward = canonical_flip(
            &first,
            Sense::Forward,
            &second,
            Sense::Forward,
            origin,
            direction,
        )
        .unwrap();
        assert_eq!(
            canonical_flip(
                &second,
                Sense::Forward,
                &first,
                Sense::Forward,
                origin,
                direction,
            ),
            Some(!forward)
        );
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Reversed,
                &second,
                Sense::Forward,
                origin,
                direction,
            ),
            Some(!forward)
        );
    }

    #[test]
    fn rounded_oblique_axes_preserve_orientation_under_axial_translation_and_swap() {
        let frame = Frame::new(
            Point3::new(2.0, -3.0, 5.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, 0.0),
        )
        .unwrap();
        assert_ne!(
            frame.z().dot(frame.z()),
            1.0,
            "fixture must exercise a rounded nonunit stored axis"
        );
        let second_frame = frame.with_origin(frame.point_at(1.0, 0.0, 0.0));
        let first = SurfaceGeom::Cylinder(Cylinder::new(frame, 1.0).unwrap());
        let second = SurfaceGeom::Cylinder(Cylinder::new(second_frame, 1.0).unwrap());
        let local_y = 3.0_f64.sqrt() * 0.5;
        let near = frame.point_at(0.5, local_y, 0.0);
        let translated = frame.point_at(0.5, local_y, 1.0e6);
        let direction = frame.z();

        let expected = canonical_flip(
            &first,
            Sense::Forward,
            &second,
            Sense::Forward,
            near,
            direction,
        )
        .unwrap();
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Forward,
                &second,
                Sense::Forward,
                translated,
                direction,
            ),
            Some(expected)
        );
        assert_eq!(
            canonical_flip(
                &second,
                Sense::Forward,
                &first,
                Sense::Forward,
                translated,
                direction,
            ),
            Some(!expected)
        );
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Forward,
                &second,
                Sense::Forward,
                translated,
                direction * 7.0,
            ),
            Some(expected)
        );
    }
}
