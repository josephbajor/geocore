//! Symbolic plane BSP for convex planar face fragments.
//!
//! Vertices are intersections of three stable source planes. Splitting never
//! constructs a floating intersection point: exact four-plane classification
//! decides sides, and a crossed edge acquires the canonical triple consisting
//! of its face plane, its edge plane, and the cutter. This gives independent
//! operand-face splits the same combinatorial keys at a Boolean seam.

use kcore::predicates::{
    Orientation, OrientedPlanePoints, oriented_plane_triple_intersection_side,
};

/// Stable plane identity within an ordered Boolean operand pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct SourcePlaneRef {
    operand: u8,
    face: u32,
}

impl SourcePlaneRef {
    pub(crate) const fn new(operand: u8, face: u32) -> Self {
        Self { operand, face }
    }
}

/// One exact oriented source-plane witness plus its material half-space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SourcePlane {
    id: SourcePlaneRef,
    points: OrientedPlanePoints,
    interior_side: Orientation,
}

impl SourcePlane {
    /// Admit a nondegenerate witness whose interior sample is strictly off
    /// the plane. The sample fixes which `orient3d` side contains material.
    pub(crate) fn from_interior_sample(
        id: SourcePlaneRef,
        points: OrientedPlanePoints,
        interior_sample: [f64; 3],
    ) -> Option<Self> {
        if points
            .iter()
            .flatten()
            .chain(interior_sample.iter())
            .any(|coordinate| !coordinate.is_finite())
        {
            return None;
        }
        let interior_side =
            kcore::predicates::orient3d(points[0], points[1], points[2], interior_sample);
        (interior_side != Orientation::Zero).then_some(Self {
            id,
            points,
            interior_side,
        })
    }
}

/// Canonical symbolic identity of one simple three-plane vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct PlaneTripleVertexKey {
    planes: [SourcePlaneRef; 3],
}

impl PlaneTripleVertexKey {
    pub(crate) fn new(mut planes: [SourcePlaneRef; 3]) -> Option<Self> {
        planes.sort_unstable();
        (planes[0] != planes[1] && planes[1] != planes[2]).then_some(Self { planes })
    }

    fn contains(self, plane: SourcePlaneRef) -> bool {
        self.planes.contains(&plane)
    }
}

/// Certified open-half-space relation retained in a fragment sign vector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum HalfspaceSide {
    Inward,
    Outward,
}

/// One convex polygon carried by a source face.
///
/// `edge_planes[i]` supports the directed edge from `vertices[i]` to the
/// cyclic successor. `signs` records exactly one strict relation for every
/// cutter already applied, in cutter order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ConvexPlanarFragment {
    source_face: SourcePlaneRef,
    support: SourcePlaneRef,
    vertices: Vec<PlaneTripleVertexKey>,
    edge_planes: Vec<SourcePlaneRef>,
    signs: Vec<(SourcePlaneRef, HalfspaceSide)>,
}

impl ConvexPlanarFragment {
    pub(crate) fn new(
        source_face: SourcePlaneRef,
        vertices: Vec<PlaneTripleVertexKey>,
        edge_planes: Vec<SourcePlaneRef>,
    ) -> Result<Self, FragmentError> {
        if vertices.len() < 3 || vertices.len() != edge_planes.len() {
            return Err(FragmentError::InvalidRing);
        }
        for index in 0..vertices.len() {
            let edge_plane = edge_planes[index];
            if edge_plane == source_face
                || !vertices[index].contains(source_face)
                || !vertices[(index + 1) % vertices.len()].contains(source_face)
                || !vertices[index].contains(edge_plane)
                || !vertices[(index + 1) % vertices.len()].contains(edge_plane)
            {
                return Err(FragmentError::InvalidRing);
            }
        }
        Ok(Self {
            source_face,
            support: source_face,
            vertices,
            edge_planes,
            signs: Vec::new(),
        })
    }

    pub(crate) fn source_face(&self) -> SourcePlaneRef {
        self.source_face
    }

    pub(crate) fn vertices(&self) -> &[PlaneTripleVertexKey] {
        &self.vertices
    }
}

/// Complete relation of a positive-area fragment to a convex solid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FragmentClassification {
    Interior,
    Exterior,
}

/// Honest refusal from the bounded symbolic fragment stage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FragmentError {
    UnknownPlane,
    DuplicateCutter,
    InvalidRing,
    BoundaryContact,
    UncertifiedPredicate,
    MissingClassification,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum FragmentSplit {
    Unsplit(ConvexPlanarFragment),
    Split {
        inward: ConvexPlanarFragment,
        outward: ConvexPlanarFragment,
    },
}

fn plane(planes: &[SourcePlane], id: SourcePlaneRef) -> Option<&SourcePlane> {
    planes.iter().find(|plane| plane.id == id)
}

fn side_of_vertex(
    planes: &[SourcePlane],
    vertex: PlaneTripleVertexKey,
    cutter: &SourcePlane,
) -> Result<HalfspaceSide, FragmentError> {
    let [first, second, third] = vertex.planes;
    let witnesses = [
        plane(planes, first)
            .ok_or(FragmentError::UnknownPlane)?
            .points,
        plane(planes, second)
            .ok_or(FragmentError::UnknownPlane)?
            .points,
        plane(planes, third)
            .ok_or(FragmentError::UnknownPlane)?
            .points,
    ];
    let side = oriented_plane_triple_intersection_side(witnesses, cutter.points)
        .ok_or(FragmentError::UncertifiedPredicate)?
        .sign();
    if side == Orientation::Zero {
        return Err(FragmentError::BoundaryContact);
    }
    Ok(if side == cutter.interior_side {
        HalfspaceSide::Inward
    } else {
        HalfspaceSide::Outward
    })
}

fn append_sign(
    fragment: &ConvexPlanarFragment,
    cutter: SourcePlaneRef,
    side: HalfspaceSide,
) -> Result<Vec<(SourcePlaneRef, HalfspaceSide)>, FragmentError> {
    if fragment.signs.iter().any(|(plane, _)| *plane == cutter) {
        return Err(FragmentError::DuplicateCutter);
    }
    let mut signs = fragment.signs.clone();
    signs.push((cutter, side));
    Ok(signs)
}

/// Sutherland-Hodgman output entry: the plane supporting the edge arriving
/// at `vertex` is retained so the cyclic edge labels can be reconstructed.
#[derive(Debug, Clone, Copy)]
struct IncomingVertex {
    vertex: PlaneTripleVertexKey,
    incoming: SourcePlaneRef,
}

fn clipped_half(
    fragment: &ConvexPlanarFragment,
    cutter: SourcePlaneRef,
    sides: &[HalfspaceSide],
    keep: HalfspaceSide,
) -> Result<ConvexPlanarFragment, FragmentError> {
    let mut output = Vec::new();
    let count = fragment.vertices.len();
    for index in 0..count {
        let next = (index + 1) % count;
        let start_kept = sides[index] == keep;
        let end_kept = sides[next] == keep;
        let edge_plane = fragment.edge_planes[index];
        match (start_kept, end_kept) {
            (true, true) => output.push(IncomingVertex {
                vertex: fragment.vertices[next],
                incoming: edge_plane,
            }),
            (true, false) => output.push(IncomingVertex {
                vertex: PlaneTripleVertexKey::new([fragment.support, edge_plane, cutter])
                    .ok_or(FragmentError::BoundaryContact)?,
                incoming: edge_plane,
            }),
            (false, true) => {
                output.push(IncomingVertex {
                    vertex: PlaneTripleVertexKey::new([fragment.support, edge_plane, cutter])
                        .ok_or(FragmentError::BoundaryContact)?,
                    incoming: cutter,
                });
                output.push(IncomingVertex {
                    vertex: fragment.vertices[next],
                    incoming: edge_plane,
                });
            }
            (false, false) => {}
        }
    }
    if output.len() < 3
        || output
            .iter()
            .zip(output.iter().cycle().skip(1))
            .any(|(first, second)| first.vertex == second.vertex)
    {
        return Err(FragmentError::InvalidRing);
    }
    let vertices = output.iter().map(|entry| entry.vertex).collect::<Vec<_>>();
    let edge_planes = (0..output.len())
        .map(|index| output[(index + 1) % output.len()].incoming)
        .collect();
    Ok(ConvexPlanarFragment {
        source_face: fragment.source_face,
        support: fragment.support,
        vertices,
        edge_planes,
        signs: append_sign(fragment, cutter, keep)?,
    })
}

/// Split one convex fragment by one source plane.
///
/// An exact vertex contact is refused rather than silently assigned to both
/// sides. The initial Boolean slice therefore admits general-position
/// transverse arrangements and leaves tangent/coincident policy explicit.
pub(crate) fn split_fragment(
    planes: &[SourcePlane],
    fragment: &ConvexPlanarFragment,
    cutter_id: SourcePlaneRef,
) -> Result<FragmentSplit, FragmentError> {
    if cutter_id == fragment.support || fragment.signs.iter().any(|(plane, _)| *plane == cutter_id)
    {
        return Err(FragmentError::DuplicateCutter);
    }
    let cutter = plane(planes, cutter_id).ok_or(FragmentError::UnknownPlane)?;
    let sides = fragment
        .vertices
        .iter()
        .map(|&vertex| side_of_vertex(planes, vertex, cutter))
        .collect::<Result<Vec<_>, _>>()?;
    let first = sides[0];
    if sides.iter().all(|&side| side == first) {
        let mut result = fragment.clone();
        result.signs = append_sign(fragment, cutter_id, first)?;
        return Ok(FragmentSplit::Unsplit(result));
    }
    let transitions = sides
        .iter()
        .zip(sides.iter().cycle().skip(1))
        .filter(|(first, second)| first != second)
        .count();
    if transitions != 2 {
        return Err(FragmentError::InvalidRing);
    }
    Ok(FragmentSplit::Split {
        inward: clipped_half(fragment, cutter_id, &sides, HalfspaceSide::Inward)?,
        outward: clipped_half(fragment, cutter_id, &sides, HalfspaceSide::Outward)?,
    })
}

/// Partition seeds by every cutter in deterministic caller order.
pub(crate) fn partition_fragments(
    planes: &[SourcePlane],
    seeds: Vec<ConvexPlanarFragment>,
    cutters: &[SourcePlaneRef],
) -> Result<Vec<ConvexPlanarFragment>, FragmentError> {
    let mut fragments = seeds;
    for &cutter in cutters {
        let mut next = Vec::new();
        for fragment in &fragments {
            match split_fragment(planes, fragment, cutter)? {
                FragmentSplit::Unsplit(fragment) => next.push(fragment),
                FragmentSplit::Split { inward, outward } => {
                    next.push(inward);
                    next.push(outward);
                }
            }
        }
        fragments = next;
    }
    Ok(fragments)
}

/// Classify one completely partitioned fragment against an ordered convex
/// solid plane set.
pub(crate) fn classify_fragment(
    fragment: &ConvexPlanarFragment,
    solid_planes: &[SourcePlaneRef],
) -> Result<FragmentClassification, FragmentError> {
    let mut all_inward = true;
    for &required in solid_planes {
        let Some((_, side)) = fragment.signs.iter().find(|(plane, _)| *plane == required) else {
            return Err(FragmentError::MissingClassification);
        };
        all_inward &= *side == HalfspaceSide::Inward;
    }
    Ok(if all_inward {
        FragmentClassification::Interior
    } else {
        FragmentClassification::Exterior
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const FACE_CORNERS: [[usize; 4]; 6] = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];

    struct BoxFixture {
        planes: Vec<SourcePlane>,
        plane_ids: Vec<SourcePlaneRef>,
        faces: Vec<ConvexPlanarFragment>,
    }

    fn transform(point: [f64; 3], center: [f64; 3], matrix: [[f64; 3]; 3]) -> [f64; 3] {
        core::array::from_fn(|row| {
            center[row]
                + matrix[row][0] * point[0]
                + matrix[row][1] * point[1]
                + matrix[row][2] * point[2]
        })
    }

    fn box_fixture(
        operand: u8,
        center: [f64; 3],
        half: [f64; 3],
        matrix: [[f64; 3]; 3],
    ) -> BoxFixture {
        let corners: Vec<[f64; 3]> = (0..8_u8)
            .map(|index| {
                let local = [
                    if index & 1 == 0 { -half[0] } else { half[0] },
                    if index & 2 == 0 { -half[1] } else { half[1] },
                    if index & 4 == 0 { -half[2] } else { half[2] },
                ];
                transform(local, center, matrix)
            })
            .collect();
        let plane_ids = (0..6)
            .map(|face| SourcePlaneRef::new(operand, face))
            .collect::<Vec<_>>();
        let planes = FACE_CORNERS
            .iter()
            .enumerate()
            .map(|(face, ring)| {
                SourcePlane::from_interior_sample(
                    plane_ids[face],
                    [corners[ring[0]], corners[ring[1]], corners[ring[2]]],
                    center,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        let corner_planes: Vec<[SourcePlaneRef; 3]> = (0..8_u8)
            .map(|index| {
                [
                    plane_ids[if index & 1 == 0 { 4 } else { 5 }],
                    plane_ids[if index & 2 == 0 { 2 } else { 3 }],
                    plane_ids[if index & 4 == 0 { 0 } else { 1 }],
                ]
            })
            .collect();
        let faces = FACE_CORNERS
            .iter()
            .enumerate()
            .map(|(face, ring)| {
                let vertices = ring
                    .iter()
                    .map(|&corner| PlaneTripleVertexKey::new(corner_planes[corner]).unwrap())
                    .collect::<Vec<_>>();
                let edge_planes = (0..4)
                    .map(|index| {
                        let first = corner_planes[ring[index]];
                        let second = corner_planes[ring[(index + 1) % 4]];
                        first
                            .into_iter()
                            .find(|plane| *plane != plane_ids[face] && second.contains(plane))
                            .unwrap()
                    })
                    .collect::<Vec<_>>();
                ConvexPlanarFragment::new(plane_ids[face], vertices, edge_planes).unwrap()
            })
            .collect();
        BoxFixture {
            planes,
            plane_ids,
            faces,
        }
    }

    fn identity() -> [[f64; 3]; 3] {
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    }

    fn rotation_z(angle: f64) -> [[f64; 3]; 3] {
        let (sin, cos) = kcore::math::sincos(angle);
        [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]]
    }

    fn matrix_product(left: [[f64; 3]; 3], right: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
        core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                (0..3)
                    .map(|inner| left[row][inner] * right[inner][column])
                    .sum()
            })
        })
    }

    fn rotation_xyz(x: f64, y: f64, z: f64) -> [[f64; 3]; 3] {
        let (sin_x, cos_x) = kcore::math::sincos(x);
        let (sin_y, cos_y) = kcore::math::sincos(y);
        let x_rotation = [[1.0, 0.0, 0.0], [0.0, cos_x, -sin_x], [0.0, sin_x, cos_x]];
        let y_rotation = [[cos_y, 0.0, sin_y], [0.0, 1.0, 0.0], [-sin_y, 0.0, cos_y]];
        matrix_product(rotation_z(z), matrix_product(y_rotation, x_rotation))
    }

    fn partition_pair(
        source: &BoxFixture,
        cutter: &BoxFixture,
    ) -> Result<Vec<ConvexPlanarFragment>, FragmentError> {
        let mut planes = source.planes.clone();
        planes.extend_from_slice(&cutter.planes);
        partition_fragments(&planes, source.faces.clone(), &cutter.plane_ids)
    }

    fn classifications(
        source: &BoxFixture,
        cutter: &BoxFixture,
    ) -> Result<Vec<FragmentClassification>, FragmentError> {
        partition_pair(source, cutter)?
            .iter()
            .map(|fragment| classify_fragment(fragment, &cutter.plane_ids))
            .collect()
    }

    #[test]
    fn containment_and_disjoint_are_classified_without_layout_cases() {
        let inner = box_fixture(0, [4.0, -3.0, 2.0], [0.5; 3], identity());
        let outer = box_fixture(1, [4.0, -3.0, 2.0], [2.0; 3], identity());
        assert!(
            classifications(&inner, &outer)
                .unwrap()
                .iter()
                .all(|class| *class == FragmentClassification::Interior)
        );

        let far = box_fixture(1, [14.0, 1.0, 6.0], [0.5; 3], identity());
        assert!(
            classifications(&inner, &far)
                .unwrap()
                .iter()
                .all(|class| *class == FragmentClassification::Exterior)
        );
    }

    #[test]
    fn rotated_off_origin_overlap_yields_both_fragment_classes() {
        let first = box_fixture(0, [7.0, -5.0, 3.0], [1.5, 1.0, 1.25], identity());
        let second = box_fixture(1, [7.6, -4.7, 3.2], [1.3, 0.9, 1.1], rotation_z(0.47));
        for classes in [
            classifications(&first, &second).unwrap(),
            classifications(&second, &first).unwrap(),
        ] {
            assert!(classes.contains(&FragmentClassification::Interior));
            assert!(classes.contains(&FragmentClassification::Exterior));
        }
    }

    fn swap_operand_label(id: SourcePlaneRef) -> SourcePlaneRef {
        SourcePlaneRef::new(1 - id.operand, id.face)
    }

    fn swap_fragment_operand_labels(mut fragment: ConvexPlanarFragment) -> ConvexPlanarFragment {
        fragment.source_face = swap_operand_label(fragment.source_face);
        fragment.support = swap_operand_label(fragment.support);
        for vertex in &mut fragment.vertices {
            *vertex = PlaneTripleVertexKey::new(vertex.planes.map(swap_operand_label)).unwrap();
        }
        fragment.edge_planes = fragment
            .edge_planes
            .into_iter()
            .map(swap_operand_label)
            .collect();
        fragment.signs = fragment
            .signs
            .into_iter()
            .map(|(plane, side)| (swap_operand_label(plane), side))
            .collect();
        fragment
    }

    #[test]
    fn partition_is_deterministic_and_invariant_to_operand_label_swap() {
        let first = box_fixture(0, [2.0, 3.0, -4.0], [1.0; 3], identity());
        let second = box_fixture(
            1,
            [2.0, 3.0, -4.0],
            [1.0; 3],
            rotation_xyz(0.21, 0.29, 0.37),
        );
        let first_run = partition_pair(&first, &second).unwrap();
        let second_run = partition_pair(&first, &second).unwrap();
        assert_eq!(first_run, second_run);

        let relabeled_first = box_fixture(1, [2.0, 3.0, -4.0], [1.0; 3], identity());
        let relabeled_second = box_fixture(
            0,
            [2.0, 3.0, -4.0],
            [1.0; 3],
            rotation_xyz(0.21, 0.29, 0.37),
        );
        let relabeled = partition_pair(&relabeled_first, &relabeled_second)
            .unwrap()
            .into_iter()
            .map(swap_fragment_operand_labels)
            .collect::<Vec<_>>();
        assert_eq!(first_run, relabeled);
    }

    #[test]
    fn exact_boundary_contact_is_refused() {
        let source = box_fixture(0, [0.0; 3], [1.0; 3], identity());
        // The cutter's x=1 face passes through four source vertices. It is a
        // coincident/boundary arrangement, not an open-half-space split.
        let touching = box_fixture(1, [2.0, 0.0, 0.0], [1.0; 3], identity());
        assert!(matches!(
            partition_pair(&source, &touching),
            Err(FragmentError::BoundaryContact)
        ));
    }

    #[test]
    fn malformed_symbolic_rings_and_incomplete_sign_vectors_are_rejected() {
        let fixture = box_fixture(0, [0.0; 3], [1.0; 3], identity());
        let face = &fixture.faces[0];
        assert!(matches!(
            ConvexPlanarFragment::new(
                face.source_face(),
                face.vertices()[..2].to_vec(),
                vec![fixture.plane_ids[2]; 2],
            ),
            Err(FragmentError::InvalidRing)
        ));
        assert!(matches!(
            classify_fragment(face, &fixture.plane_ids),
            Err(FragmentError::MissingClassification)
        ));
    }
}
