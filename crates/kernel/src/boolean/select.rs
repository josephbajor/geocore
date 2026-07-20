//! Deterministic truth selection for symbolic planar Boolean fragments.
//!
//! The BSP stage partitions every source boundary face by the other convex
//! solid and classifies each positive-area fragment as strictly interior or
//! exterior. This module applies the three regularized CSG truth tables to
//! those classifications. It retains source-face orientation for union and
//! intersection; subtraction reverses only boundary contributed by the
//! subtrahend.

use std::collections::{BTreeMap, BTreeSet};

use super::planar_bsp::{
    ConvexPlanarFragment, FragmentClassification, FragmentError, PlaneTripleVertexKey, SourcePlane,
    SourcePlaneRef, classify_fragment, partition_fragments,
};

/// One regularized planar Boolean truth table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PlanarBooleanOperation {
    Unite,
    Intersect,
    Subtract,
}

/// Operand ownership is explicit rather than inferred from caller-assigned
/// plane numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum OperandSide {
    Left,
    Right,
}

/// Orientation of a selected polygon relative to its source face ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum SelectedOrientation {
    Preserved,
    Reversed,
}

/// Canonical identity of one partitioned source-face fragment.
///
/// The sorted symbolic vertex set makes identity independent of a cyclic
/// start vertex and source traversal direction. `ConvexPlanarFragment`
/// already admits only a labeled cyclic polygon, so the set plus source face
/// is sufficient for this bounded stage.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct SelectedFragmentKey {
    operand: OperandSide,
    source_face: SourcePlaneRef,
    vertices: Vec<PlaneTripleVertexKey>,
}

impl SelectedFragmentKey {
    fn from_fragment(
        operand: OperandSide,
        fragment: &ConvexPlanarFragment,
    ) -> Result<Self, SelectionError> {
        let mut vertices = fragment.vertices().to_vec();
        vertices.sort_unstable();
        if vertices.windows(2).any(|pair| pair[0] == pair[1]) {
            return Err(SelectionError::DegenerateFragmentKey);
        }
        Ok(Self {
            operand,
            source_face: fragment.source_face(),
            vertices,
        })
    }
}

/// A BSP fragment paired with its complete classification against the other
/// operand.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ClassifiedPlanarFragment {
    operand: OperandSide,
    fragment: ConvexPlanarFragment,
    classification: FragmentClassification,
}

impl ClassifiedPlanarFragment {
    const fn new(
        operand: OperandSide,
        fragment: ConvexPlanarFragment,
        classification: FragmentClassification,
    ) -> Self {
        Self {
            operand,
            fragment,
            classification,
        }
    }
}

/// One boundary polygon retained by the requested Boolean truth table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SelectedPlanarFragment {
    key: SelectedFragmentKey,
    fragment: ConvexPlanarFragment,
    orientation: SelectedOrientation,
}

impl SelectedPlanarFragment {
    pub(crate) fn key(&self) -> &SelectedFragmentKey {
        &self.key
    }

    pub(crate) fn fragment(&self) -> &ConvexPlanarFragment {
        &self.fragment
    }

    pub(crate) const fn orientation(&self) -> SelectedOrientation {
        self.orientation
    }

    /// Return a deterministic cyclic ring in result-boundary orientation.
    ///
    /// The smallest symbolic vertex is chosen as the start without changing
    /// the selected direction.
    pub(crate) fn oriented_vertices(&self) -> Vec<PlaneTripleVertexKey> {
        self.oriented_boundary()
            .into_iter()
            .map(|(vertex, _)| vertex)
            .collect()
    }

    /// Return the result-oriented ring with each vertex paired to the plane
    /// carrying its directed edge to the cyclic successor.
    ///
    /// Vertex and edge-plane sequences are rotated together so proof-bearing
    /// edge provenance cannot be detached by canonical start selection.
    pub(crate) fn oriented_boundary(&self) -> Vec<(PlaneTripleVertexKey, SourcePlaneRef)> {
        let mut boundary = self
            .fragment
            .vertices()
            .iter()
            .copied()
            .zip(self.fragment.edge_planes().iter().copied())
            .collect::<Vec<_>>();
        let start = boundary
            .iter()
            .enumerate()
            .min_by_key(|(_, (vertex, _))| *vertex)
            .map_or(0, |(index, _)| index);
        boundary.rotate_left(start);
        boundary
    }
}

/// Honest refusal from symbolic selection and its prerequisite BSP work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SelectionError {
    Fragment(FragmentError),
    DuplicateSolidPlane,
    InsufficientSolidPlanes,
    DuplicatePlaneWitness,
    MissingPlaneWitness,
    UnexpectedPlaneWitness,
    OverlappingOperandPlane,
    InvalidOperandBoundary,
    DegenerateFragmentKey,
    DuplicateFragmentKey,
}

impl From<FragmentError> for SelectionError {
    fn from(error: FragmentError) -> Self {
        Self::Fragment(error)
    }
}

fn selected_orientation(
    operation: PlanarBooleanOperation,
    operand: OperandSide,
    classification: FragmentClassification,
) -> Option<SelectedOrientation> {
    use FragmentClassification::{Exterior, Interior};
    use OperandSide::{Left, Right};
    use PlanarBooleanOperation::{Intersect, Subtract, Unite};
    use SelectedOrientation::{Preserved, Reversed};

    match (operation, operand, classification) {
        (Unite, _, Exterior) | (Intersect, _, Interior) => Some(Preserved),
        (Subtract, Left, Exterior) => Some(Preserved),
        (Subtract, Right, Interior) => Some(Reversed),
        _ => None,
    }
}

/// Select classified boundary fragments and return them in canonical key
/// order, independent of the input sequence.
fn select_classified_fragments(
    operation: PlanarBooleanOperation,
    fragments: impl IntoIterator<Item = ClassifiedPlanarFragment>,
) -> Result<Vec<SelectedPlanarFragment>, SelectionError> {
    let mut admitted = BTreeSet::new();
    let mut selected = BTreeMap::new();
    for classified in fragments {
        let key = SelectedFragmentKey::from_fragment(classified.operand, &classified.fragment)?;
        if !admitted.insert(key.clone()) {
            return Err(SelectionError::DuplicateFragmentKey);
        }
        let Some(orientation) =
            selected_orientation(operation, classified.operand, classified.classification)
        else {
            continue;
        };
        let fragment = match orientation {
            SelectedOrientation::Preserved => classified.fragment,
            SelectedOrientation::Reversed => classified.fragment.reversed_orientation(),
        };
        selected.insert(
            key.clone(),
            SelectedPlanarFragment {
                key,
                fragment,
                orientation,
            },
        );
    }
    Ok(selected.into_values().collect())
}

fn canonical_solid_planes(
    planes: &[SourcePlaneRef],
) -> Result<Vec<SourcePlaneRef>, SelectionError> {
    let mut canonical = planes.to_vec();
    canonical.sort_unstable();
    if canonical.windows(2).any(|pair| pair[0] == pair[1]) {
        return Err(SelectionError::DuplicateSolidPlane);
    }
    if canonical.len() < 4 {
        return Err(SelectionError::InsufficientSolidPlanes);
    }
    Ok(canonical)
}

fn validate_plane_registry(
    planes: &[SourcePlane],
    left_solid_planes: &[SourcePlaneRef],
    right_solid_planes: &[SourcePlaneRef],
) -> Result<(), SelectionError> {
    let mut witnesses = BTreeSet::new();
    for plane in planes {
        if !witnesses.insert(plane.id()) {
            return Err(SelectionError::DuplicatePlaneWitness);
        }
    }
    if left_solid_planes
        .iter()
        .any(|plane| right_solid_planes.binary_search(plane).is_ok())
    {
        return Err(SelectionError::OverlappingOperandPlane);
    }
    if left_solid_planes
        .iter()
        .chain(right_solid_planes)
        .any(|plane| !witnesses.contains(plane))
    {
        return Err(SelectionError::MissingPlaneWitness);
    }
    if witnesses.len() != left_solid_planes.len() + right_solid_planes.len() {
        return Err(SelectionError::UnexpectedPlaneWitness);
    }
    Ok(())
}

fn validate_operand_boundary(
    faces: &[ConvexPlanarFragment],
    solid_planes: &[SourcePlaneRef],
) -> Result<(), SelectionError> {
    let source_faces = faces
        .iter()
        .map(ConvexPlanarFragment::source_face)
        .collect::<BTreeSet<_>>();
    if faces.len() != solid_planes.len()
        || source_faces.len() != solid_planes.len()
        || source_faces
            .iter()
            .any(|face| solid_planes.binary_search(face).is_err())
    {
        return Err(SelectionError::InvalidOperandBoundary);
    }
    Ok(())
}

fn partition_and_classify(
    planes: &[SourcePlane],
    operand: OperandSide,
    seeds: Vec<ConvexPlanarFragment>,
    cutters: &[SourcePlaneRef],
) -> Result<Vec<ClassifiedPlanarFragment>, SelectionError> {
    partition_fragments(planes, seeds, cutters)?
        .into_iter()
        .map(|fragment| {
            let classification = classify_fragment(&fragment, cutters)?;
            Ok(ClassifiedPlanarFragment::new(
                operand,
                fragment,
                classification,
            ))
        })
        .collect()
}

/// Partition, classify, and truth-select two convex planar boundary sets.
///
/// This remains a bounded internal stage: exact boundary contact and any
/// uncertified plane predicate are propagated as refusals, and no topology is
/// allocated here.
pub(crate) fn select_boolean_fragments(
    operation: PlanarBooleanOperation,
    planes: &[SourcePlane],
    left_faces: Vec<ConvexPlanarFragment>,
    left_solid_planes: &[SourcePlaneRef],
    right_faces: Vec<ConvexPlanarFragment>,
    right_solid_planes: &[SourcePlaneRef],
) -> Result<Vec<SelectedPlanarFragment>, SelectionError> {
    let left_solid_planes = canonical_solid_planes(left_solid_planes)?;
    let right_solid_planes = canonical_solid_planes(right_solid_planes)?;
    validate_plane_registry(planes, &left_solid_planes, &right_solid_planes)?;
    validate_operand_boundary(&left_faces, &left_solid_planes)?;
    validate_operand_boundary(&right_faces, &right_solid_planes)?;
    let mut classified =
        partition_and_classify(planes, OperandSide::Left, left_faces, &right_solid_planes)?;
    classified.extend(partition_and_classify(
        planes,
        OperandSide::Right,
        right_faces,
        &left_solid_planes,
    )?);
    select_classified_fragments(operation, classified)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    const FACE_CORNERS: [[usize; 4]; 6] = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];

    #[derive(Clone, Copy)]
    struct NumericPlane {
        normal: [f64; 3],
        offset: f64,
        interior_sign: f64,
    }

    struct BoxFixture {
        planes: Vec<SourcePlane>,
        numeric_planes: BTreeMap<SourcePlaneRef, NumericPlane>,
        plane_ids: Vec<SourcePlaneRef>,
        faces: Vec<ConvexPlanarFragment>,
    }

    fn subtract(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
        core::array::from_fn(|index| left[index] - right[index])
    }

    fn cross(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
        [
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        ]
    }

    fn dot(left: [f64; 3], right: [f64; 3]) -> f64 {
        left.into_iter()
            .zip(right)
            .map(|(left, right)| left * right)
            .sum()
    }

    fn numeric_plane(points: [[f64; 3]; 3], interior_sample: [f64; 3]) -> NumericPlane {
        let normal = cross(
            subtract(points[1], points[0]),
            subtract(points[2], points[0]),
        );
        let offset = -dot(normal, points[0]);
        let interior_sign = dot(normal, interior_sample) + offset;
        assert_ne!(interior_sign, 0.0);
        NumericPlane {
            normal,
            offset,
            interior_sign,
        }
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
                transform(
                    [
                        if index & 1 == 0 { -half[0] } else { half[0] },
                        if index & 2 == 0 { -half[1] } else { half[1] },
                        if index & 4 == 0 { -half[2] } else { half[2] },
                    ],
                    center,
                    matrix,
                )
            })
            .collect();
        let plane_ids = (0..6)
            .map(|face| SourcePlaneRef::new(operand, face))
            .collect::<Vec<_>>();
        let mut numeric_planes = BTreeMap::new();
        let planes = FACE_CORNERS
            .iter()
            .enumerate()
            .map(|(face, ring)| {
                let points = [corners[ring[0]], corners[ring[1]], corners[ring[2]]];
                numeric_planes.insert(plane_ids[face], numeric_plane(points, center));
                SourcePlane::from_interior_sample(plane_ids[face], points, center).unwrap()
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
            numeric_planes,
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

    fn combined_planes(left: &BoxFixture, right: &BoxFixture) -> Vec<SourcePlane> {
        let mut planes = left.planes.clone();
        planes.extend_from_slice(&right.planes);
        planes
    }

    fn select(
        operation: PlanarBooleanOperation,
        left: &BoxFixture,
        right: &BoxFixture,
    ) -> Result<Vec<SelectedPlanarFragment>, SelectionError> {
        select_boolean_fragments(
            operation,
            &combined_planes(left, right),
            left.faces.clone(),
            &left.plane_ids,
            right.faces.clone(),
            &right.plane_ids,
        )
    }

    fn orientation_counts(selected: &[SelectedPlanarFragment]) -> (usize, usize) {
        selected.iter().fold((0, 0), |mut counts, fragment| {
            match fragment.orientation() {
                SelectedOrientation::Preserved => counts.0 += 1,
                SelectedOrientation::Reversed => counts.1 += 1,
            }
            counts
        })
    }

    #[test]
    fn disjoint_truth_tables_retain_exact_source_boundaries() {
        let left = box_fixture(0, [-4.0, 1.0, 2.0], [1.0; 3], identity());
        let right = box_fixture(1, [4.0, -1.0, -2.0], [0.75; 3], identity());

        assert_eq!(
            orientation_counts(&select(PlanarBooleanOperation::Unite, &left, &right).unwrap()),
            (12, 0)
        );
        assert!(
            select(PlanarBooleanOperation::Intersect, &left, &right)
                .unwrap()
                .is_empty()
        );
        let difference = select(PlanarBooleanOperation::Subtract, &left, &right).unwrap();
        assert_eq!(orientation_counts(&difference), (6, 0));
        assert!(
            difference
                .iter()
                .all(|fragment| fragment.key().operand == OperandSide::Left)
        );
    }

    #[test]
    fn containment_truth_tables_create_a_reversed_cavity_boundary() {
        let outer = box_fixture(0, [3.0, -2.0, 5.0], [2.0, 1.5, 1.25], identity());
        let inner = box_fixture(1, [3.0, -2.0, 5.0], [0.5, 0.4, 0.3], identity());

        let union = select(PlanarBooleanOperation::Unite, &outer, &inner).unwrap();
        assert_eq!(orientation_counts(&union), (union.len(), 0));
        assert!(
            union
                .iter()
                .all(|fragment| fragment.key().operand == OperandSide::Left)
        );
        assert_eq!(
            union
                .iter()
                .map(|fragment| fragment.fragment().source_face())
                .collect::<BTreeSet<_>>(),
            outer.plane_ids.iter().copied().collect()
        );

        let intersection = select(PlanarBooleanOperation::Intersect, &outer, &inner).unwrap();
        assert_eq!(orientation_counts(&intersection), (6, 0));
        assert!(
            intersection
                .iter()
                .all(|fragment| fragment.key().operand == OperandSide::Right)
        );

        let cavity = select(PlanarBooleanOperation::Subtract, &outer, &inner).unwrap();
        assert_eq!(orientation_counts(&cavity), (union.len(), 6));
        assert!(cavity.iter().all(|fragment| {
            fragment.orientation()
                == if fragment.key().operand == OperandSide::Left {
                    SelectedOrientation::Preserved
                } else {
                    SelectedOrientation::Reversed
                }
        }));
        assert_eq!(
            cavity
                .iter()
                .filter(|fragment| fragment.key().operand == OperandSide::Left)
                .map(|fragment| fragment.key().clone())
                .collect::<BTreeSet<_>>(),
            union
                .iter()
                .map(|fragment| fragment.key().clone())
                .collect()
        );

        assert!(
            select(PlanarBooleanOperation::Subtract, &inner, &outer)
                .unwrap()
                .is_empty()
        );
    }

    fn plane_triple_for_vertex(
        vertex: PlaneTripleVertexKey,
        plane_ids: &[SourcePlaneRef],
    ) -> [SourcePlaneRef; 3] {
        for first in 0..plane_ids.len() {
            for second in first + 1..plane_ids.len() {
                for third in second + 1..plane_ids.len() {
                    let triple = [plane_ids[first], plane_ids[second], plane_ids[third]];
                    if PlaneTripleVertexKey::new(triple) == Some(vertex) {
                        return triple;
                    }
                }
            }
        }
        panic!("symbolic vertex must decode against fixture planes")
    }

    fn determinant(matrix: [[f64; 3]; 3]) -> f64 {
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    }

    fn intersection(
        triple: [SourcePlaneRef; 3],
        numeric_planes: &BTreeMap<SourcePlaneRef, NumericPlane>,
    ) -> [f64; 3] {
        let rows = triple.map(|id| numeric_planes[&id]);
        let matrix = rows.map(|plane| plane.normal);
        let rhs = rows.map(|plane| -plane.offset);
        let denominator = determinant(matrix);
        assert_ne!(denominator, 0.0);
        core::array::from_fn(|column| {
            let mut numerator = matrix;
            for row in 0..3 {
                numerator[row][column] = rhs[row];
            }
            determinant(numerator) / denominator
        })
    }

    fn centroid(
        fragment: &ConvexPlanarFragment,
        plane_ids: &[SourcePlaneRef],
        numeric_planes: &BTreeMap<SourcePlaneRef, NumericPlane>,
    ) -> [f64; 3] {
        let mut sum = [0.0; 3];
        for &vertex in fragment.vertices() {
            let point = intersection(plane_triple_for_vertex(vertex, plane_ids), numeric_planes);
            for coordinate in 0..3 {
                sum[coordinate] += point[coordinate];
            }
        }
        sum.map(|coordinate| coordinate / fragment.vertices().len() as f64)
    }

    fn strictly_inside(point: [f64; 3], solid: &BoxFixture) -> bool {
        solid.numeric_planes.values().all(|plane| {
            let point_sign = dot(plane.normal, point) + plane.offset;
            point_sign * plane.interior_sign > 0.0
        })
    }

    fn independent_boolean_truth(
        operation: PlanarBooleanOperation,
        left_inside: bool,
        right_inside: bool,
    ) -> bool {
        match operation {
            PlanarBooleanOperation::Unite => left_inside || right_inside,
            PlanarBooleanOperation::Intersect => left_inside && right_inside,
            PlanarBooleanOperation::Subtract => left_inside && !right_inside,
        }
    }

    /// Compare result occupancy immediately inside and outside a source
    /// boundary. This derives retention and direction from Boolean set
    /// membership rather than reusing the production selection table.
    fn independent_orientation(
        operation: PlanarBooleanOperation,
        operand: OperandSide,
        other_inside: bool,
    ) -> Option<SelectedOrientation> {
        let (result_on_source_interior, result_on_source_exterior) = match operand {
            OperandSide::Left => (
                independent_boolean_truth(operation, true, other_inside),
                independent_boolean_truth(operation, false, other_inside),
            ),
            OperandSide::Right => (
                independent_boolean_truth(operation, other_inside, true),
                independent_boolean_truth(operation, other_inside, false),
            ),
        };
        match (result_on_source_interior, result_on_source_exterior) {
            (true, false) => Some(SelectedOrientation::Preserved),
            (false, true) => Some(SelectedOrientation::Reversed),
            _ => None,
        }
    }

    fn independent_expected(
        operation: PlanarBooleanOperation,
        left: &BoxFixture,
        right: &BoxFixture,
    ) -> BTreeMap<SelectedFragmentKey, SelectedOrientation> {
        let planes = combined_planes(left, right);
        let mut numeric_planes = left.numeric_planes.clone();
        numeric_planes.extend(right.numeric_planes.clone());
        let mut plane_ids = left.plane_ids.clone();
        plane_ids.extend_from_slice(&right.plane_ids);
        let mut expected = BTreeMap::new();
        for (operand, source, cutter) in [
            (OperandSide::Left, left, right),
            (OperandSide::Right, right, left),
        ] {
            let fragments = partition_fragments(&planes, source.faces.clone(), &cutter.plane_ids)
                .expect("general-position fixture partitions");
            for fragment in fragments {
                let inside =
                    strictly_inside(centroid(&fragment, &plane_ids, &numeric_planes), cutter);
                if let Some(orientation) = independent_orientation(operation, operand, inside) {
                    expected.insert(
                        SelectedFragmentKey::from_fragment(operand, &fragment).unwrap(),
                        orientation,
                    );
                }
            }
        }
        expected
    }

    #[test]
    fn rotated_overlap_matches_an_independent_numeric_halfspace_oracle() {
        let left = box_fixture(0, [7.0, -5.0, 3.0], [1.5, 1.0, 1.25], identity());
        let right = box_fixture(1, [7.6, -4.7, 3.2], [1.3, 0.9, 1.1], rotation_z(0.47));

        for operation in [
            PlanarBooleanOperation::Unite,
            PlanarBooleanOperation::Intersect,
            PlanarBooleanOperation::Subtract,
        ] {
            let selected = select(operation, &left, &right).unwrap();
            let actual = selected
                .iter()
                .map(|fragment| (fragment.key().clone(), fragment.orientation()))
                .collect::<BTreeMap<_, _>>();
            assert_eq!(actual, independent_expected(operation, &left, &right));
            assert!(orientation_counts(&selected).0 > 0);
            assert_eq!(
                selected
                    .iter()
                    .map(SelectedPlanarFragment::key)
                    .collect::<Vec<_>>(),
                actual.keys().collect::<Vec<_>>()
            );
        }

        let mut reversed_planes = combined_planes(&left, &right);
        reversed_planes.reverse();
        let mut reversed_left_faces = left.faces.clone();
        reversed_left_faces.reverse();
        let mut reversed_right_faces = right.faces.clone();
        reversed_right_faces.reverse();
        let mut reversed_left_ids = left.plane_ids.clone();
        reversed_left_ids.reverse();
        let mut reversed_right_ids = right.plane_ids.clone();
        reversed_right_ids.reverse();
        assert_eq!(
            select(PlanarBooleanOperation::Unite, &left, &right).unwrap(),
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &reversed_planes,
                reversed_left_faces,
                &reversed_left_ids,
                reversed_right_faces,
                &reversed_right_ids,
            )
            .unwrap()
        );
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
    enum Shape {
        First,
        Second,
    }

    #[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
    struct SemanticFragmentKey {
        source_shape: Shape,
        source_face: u32,
        vertices: Vec<[(Shape, u32); 3]>,
        orientation: SelectedOrientation,
    }

    fn semantic_plane(
        id: SourcePlaneRef,
        first_ids: &[SourcePlaneRef],
        second_ids: &[SourcePlaneRef],
    ) -> (Shape, u32) {
        if let Some(face) = first_ids.iter().position(|candidate| *candidate == id) {
            (Shape::First, face as u32)
        } else {
            let face = second_ids
                .iter()
                .position(|candidate| *candidate == id)
                .expect("fixture owns every symbolic plane");
            (Shape::Second, face as u32)
        }
    }

    fn semantic_result(
        selected: &[SelectedPlanarFragment],
        first_side: OperandSide,
        first_ids: &[SourcePlaneRef],
        second_ids: &[SourcePlaneRef],
    ) -> BTreeSet<SemanticFragmentKey> {
        let mut all_ids = first_ids.to_vec();
        all_ids.extend_from_slice(second_ids);
        selected
            .iter()
            .map(|selected| {
                let source_shape = if selected.key().operand == first_side {
                    Shape::First
                } else {
                    Shape::Second
                };
                let source_ids = if source_shape == Shape::First {
                    first_ids
                } else {
                    second_ids
                };
                let source_face = source_ids
                    .iter()
                    .position(|id| *id == selected.fragment().source_face())
                    .unwrap() as u32;
                let mut vertices = selected
                    .fragment()
                    .vertices()
                    .iter()
                    .map(|&vertex| {
                        let mut semantic = plane_triple_for_vertex(vertex, &all_ids)
                            .map(|id| semantic_plane(id, first_ids, second_ids));
                        semantic.sort_unstable();
                        semantic
                    })
                    .collect::<Vec<_>>();
                vertices.sort_unstable();
                SemanticFragmentKey {
                    source_shape,
                    source_face,
                    vertices,
                    orientation: selected.orientation(),
                }
            })
            .collect()
    }

    #[test]
    fn commutative_operations_are_invariant_to_operand_swap() {
        let first = box_fixture(0, [2.0, 3.0, -4.0], [1.2, 0.9, 1.1], identity());
        let second = box_fixture(1, [2.25, 2.8, -3.7], [1.0, 1.1, 0.8], rotation_z(0.39));
        let swapped_first = box_fixture(1, [2.0, 3.0, -4.0], [1.2, 0.9, 1.1], identity());
        let swapped_second = box_fixture(0, [2.25, 2.8, -3.7], [1.0, 1.1, 0.8], rotation_z(0.39));

        for operation in [
            PlanarBooleanOperation::Unite,
            PlanarBooleanOperation::Intersect,
        ] {
            let direct = select(operation, &first, &second).unwrap();
            let swapped = select(operation, &swapped_second, &swapped_first).unwrap();
            assert_eq!(
                semantic_result(
                    &direct,
                    OperandSide::Left,
                    &first.plane_ids,
                    &second.plane_ids,
                ),
                semantic_result(
                    &swapped,
                    OperandSide::Right,
                    &swapped_first.plane_ids,
                    &swapped_second.plane_ids,
                )
            );
        }
    }

    #[test]
    fn exact_contact_is_refused_for_every_truth_table_and_operand_order() {
        let first = box_fixture(0, [0.0; 3], [1.0; 3], identity());
        let touching = box_fixture(1, [2.0, 0.0, 0.0], [1.0; 3], identity());
        let swapped_first = box_fixture(1, [0.0; 3], [1.0; 3], identity());
        let swapped_touching = box_fixture(0, [2.0, 0.0, 0.0], [1.0; 3], identity());

        for operation in [
            PlanarBooleanOperation::Unite,
            PlanarBooleanOperation::Intersect,
            PlanarBooleanOperation::Subtract,
        ] {
            for result in [
                select(operation, &first, &touching),
                select(operation, &swapped_touching, &swapped_first),
            ] {
                assert_eq!(
                    result,
                    Err(SelectionError::Fragment(FragmentError::BoundaryContact))
                );
            }
        }
    }

    #[test]
    fn duplicate_evidence_is_rejected_before_truth_filtering() {
        let fixture = box_fixture(0, [0.0; 3], [1.0; 3], identity());
        let duplicate = ClassifiedPlanarFragment::new(
            OperandSide::Left,
            fixture.faces[0].clone(),
            FragmentClassification::Interior,
        );
        assert_eq!(
            select_classified_fragments(
                PlanarBooleanOperation::Unite,
                [duplicate.clone(), duplicate]
            ),
            Err(SelectionError::DuplicateFragmentKey)
        );
        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &fixture.planes,
                fixture.faces.clone(),
                &[fixture.plane_ids[0], fixture.plane_ids[0],],
                Vec::new(),
                &[],
            ),
            Err(SelectionError::DuplicateSolidPlane)
        );
    }

    #[test]
    fn operand_preflight_rejects_ambiguous_or_incomplete_registries() {
        let left = box_fixture(0, [-3.0, 0.0, 0.0], [1.0; 3], identity());
        let right = box_fixture(1, [3.0, 0.0, 0.0], [1.0; 3], identity());
        let planes = combined_planes(&left, &right);

        let mut duplicate_witness = planes.clone();
        duplicate_witness.push(planes[0]);
        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &duplicate_witness,
                left.faces.clone(),
                &left.plane_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::DuplicatePlaneWitness)
        );

        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &planes[1..],
                left.faces.clone(),
                &left.plane_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::MissingPlaneWitness)
        );

        let unrelated = box_fixture(2, [9.0, 0.0, 0.0], [1.0; 3], identity());
        let mut extra_witness = planes.clone();
        extra_witness.push(unrelated.planes[0]);
        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &extra_witness,
                left.faces.clone(),
                &left.plane_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::UnexpectedPlaneWitness)
        );

        let mut overlapping_left_ids = left.plane_ids.clone();
        overlapping_left_ids.push(right.plane_ids[0]);
        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &planes,
                left.faces.clone(),
                &overlapping_left_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::OverlappingOperandPlane)
        );

        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &planes,
                left.faces[..5].to_vec(),
                &left.plane_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::InvalidOperandBoundary)
        );

        let mut duplicate_seed = left.faces.clone();
        duplicate_seed.push(left.faces[0].clone());
        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &planes,
                duplicate_seed,
                &left.plane_ids,
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::InvalidOperandBoundary)
        );

        assert_eq!(
            select_boolean_fragments(
                PlanarBooleanOperation::Unite,
                &planes,
                left.faces.clone(),
                &left.plane_ids[..3],
                right.faces.clone(),
                &right.plane_ids,
            ),
            Err(SelectionError::InsufficientSolidPlanes)
        );
    }

    #[test]
    fn reversed_output_ring_has_the_opposite_cyclic_direction() {
        let fixture = box_fixture(0, [0.0; 3], [1.0; 3], identity());
        let fragment = fixture.faces[0].clone();
        let selected = select_classified_fragments(
            PlanarBooleanOperation::Subtract,
            [ClassifiedPlanarFragment::new(
                OperandSide::Right,
                fragment.clone(),
                FragmentClassification::Interior,
            )],
        )
        .unwrap()
        .pop()
        .unwrap();
        let ring = selected.oriented_vertices();
        let boundary = selected.oriented_boundary();
        assert_eq!(
            ring,
            boundary
                .iter()
                .map(|(vertex, _)| *vertex)
                .collect::<Vec<_>>()
        );
        for index in 0..boundary.len() {
            let (from, carrier) = boundary[index];
            let (to, _) = boundary[(index + 1) % boundary.len()];
            assert!(from.planes().contains(&carrier));
            assert!(to.planes().contains(&carrier));
        }
        let source = fragment.vertices();
        let start = source.iter().position(|vertex| *vertex == ring[0]).unwrap();
        assert_eq!(ring[1], source[(start + source.len() - 1) % source.len()]);
        assert_eq!(ring.last(), Some(&source[(start + 1) % source.len()]));
    }
}
