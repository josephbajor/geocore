//! Deterministic shell-component partitioning for selected planar boundaries.
//!
//! Connectivity here is purely combinatorial. Two selected faces are adjacent
//! only when they use the same complete canonical symbolic edge in opposite
//! directions. Shared vertices never connect components, and numeric vertex
//! representatives have no role in this stage.

use std::collections::{BTreeMap, BTreeSet};

use super::planar_bsp::PlaneTripleVertexKey;
use super::select::{SelectedFragmentKey, SelectedPlanarFragment};

/// Canonical unordered identity of one complete symbolic boundary edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct SymbolicEdgeKey {
    first: PlaneTripleVertexKey,
    second: PlaneTripleVertexKey,
}

impl SymbolicEdgeKey {
    fn new(first: PlaneTripleVertexKey, second: PlaneTripleVertexKey) -> Option<Self> {
        (first != second).then(|| {
            let (first, second) = if first < second {
                (first, second)
            } else {
                (second, first)
            };
            Self { first, second }
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct DirectedEdgeUse {
    face: usize,
    from: PlaneTripleVertexKey,
    to: PlaneTripleVertexKey,
}

/// Fail-closed refusal from symbolic shell-component partitioning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ComponentPartitionError {
    /// A caller supplied the same selected face identity more than once.
    DuplicateFragmentKey,
    /// A selected face contained a zero-length symbolic boundary edge.
    DegenerateEdge,
    /// A complete boundary edge did not have exactly two incident face uses.
    MalformedEdgeUseCount {
        first: PlaneTripleVertexKey,
        second: PlaneTripleVertexKey,
        count: usize,
    },
    /// The two incident faces did not traverse their common edge oppositely.
    NonOpposedEdgeUses {
        first: PlaneTripleVertexKey,
        second: PlaneTripleVertexKey,
    },
}

/// One connected, closed selected boundary component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct SelectedShellComponent {
    faces: Vec<SelectedPlanarFragment>,
}

impl SelectedShellComponent {
    /// Selected faces in ascending stable fragment-key order.
    pub(crate) fn faces(&self) -> &[SelectedPlanarFragment] {
        &self.faces
    }

    /// Minimum stable face identity, which also orders result components.
    pub(crate) fn minimum_key(&self) -> &SelectedFragmentKey {
        self.faces[0].key()
    }
}

/// Partition a selected planar boundary into closed edge-connected shells.
///
/// Input order is discarded. Every admitted edge has exactly two uses by
/// distinct faces in opposite directions. Components are ordered by their
/// minimum stable selected-fragment key, and faces within each component use
/// that same stable order.
pub(crate) fn partition_shell_components(
    mut selected: Vec<SelectedPlanarFragment>,
) -> Result<Vec<SelectedShellComponent>, ComponentPartitionError> {
    selected.sort_by(|left, right| left.key().cmp(right.key()));
    if selected
        .windows(2)
        .any(|pair| pair[0].key() == pair[1].key())
    {
        return Err(ComponentPartitionError::DuplicateFragmentKey);
    }
    if selected.is_empty() {
        return Ok(Vec::new());
    }

    let mut edge_uses: BTreeMap<SymbolicEdgeKey, Vec<DirectedEdgeUse>> = BTreeMap::new();
    for (face, fragment) in selected.iter().enumerate() {
        let boundary = fragment.oriented_vertices();
        for index in 0..boundary.len() {
            let from = boundary[index];
            let to = boundary[(index + 1) % boundary.len()];
            let edge =
                SymbolicEdgeKey::new(from, to).ok_or(ComponentPartitionError::DegenerateEdge)?;
            edge_uses
                .entry(edge)
                .or_default()
                .push(DirectedEdgeUse { face, from, to });
        }
    }

    let mut neighbors = vec![BTreeSet::new(); selected.len()];
    for (edge, uses) in edge_uses {
        let [first, second] = uses.as_slice() else {
            return Err(ComponentPartitionError::MalformedEdgeUseCount {
                first: edge.first,
                second: edge.second,
                count: uses.len(),
            });
        };
        if first.face == second.face || first.from != second.to || first.to != second.from {
            return Err(ComponentPartitionError::NonOpposedEdgeUses {
                first: edge.first,
                second: edge.second,
            });
        }
        neighbors[first.face].insert(second.face);
        neighbors[second.face].insert(first.face);
    }

    let mut component_indices = Vec::new();
    let mut seen = vec![false; selected.len()];
    for start in 0..selected.len() {
        if seen[start] {
            continue;
        }
        let mut component = BTreeSet::new();
        let mut pending = vec![start];
        while let Some(face) = pending.pop() {
            if !seen[face] {
                seen[face] = true;
                component.insert(face);
                pending.extend(neighbors[face].iter().rev().copied());
            }
        }
        component_indices.push(component.into_iter().collect::<Vec<_>>());
    }

    let mut selected = selected.into_iter().map(Some).collect::<Vec<_>>();
    let mut components = component_indices
        .into_iter()
        .map(|indices| SelectedShellComponent {
            faces: indices
                .into_iter()
                .map(|index| {
                    selected[index]
                        .take()
                        .expect("component indices are disjoint")
                })
                .collect(),
        })
        .collect::<Vec<_>>();
    components.sort_by(|left, right| left.minimum_key().cmp(right.minimum_key()));
    Ok(components)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::planar_bsp::{ConvexPlanarFragment, SourcePlane, SourcePlaneRef};
    use crate::boolean::select::{
        PlanarBooleanOperation, SelectedOrientation, select_boolean_fragments,
    };

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
        let corners = (0..8_u8)
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
            .collect::<Vec<_>>();
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
        let corner_planes = (0..8_u8)
            .map(|index| {
                [
                    plane_ids[if index & 1 == 0 { 4 } else { 5 }],
                    plane_ids[if index & 2 == 0 { 2 } else { 3 }],
                    plane_ids[if index & 4 == 0 { 0 } else { 1 }],
                ]
            })
            .collect::<Vec<_>>();
        let faces = FACE_CORNERS
            .iter()
            .enumerate()
            .map(|(face, ring)| {
                let vertices = ring
                    .iter()
                    .map(|&corner| PlaneTripleVertexKey::new(corner_planes[corner]).unwrap())
                    .collect::<Vec<_>>();
                let edge_planes = (0..ring.len())
                    .map(|index| {
                        let first = corner_planes[ring[index]];
                        let second = corner_planes[ring[(index + 1) % ring.len()]];
                        first
                            .into_iter()
                            .find(|plane| *plane != plane_ids[face] && second.contains(plane))
                            .unwrap()
                    })
                    .collect();
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

    fn select(
        operation: PlanarBooleanOperation,
        left: &BoxFixture,
        right: &BoxFixture,
    ) -> Vec<SelectedPlanarFragment> {
        let mut planes = left.planes.clone();
        planes.extend_from_slice(&right.planes);
        select_boolean_fragments(
            operation,
            &planes,
            left.faces.clone(),
            &left.plane_ids,
            right.faces.clone(),
            &right.plane_ids,
        )
        .unwrap()
    }

    #[derive(Clone, Copy)]
    enum ExpectedComponent {
        Positive,
        Negative,
    }

    #[test]
    fn partitions_connected_disjoint_and_cavity_boundaries_by_full_edges() {
        let overlapping_left = box_fixture(0, [0.0, 0.0, 0.0], [1.5, 1.2, 1.0], identity());
        let overlapping_right =
            box_fixture(1, [0.4, -0.2, 0.2], [1.25, 1.0, 0.9], rotation_z(0.47));
        let disjoint_left = box_fixture(0, [-4.0, 1.0, 2.0], [1.0; 3], identity());
        let disjoint_right = box_fixture(1, [4.0, -1.0, -2.0], [0.75; 3], identity());
        let outer = box_fixture(0, [3.0, -2.0, 5.0], [2.0, 1.5, 1.25], identity());
        let inner = box_fixture(1, [3.0, -2.0, 5.0], [0.5, 0.4, 0.3], identity());

        let cases = [
            (
                "connected overlap",
                select(
                    PlanarBooleanOperation::Unite,
                    &overlapping_left,
                    &overlapping_right,
                ),
                vec![ExpectedComponent::Positive],
            ),
            (
                "disjoint union",
                select(
                    PlanarBooleanOperation::Unite,
                    &disjoint_left,
                    &disjoint_right,
                ),
                vec![ExpectedComponent::Positive, ExpectedComponent::Positive],
            ),
            (
                "containment subtraction",
                select(PlanarBooleanOperation::Subtract, &outer, &inner),
                vec![ExpectedComponent::Positive, ExpectedComponent::Negative],
            ),
        ];

        for (name, selected, expected) in cases {
            let mut reversed = selected.clone();
            reversed.reverse();
            let components = partition_shell_components(selected).unwrap();
            assert_eq!(
                components,
                partition_shell_components(reversed).unwrap(),
                "{name} depends on input order"
            );
            assert_eq!(components.len(), expected.len(), "{name}");
            for (component, expected) in components.iter().zip(expected) {
                assert!(!component.faces().is_empty(), "{name}");
                assert_eq!(
                    component.minimum_key(),
                    component.faces()[0].key(),
                    "{name}"
                );
                assert!(
                    component
                        .faces()
                        .windows(2)
                        .all(|pair| pair[0].key() < pair[1].key()),
                    "{name} face order is unstable"
                );
                let orientation = match expected {
                    ExpectedComponent::Positive => SelectedOrientation::Preserved,
                    ExpectedComponent::Negative => SelectedOrientation::Reversed,
                };
                assert!(
                    component
                        .faces()
                        .iter()
                        .all(|face| face.orientation() == orientation),
                    "{name} component orientation"
                );
            }
        }
    }

    #[test]
    fn incomplete_boundary_refuses_with_exact_edge_use_count() {
        let left = box_fixture(0, [-4.0, 1.0, 2.0], [1.0; 3], identity());
        let right = box_fixture(1, [4.0, -1.0, -2.0], [0.75; 3], identity());
        let mut selected = select(PlanarBooleanOperation::Unite, &left, &right);
        selected.pop();

        assert!(matches!(
            partition_shell_components(selected),
            Err(ComponentPartitionError::MalformedEdgeUseCount { count: 1, .. })
        ));
    }
}
