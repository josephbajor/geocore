//! Pairwise embedding and orientation consumers for semantic planar shells.
//!
//! Stable support/carrier identities determine the ideal planes and lines;
//! complete vertex coordinate intervals determine every numeric bound. No
//! representative point or midpoint is granted topological authority.

use crate::entity::{EdgeId, SurfaceId, VertexId};
use crate::semantic_planar_math::{
    IntervalPlane, IntervalVec3, certified_nonzero, cross, determinant, finite_interval,
    plane_from_witness, plane_value, strictly_separated, sub,
};
use crate::semantic_planar_shell_proof::{
    SemanticFacetEvidence, SemanticPlanarShellEvidence, SemanticVertexEvidence,
};
use kcore::interval::Interval;

/// Certified relation between two ideal convex planar facets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SemanticFacetPairRelation {
    /// A complete interval axis proves a positive projection gap.
    Disjoint,
    /// The only contact is a shared topological edge or vertex.
    AuthorizedContact,
    /// No positive gap or authorized ideal contact was certified.
    Ambiguous,
}

/// Deterministic upper-bound work charged for one facet pair.
///
/// For rings of `m` and `n` edges, `3 + 2m + 2n + mn` is the complete
/// thin-prism axis count. Every axis is constructed and projected over all
/// vertices. Two `mn` scans cover stable common-edge/common-vertex discovery,
/// and one `m + n` scan covers the largest authorized-contact side proof.
pub(crate) fn semantic_facet_pair_work(
    left: &SemanticFacetEvidence,
    right: &SemanticFacetEvidence,
) -> Option<u64> {
    let m = u64::try_from(left.edges().len()).ok()?;
    let n = u64::try_from(right.edges().len()).ok()?;
    pair_work_counts(m, n)
}

fn pair_work_counts(m: u64, n: u64) -> Option<u64> {
    let vertices = m.checked_add(n)?;
    let products = m.checked_mul(n)?;
    let axes = 3_u64
        .checked_add(m.checked_mul(2)?)?
        .checked_add(n.checked_mul(2)?)?
        .checked_add(products)?;
    axes.checked_mul(vertices.checked_add(1)?)?
        .checked_add(products.checked_mul(2)?)?
        .checked_add(vertices)?
        .checked_add(1)
}

/// Deterministic work for the interval signed-volume fan sum.
pub(crate) fn semantic_signed_volume_work(evidence: &SemanticPlanarShellEvidence) -> Option<u64> {
    evidence.facets().iter().try_fold(0_u64, |sum, facet| {
        let vertices = u64::try_from(facet.vertices().len()).ok()?;
        sum.checked_add(vertices.checked_sub(2)?)
    })
}

/// Prove one pair disjoint or confined to an authorized shared feature.
pub(crate) fn certify_semantic_facet_pair(
    shell: &SemanticPlanarShellEvidence,
    left: &SemanticFacetEvidence,
    right: &SemanticFacetEvidence,
) -> SemanticFacetPairRelation {
    if left.face() == right.face() {
        return SemanticFacetPairRelation::Ambiguous;
    }
    certify_prepared_facet_pair(shell, left, right)
}

fn certify_prepared_facet_pair(
    shell: &SemanticPlanarShellEvidence,
    left: &SemanticFacetEvidence,
    right: &SemanticFacetEvidence,
) -> SemanticFacetPairRelation {
    let Some(left) = ProofFacet::new(shell, left) else {
        return SemanticFacetPairRelation::Ambiguous;
    };
    let Some(right) = ProofFacet::new(shell, right) else {
        return SemanticFacetPairRelation::Ambiguous;
    };

    let common_vertices: Vec<_> = left
        .vertices
        .iter()
        .filter(|vertex| {
            right
                .vertices
                .iter()
                .any(|candidate| candidate.id == vertex.id)
        })
        .map(|vertex| vertex.id)
        .collect();
    let common_edges: Vec<_> = left
        .edges
        .iter()
        .filter_map(|edge| {
            right
                .edges
                .iter()
                .find(|candidate| candidate.id == edge.id)
                .map(|other| (edge, other))
        })
        .collect();

    if common_edges.len() > 1 || common_vertices.len() > 2 {
        return SemanticFacetPairRelation::Ambiguous;
    }
    if let [(left_edge, right_edge)] = common_edges.as_slice() {
        let endpoints = left_edge.endpoints;
        if common_vertices.len() != 2
            || !same_vertex_set(&common_vertices, &endpoints)
            || !same_vertex_set(&common_vertices, &right_edge.endpoints)
            || !same_surface_pair(left_edge.sources, right_edge.sources)
        {
            return SemanticFacetPairRelation::Ambiguous;
        }
        if left.support == right.support {
            if left_edge.carrier != right_edge.carrier {
                return SemanticFacetPairRelation::Ambiguous;
            }
            let left_side = strict_facet_side(&left, left_edge.carrier_plane, &endpoints);
            let right_side = strict_facet_side(&right, right_edge.carrier_plane, &endpoints);
            if matches!(
                (left_side, right_side),
                (Some(-1), Some(1)) | (Some(1), Some(-1))
            ) {
                return SemanticFacetPairRelation::AuthorizedContact;
            }
        } else if left_edge.sources.contains(&left.support)
            && left_edge.sources.contains(&right.support)
            && certified_nonzero(cross(left.normal, right.normal))
        {
            // The two exact, distinct support planes meet in the verified
            // common edge line. Strict facet halfspaces established during
            // preparation make that edge the complete line section of each
            // convex facet.
            return SemanticFacetPairRelation::AuthorizedContact;
        }
    } else if let [shared] = common_vertices.as_slice()
        && left.support != right.support
        && left.vertex(*shared).is_some_and(|vertex| {
            vertex.surfaces.contains(&left.support) && vertex.surfaces.contains(&right.support)
        })
        && distinct_support_vertex_contact_is_authorized(shell, &left, &right, *shared)
    {
        // Each strictly convex facet meets the other's exact support plane
        // only at the shared ideal vertex.
        return SemanticFacetPairRelation::AuthorizedContact;
    } else if let [shared] = common_vertices.as_slice()
        && left.support == right.support
        && same_support_vertex_contact_is_authorized(&left, &right, *shared)
    {
        return SemanticFacetPairRelation::AuthorizedContact;
    }

    for axis in separating_axes(&left, &right) {
        if strictly_separated(&left.coordinates, &right.coordinates, axis) == Some(true) {
            return SemanticFacetPairRelation::Disjoint;
        }
    }
    SemanticFacetPairRelation::Ambiguous
}

/// Complete interval enclosing six times the shell's oriented volume.
pub(crate) fn semantic_signed_volume_interval(
    evidence: &SemanticPlanarShellEvidence,
) -> Option<Interval> {
    let reference = evidence.vertices().first()?.coordinates();
    let mut six_volume = Interval::point(0.0);
    for facet in evidence.facets() {
        let [first, rest @ ..] = facet.vertices() else {
            return None;
        };
        if rest.len() < 2 {
            return None;
        }
        let origin = sub(vertex(evidence, *first)?.coordinates(), reference);
        for index in 0..rest.len() - 1 {
            let second = sub(vertex(evidence, rest[index])?.coordinates(), reference);
            let third = sub(vertex(evidence, rest[index + 1])?.coordinates(), reference);
            six_volume = six_volume + determinant(origin, second, third);
            if !finite_interval(six_volume) {
                return None;
            }
        }
    }
    Some(six_volume)
}

#[derive(Debug, Clone, Copy)]
struct ProofVertex {
    id: VertexId,
    surfaces: [SurfaceId; 3],
}

#[derive(Debug, Clone, Copy)]
struct ProofEdge {
    id: EdgeId,
    endpoints: [VertexId; 2],
    sources: [SurfaceId; 2],
    carrier: SurfaceId,
    carrier_plane: IntervalPlane,
    direction: IntervalVec3,
}

struct ProofFacet {
    support: SurfaceId,
    support_plane: IntervalPlane,
    normal: IntervalVec3,
    vertices: Vec<ProofVertex>,
    coordinates: Vec<IntervalVec3>,
    edges: Vec<ProofEdge>,
}

impl ProofFacet {
    fn new(shell: &SemanticPlanarShellEvidence, facet: &SemanticFacetEvidence) -> Option<Self> {
        if facet.vertices().len() < 3 || facet.vertices().len() != facet.edges().len() {
            return None;
        }
        let support_plane = plane_from_witness(shell.plane_witness(facet.support())?)?;
        let mut vertices = Vec::with_capacity(facet.vertices().len());
        let mut coordinates = Vec::with_capacity(facet.vertices().len());
        for &id in facet.vertices() {
            let vertex = vertex(shell, id)?;
            vertices.push(ProofVertex {
                id: vertex.vertex(),
                surfaces: vertex.surfaces(),
            });
            coordinates.push(vertex.coordinates());
        }

        let mut edges = Vec::with_capacity(facet.edges().len());
        for (index, edge) in facet.edges().iter().copied().enumerate() {
            let expected = [
                facet.vertices()[index],
                facet.vertices()[(index + 1) % facet.vertices().len()],
            ];
            if edge.endpoints() != expected {
                return None;
            }
            let sources = edge.source_surfaces();
            let carrier = sources
                .into_iter()
                .find(|surface| *surface != facet.support())?;
            if !sources.contains(&facet.support()) {
                return None;
            }
            let carrier_plane = plane_from_witness(shell.plane_witness(carrier)?)?;
            let direction = cross(facet.normal(), carrier_plane.normal);
            if !certified_nonzero(direction) {
                return None;
            }
            edges.push(ProofEdge {
                id: edge.edge(),
                endpoints: edge.endpoints(),
                sources,
                carrier,
                carrier_plane,
                direction,
            });
        }
        Some(Self {
            support: facet.support(),
            support_plane,
            normal: facet.normal(),
            vertices,
            coordinates,
            edges,
        })
    }

    fn vertex(&self, id: VertexId) -> Option<ProofVertex> {
        self.vertices
            .iter()
            .copied()
            .find(|candidate| candidate.id == id)
    }

    fn coordinates(&self, id: VertexId) -> Option<IntervalVec3> {
        self.vertices
            .iter()
            .position(|candidate| candidate.id == id)
            .map(|index| self.coordinates[index])
    }
}

fn vertex(evidence: &SemanticPlanarShellEvidence, id: VertexId) -> Option<SemanticVertexEvidence> {
    evidence.vertex(id)
}

fn strict_facet_side(
    facet: &ProofFacet,
    plane: IntervalPlane,
    excluded: &[VertexId],
) -> Option<i8> {
    let mut expected = None;
    for (vertex, &coordinates) in facet.vertices.iter().zip(&facet.coordinates) {
        if excluded.contains(&vertex.id) {
            continue;
        }
        let side = plane_value(plane, coordinates).sign()?;
        if side == 0 || expected.is_some_and(|value| value != side) {
            return None;
        }
        expected = Some(side);
    }
    expected
}

fn same_support_vertex_contact_is_authorized(
    left: &ProofFacet,
    right: &ProofFacet,
    shared: VertexId,
) -> bool {
    let left_edges: Vec<_> = left
        .edges
        .iter()
        .filter(|edge| edge.endpoints.contains(&shared))
        .collect();
    let right_edges: Vec<_> = right
        .edges
        .iter()
        .filter(|edge| edge.endpoints.contains(&shared))
        .collect();
    if left_edges.len() != 2
        || right_edges.len() != 2
        || left_edges[0].carrier == left_edges[1].carrier
    {
        return false;
    }
    let Some(vertex) = left.vertex(shared) else {
        return false;
    };
    if !vertex.surfaces.contains(&left.support)
        || left_edges
            .iter()
            .any(|edge| !vertex.surfaces.contains(&edge.carrier))
    {
        return false;
    }
    left_edges.into_iter().all(|left_edge| {
        let Some(right_edge) = right_edges
            .iter()
            .find(|right_edge| right_edge.carrier == left_edge.carrier)
        else {
            return false;
        };
        matches!(
            (
                strict_facet_side(left, left_edge.carrier_plane, &left_edge.endpoints),
                strict_facet_side(right, right_edge.carrier_plane, &right_edge.endpoints),
            ),
            (Some(-1), Some(1)) | (Some(1), Some(-1))
        )
    })
}

fn distinct_support_vertex_contact_is_authorized(
    shell: &SemanticPlanarShellEvidence,
    left: &ProofFacet,
    right: &ProofFacet,
    shared: VertexId,
) -> bool {
    if strict_facet_side(left, right.support_plane, &[shared]).is_some()
        && strict_facet_side(right, left.support_plane, &[shared]).is_some()
    {
        return true;
    }

    let (Some(left_edge), Some(right_edge)) = (
        unique_common_line_edge(left, shared, left.support, right.support),
        unique_common_line_edge(right, shared, left.support, right.support),
    ) else {
        return false;
    };
    if strict_facet_side(left, right.support_plane, &left_edge.endpoints).is_none()
        || strict_facet_side(right, left.support_plane, &right_edge.endpoints).is_none()
    {
        return false;
    }
    let Some(shared_vertex) = shell.vertex(shared) else {
        return false;
    };
    let Some(separator) = shared_vertex
        .surfaces()
        .into_iter()
        .find(|surface| *surface != left.support && *surface != right.support)
        .and_then(|surface| shell.plane_witness(surface))
        .and_then(plane_from_witness)
    else {
        return false;
    };
    let other_endpoint = |edge: &ProofEdge| {
        edge.endpoints
            .into_iter()
            .find(|endpoint| *endpoint != shared)
    };
    let (Some(left_point), Some(right_point)) = (
        other_endpoint(left_edge).and_then(|vertex| left.coordinates(vertex)),
        other_endpoint(right_edge).and_then(|vertex| right.coordinates(vertex)),
    ) else {
        return false;
    };
    matches!(
        (
            plane_value(separator, left_point).sign(),
            plane_value(separator, right_point).sign(),
        ),
        (Some(-1), Some(1)) | (Some(1), Some(-1))
    )
}

fn unique_common_line_edge(
    facet: &ProofFacet,
    shared: VertexId,
    first_support: SurfaceId,
    second_support: SurfaceId,
) -> Option<&ProofEdge> {
    let mut matching = facet.edges.iter().filter(|edge| {
        edge.endpoints.contains(&shared)
            && edge.sources.contains(&first_support)
            && edge.sources.contains(&second_support)
    });
    let edge = matching.next()?;
    matching.next().is_none().then_some(edge)
}

fn separating_axes(left: &ProofFacet, right: &ProofFacet) -> Vec<IntervalVec3> {
    let mut axes = Vec::new();
    axes.push(left.normal);
    axes.push(right.normal);
    for edge in &left.edges {
        axes.push(cross(left.normal, edge.direction));
    }
    for edge in &right.edges {
        axes.push(cross(right.normal, edge.direction));
    }
    for left_edge in &left.edges {
        for right_edge in &right.edges {
            axes.push(cross(left_edge.direction, right_edge.direction));
        }
    }
    for edge in &left.edges {
        axes.push(cross(edge.direction, right.normal));
    }
    for edge in &right.edges {
        axes.push(cross(left.normal, edge.direction));
    }
    axes.push(cross(left.normal, right.normal));
    axes
}

fn same_vertex_set(left: &[VertexId], right: &[VertexId]) -> bool {
    left.len() == right.len()
        && left.iter().all(|vertex| right.contains(vertex))
        && right.iter().all(|vertex| left.contains(vertex))
}

fn same_surface_pair(left: [SurfaceId; 2], right: [SurfaceId; 2]) -> bool {
    left == right || left == [right[1], right[0]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_pair_work_matches_independent_formula() {
        fn formula(m: u64, n: u64) -> u64 {
            let axes = 3 + 2 * m + 2 * n + m * n;
            axes * (m + n + 1) + 2 * m * n + m + n + 1
        }

        assert_eq!(pair_work_counts(3, 4), Some(formula(3, 4)));
        assert_eq!(pair_work_counts(4, 4), Some(formula(4, 4)));
        assert_eq!(formula(3, 4), 264);
        assert_eq!(formula(4, 4), 356);
    }

    #[test]
    fn unordered_surface_pairs_are_identity_based() {
        // The generic helper is exercised through equality laws here because
        // arena-owned SurfaceIds intentionally have no public raw constructor.
        fn law<T: Copy + Eq>(left: [T; 2], right: [T; 2]) -> bool {
            left == right || left == [right[1], right[0]]
        }
        assert!(law([1_u8, 2], [2, 1]));
        assert!(!law([1_u8, 2], [1, 3]));
    }
}
