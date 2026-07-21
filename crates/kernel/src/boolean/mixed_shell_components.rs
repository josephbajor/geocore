//! Deterministic physical-edge connected components of a mixed-shell plan.
//!
//! The proof plan deliberately keeps source-face-qualified symbolic edges.
//! Materialization coalesces those aliases by exact raw source edge and exact
//! endpoint identity.  This read-only pass consumes that physical incidence,
//! validates the closed two-manifold adjacency contract again, and partitions
//! faces without consulting geometry, layout names, or raw-handle ordering.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use super::materialize::{
    MixedShellMaterializationBlueprint, MixedShellMaterializationError, PhysicalCarrier,
    PhysicalEdge, PhysicalUse, PhysicalVertex, prepare_mixed_shell_materialization,
};
use super::{
    MixedShellEdgeKey, MixedShellFacePlan, MixedShellProofPlan, MixedShellVertexKey,
    MixedSourceFaceKey,
};
use crate::FaceId;
use ktopo::store::Store;

/// Stable symbolic occurrence used to order physical edges and components.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct MixedShellSymbolicMinimum {
    edge: MixedShellEdgeKey,
    face: usize,
    loop_index: usize,
    use_index: usize,
}

impl MixedShellSymbolicMinimum {
    pub(crate) const fn edge(&self) -> &MixedShellEdgeKey {
        &self.edge
    }

    pub(crate) const fn face(&self) -> usize {
        self.face
    }

    pub(crate) const fn loop_index(&self) -> usize {
        self.loop_index
    }

    pub(crate) const fn use_index(&self) -> usize {
        self.use_index
    }
}

/// Exact identity of one face retained in a component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellComponentFace {
    plan_index: usize,
    source: MixedSourceFaceKey,
    source_face: FaceId,
}

impl MixedShellComponentFace {
    pub(crate) const fn plan_index(&self) -> usize {
        self.plan_index
    }

    pub(crate) const fn source(&self) -> MixedSourceFaceKey {
        self.source
    }

    pub(crate) const fn source_face(&self) -> &FaceId {
        &self.source_face
    }
}

/// One exact face-local occurrence of a coalesced physical edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellComponentEdgeUse {
    face: usize,
    loop_index: usize,
    use_index: usize,
    forward: bool,
    symbolic_edge: MixedShellEdgeKey,
}

impl MixedShellComponentEdgeUse {
    pub(crate) const fn face(&self) -> usize {
        self.face
    }

    pub(crate) const fn loop_index(&self) -> usize {
        self.loop_index
    }

    pub(crate) const fn use_index(&self) -> usize {
        self.use_index
    }

    pub(crate) const fn forward(&self) -> bool {
        self.forward
    }

    pub(crate) const fn symbolic_edge(&self) -> &MixedShellEdgeKey {
        &self.symbolic_edge
    }
}

/// Complete exact identity and two face uses of one physical edge.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellComponentEdge {
    carrier: PhysicalCarrier,
    endpoints: Option<[PhysicalVertex; 2]>,
    uses: [MixedShellComponentEdgeUse; 2],
    symbolic_minimum: MixedShellSymbolicMinimum,
}

impl MixedShellComponentEdge {
    pub(crate) const fn carrier(&self) -> PhysicalCarrier {
        self.carrier
    }

    pub(crate) const fn endpoints(&self) -> Option<[PhysicalVertex; 2]> {
        self.endpoints
    }

    pub(crate) const fn uses(&self) -> &[MixedShellComponentEdgeUse; 2] {
        &self.uses
    }

    pub(crate) const fn symbolic_minimum(&self) -> &MixedShellSymbolicMinimum {
        &self.symbolic_minimum
    }
}

/// One physical vertex plus every exact symbolic alias in this component.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellComponentVertex {
    identity: PhysicalVertex,
    symbolic_aliases: Vec<MixedShellVertexKey>,
}

impl MixedShellComponentVertex {
    pub(crate) const fn identity(&self) -> PhysicalVertex {
        self.identity
    }

    pub(crate) fn symbolic_aliases(&self) -> &[MixedShellVertexKey] {
        &self.symbolic_aliases
    }
}

/// One closed face-adjacency component in deterministic symbolic order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct MixedShellComponent {
    symbolic_minimum: MixedShellSymbolicMinimum,
    faces: Vec<MixedShellComponentFace>,
    edges: Vec<MixedShellComponentEdge>,
    vertices: Vec<MixedShellComponentVertex>,
}

impl MixedShellComponent {
    pub(crate) const fn symbolic_minimum(&self) -> &MixedShellSymbolicMinimum {
        &self.symbolic_minimum
    }

    pub(crate) fn faces(&self) -> &[MixedShellComponentFace] {
        &self.faces
    }

    pub(crate) fn edges(&self) -> &[MixedShellComponentEdge] {
        &self.edges
    }

    pub(crate) fn vertices(&self) -> &[MixedShellComponentVertex] {
        &self.vertices
    }
}

/// Fail-closed physical component validation.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum MixedShellComponentError {
    EmptyPlan,
    EmptyFaceBoundary {
        face: usize,
    },
    OpenLoop {
        face: usize,
        loop_index: usize,
    },
    OpenBoundary {
        edge: usize,
        uses: usize,
    },
    NonManifoldBoundary {
        edge: usize,
        uses: usize,
    },
    SelfAdjacentEdge {
        edge: usize,
    },
    EdgeUsesNotOpposed {
        edge: usize,
    },
    UnknownFaceUse {
        edge: usize,
        face: usize,
    },
    InvalidUseLocation {
        edge: usize,
        face: usize,
        loop_index: usize,
        use_index: usize,
    },
    VertexIdentityMismatch(MixedShellVertexKey),
    PhysicalIncidence(MixedShellMaterializationError),
}

/// Conservative checked work ceiling for component validation and slicing.
///
/// `N = 1 + F + L + U + E` covers every face, loop, directed use, and
/// coalesced physical edge. `N²` dominates stable ordering and alias scans;
/// `8N` covers loop validation, adjacency, traversal, and closure retention.
pub(crate) fn mixed_shell_component_work(
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
) -> Option<u64> {
    let mut loops = 0_usize;
    let mut uses = 0_usize;
    for face in plan.faces() {
        loops = loops.checked_add(face.loops().len())?;
        for loop_ in face.loops() {
            uses = uses.checked_add(loop_.uses().len())?;
        }
    }
    let size = 1_u64
        .checked_add(u64::try_from(plan.faces().len()).ok()?)?
        .checked_add(u64::try_from(loops).ok()?)?
        .checked_add(u64::try_from(uses).ok()?)?
        .checked_add(u64::try_from(blueprint.edges().len()).ok()?)?;
    size.checked_mul(size)?.checked_add(size.checked_mul(8)?)
}

#[derive(Debug, Clone)]
struct ResolvedUse {
    physical: PhysicalUse,
    symbolic_edge: MixedShellEdgeKey,
    canonical_vertex_aliases: Option<[MixedShellVertexKey; 2]>,
}

#[derive(Debug, Clone)]
struct ResolvedEdge {
    carrier: PhysicalCarrier,
    endpoints: Option<[PhysicalVertex; 2]>,
    uses: Vec<ResolvedUse>,
    symbolic_minimum: MixedShellSymbolicMinimum,
}

#[derive(Debug, Clone, Copy)]
struct CoreUse {
    face: usize,
    forward: bool,
}

#[derive(Debug, Clone)]
struct CoreEdge {
    stable_ordinal: usize,
    uses: Vec<CoreUse>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CoreComponent {
    faces: Vec<usize>,
    edges: Vec<usize>,
    minimum_edge: usize,
}

/// Partition one proof plan by exact shared physical edges.
///
/// No topology is allocated and no scalar or geometric representative is
/// consulted. The returned face, edge, and vertex identities are complete
/// subsets of the proposal, ordered only by comparable symbolic plan keys.
pub(crate) fn partition_mixed_shell_components(
    plan: &MixedShellProofPlan,
    store: &Store,
) -> Result<Vec<MixedShellComponent>, MixedShellComponentError> {
    validate_symbolic_loops(plan)?;
    let blueprint =
        prepare_mixed_shell_materialization(plan, store).map_err(map_materialization_error)?;
    partition_prepared_mixed_shell_components(plan, &blueprint)
}

/// Partition a plan from one already-prepared, budget-charged physical view.
///
/// Preparing physical incidence can be charged once, then reused here and by
/// scalar completion without rescanning the plan or source topology.
pub(crate) fn partition_prepared_mixed_shell_components(
    plan: &MixedShellProofPlan,
    blueprint: &MixedShellMaterializationBlueprint,
) -> Result<Vec<MixedShellComponent>, MixedShellComponentError> {
    validate_symbolic_loops(plan)?;
    let mut edges = blueprint
        .edges()
        .iter()
        .enumerate()
        .map(|(index, edge)| resolve_edge(plan, index, edge))
        .collect::<Result<Vec<_>, _>>()?;
    edges.sort_by(|left, right| left.symbolic_minimum.cmp(&right.symbolic_minimum));

    let core_edges = edges
        .iter()
        .enumerate()
        .map(|(stable_ordinal, edge)| CoreEdge {
            stable_ordinal,
            uses: edge
                .uses
                .iter()
                .map(|use_| CoreUse {
                    face: use_.physical.face(),
                    forward: use_.physical.forward(),
                })
                .collect(),
        })
        .collect::<Vec<_>>();
    let memberships = partition_core(plan.faces().len(), &core_edges)?;
    validate_symbolic_vertex_identity(&edges)?;
    memberships
        .into_iter()
        .map(|membership| build_component(plan, &edges, membership))
        .collect()
}

fn validate_symbolic_loops(plan: &MixedShellProofPlan) -> Result<(), MixedShellComponentError> {
    if plan.faces().is_empty() {
        return Err(MixedShellComponentError::EmptyPlan);
    }
    for (face_index, face) in plan.faces().iter().enumerate() {
        if face.loops().is_empty() {
            return Err(MixedShellComponentError::EmptyFaceBoundary { face: face_index });
        }
        for (loop_index, loop_) in face.loops().iter().enumerate() {
            let vertices = loop_.vertices();
            if loop_.uses().is_empty()
                || vertices.len() != loop_.uses().len() + 1
                || vertices.first() != vertices.last()
            {
                return Err(MixedShellComponentError::OpenLoop {
                    face: face_index,
                    loop_index,
                });
            }
        }
    }
    Ok(())
}

fn map_materialization_error(error: MixedShellMaterializationError) -> MixedShellComponentError {
    match error {
        MixedShellMaterializationError::EdgeUseCount { edge, uses } if uses < 2 => {
            MixedShellComponentError::OpenBoundary { edge, uses }
        }
        MixedShellMaterializationError::EdgeUseCount { edge, uses } => {
            MixedShellComponentError::NonManifoldBoundary { edge, uses }
        }
        MixedShellMaterializationError::EdgeUsesNotOpposed(edge) => {
            MixedShellComponentError::EdgeUsesNotOpposed { edge }
        }
        MixedShellMaterializationError::SelfAdjacentEdge(edge) => {
            MixedShellComponentError::SelfAdjacentEdge { edge }
        }
        other => MixedShellComponentError::PhysicalIncidence(other),
    }
}

fn resolve_edge(
    plan: &MixedShellProofPlan,
    edge_index: usize,
    edge: &PhysicalEdge,
) -> Result<ResolvedEdge, MixedShellComponentError> {
    let mut uses = edge
        .uses()
        .iter()
        .copied()
        .map(|physical| resolve_use(plan, edge_index, edge.endpoints(), physical))
        .collect::<Result<Vec<_>, _>>()?;
    uses.sort_by_key(|use_| {
        (
            use_.physical.face(),
            use_.physical.loop_index(),
            use_.physical.use_index(),
        )
    });
    let symbolic_minimum =
        uses.iter()
            .map(symbolic_minimum)
            .min()
            .ok_or(MixedShellComponentError::OpenBoundary {
                edge: edge_index,
                uses: 0,
            })?;
    Ok(ResolvedEdge {
        carrier: edge.carrier(),
        endpoints: edge.endpoints(),
        uses,
        symbolic_minimum,
    })
}

fn resolve_use(
    plan: &MixedShellProofPlan,
    edge_index: usize,
    endpoints: Option<[PhysicalVertex; 2]>,
    physical: PhysicalUse,
) -> Result<ResolvedUse, MixedShellComponentError> {
    let Some(face) = plan.faces().get(physical.face()) else {
        return Err(MixedShellComponentError::UnknownFaceUse {
            edge: edge_index,
            face: physical.face(),
        });
    };
    let Some(loop_) = face.loops().get(physical.loop_index()) else {
        return Err(invalid_location(edge_index, physical));
    };
    let Some(use_) = loop_.uses().get(physical.use_index()) else {
        return Err(invalid_location(edge_index, physical));
    };
    let Some(tail) = loop_.vertices().get(physical.use_index()).cloned() else {
        return Err(invalid_location(edge_index, physical));
    };
    let Some(head) = loop_.vertices().get(physical.use_index() + 1).cloned() else {
        return Err(invalid_location(edge_index, physical));
    };
    let canonical_vertex_aliases = endpoints.map(|_| {
        if physical.forward() {
            [tail, head]
        } else {
            [head, tail]
        }
    });
    Ok(ResolvedUse {
        physical,
        symbolic_edge: use_.edge().clone(),
        canonical_vertex_aliases,
    })
}

const fn invalid_location(edge: usize, use_: PhysicalUse) -> MixedShellComponentError {
    MixedShellComponentError::InvalidUseLocation {
        edge,
        face: use_.face(),
        loop_index: use_.loop_index(),
        use_index: use_.use_index(),
    }
}

fn symbolic_minimum(use_: &ResolvedUse) -> MixedShellSymbolicMinimum {
    MixedShellSymbolicMinimum {
        edge: use_.symbolic_edge.clone(),
        face: use_.physical.face(),
        loop_index: use_.physical.loop_index(),
        use_index: use_.physical.use_index(),
    }
}

fn partition_core(
    face_count: usize,
    edges: &[CoreEdge],
) -> Result<Vec<CoreComponent>, MixedShellComponentError> {
    if face_count == 0 {
        return Err(MixedShellComponentError::EmptyPlan);
    }
    let mut adjacency = vec![BTreeSet::new(); face_count];
    for edge in edges {
        if edge.uses.len() < 2 {
            return Err(MixedShellComponentError::OpenBoundary {
                edge: edge.stable_ordinal,
                uses: edge.uses.len(),
            });
        }
        if edge.uses.len() > 2 {
            return Err(MixedShellComponentError::NonManifoldBoundary {
                edge: edge.stable_ordinal,
                uses: edge.uses.len(),
            });
        }
        let first = edge.uses[0];
        let second = edge.uses[1];
        for use_ in [first, second] {
            if use_.face >= face_count {
                return Err(MixedShellComponentError::UnknownFaceUse {
                    edge: edge.stable_ordinal,
                    face: use_.face,
                });
            }
        }
        if first.face == second.face {
            return Err(MixedShellComponentError::SelfAdjacentEdge {
                edge: edge.stable_ordinal,
            });
        }
        if first.forward == second.forward {
            return Err(MixedShellComponentError::EdgeUsesNotOpposed {
                edge: edge.stable_ordinal,
            });
        }
        adjacency[first.face].insert(second.face);
        adjacency[second.face].insert(first.face);
    }
    if let Some(face) = adjacency.iter().position(BTreeSet::is_empty) {
        return Err(MixedShellComponentError::EmptyFaceBoundary { face });
    }

    let mut visited = vec![false; face_count];
    let mut output = Vec::new();
    for start in 0..face_count {
        if visited[start] {
            continue;
        }
        let mut faces = Vec::new();
        let mut queue = VecDeque::from([start]);
        while let Some(face) = queue.pop_front() {
            if visited[face] {
                continue;
            }
            visited[face] = true;
            faces.push(face);
            queue.extend(adjacency[face].iter().copied());
        }
        faces.sort_unstable();
        let membership = faces.iter().copied().collect::<BTreeSet<_>>();
        let mut component_edges = edges
            .iter()
            .filter(|edge| edge.uses.iter().all(|use_| membership.contains(&use_.face)))
            .map(|edge| edge.stable_ordinal)
            .collect::<Vec<_>>();
        component_edges.sort_unstable();
        let minimum_edge = *component_edges
            .first()
            .ok_or(MixedShellComponentError::EmptyFaceBoundary { face: start })?;
        output.push(CoreComponent {
            faces,
            edges: component_edges,
            minimum_edge,
        });
    }
    output.sort_by_key(|component| component.minimum_edge);
    Ok(output)
}

fn validate_symbolic_vertex_identity(
    edges: &[ResolvedEdge],
) -> Result<(), MixedShellComponentError> {
    let mut aliases = BTreeMap::<MixedShellVertexKey, PhysicalVertex>::new();
    for edge in edges {
        let Some(endpoints) = edge.endpoints else {
            continue;
        };
        for use_ in &edge.uses {
            let aliases_for_use = use_.canonical_vertex_aliases.as_ref().ok_or_else(|| {
                MixedShellComponentError::InvalidUseLocation {
                    edge: 0,
                    face: use_.physical.face(),
                    loop_index: use_.physical.loop_index(),
                    use_index: use_.physical.use_index(),
                }
            })?;
            for (identity, alias) in endpoints.into_iter().zip(aliases_for_use.iter().cloned()) {
                if aliases
                    .insert(alias.clone(), identity)
                    .is_some_and(|existing| existing != identity)
                {
                    return Err(MixedShellComponentError::VertexIdentityMismatch(alias));
                }
            }
        }
    }
    Ok(())
}

fn build_component(
    plan: &MixedShellProofPlan,
    edges: &[ResolvedEdge],
    membership: CoreComponent,
) -> Result<MixedShellComponent, MixedShellComponentError> {
    let faces = membership
        .faces
        .iter()
        .map(|&plan_index| component_face(plan_index, &plan.faces()[plan_index]))
        .collect();
    let component_edges = membership
        .edges
        .iter()
        .map(|&edge_index| component_edge(edge_index, &edges[edge_index]))
        .collect::<Result<Vec<_>, _>>()?;
    let vertices = component_vertices(
        membership
            .edges
            .iter()
            .map(|&edge_index| &edges[edge_index]),
    );
    Ok(MixedShellComponent {
        symbolic_minimum: edges[membership.minimum_edge].symbolic_minimum.clone(),
        faces,
        edges: component_edges,
        vertices,
    })
}

fn component_face(plan_index: usize, face: &MixedShellFacePlan) -> MixedShellComponentFace {
    MixedShellComponentFace {
        plan_index,
        source: face.source(),
        source_face: face.source_face().clone(),
    }
}

fn component_edge(
    edge_index: usize,
    edge: &ResolvedEdge,
) -> Result<MixedShellComponentEdge, MixedShellComponentError> {
    let uses: [MixedShellComponentEdgeUse; 2] = edge
        .uses
        .iter()
        .map(|use_| MixedShellComponentEdgeUse {
            face: use_.physical.face(),
            loop_index: use_.physical.loop_index(),
            use_index: use_.physical.use_index(),
            forward: use_.physical.forward(),
            symbolic_edge: use_.symbolic_edge.clone(),
        })
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|uses: Vec<_>| {
            if uses.len() < 2 {
                MixedShellComponentError::OpenBoundary {
                    edge: edge_index,
                    uses: uses.len(),
                }
            } else {
                MixedShellComponentError::NonManifoldBoundary {
                    edge: edge_index,
                    uses: uses.len(),
                }
            }
        })?;
    Ok(MixedShellComponentEdge {
        carrier: edge.carrier,
        endpoints: edge.endpoints,
        uses,
        symbolic_minimum: edge.symbolic_minimum.clone(),
    })
}

fn component_vertices<'a>(
    edges: impl IntoIterator<Item = &'a ResolvedEdge>,
) -> Vec<MixedShellComponentVertex> {
    let mut vertices = Vec::<MixedShellComponentVertex>::new();
    for edge in edges {
        let Some(endpoints) = edge.endpoints else {
            continue;
        };
        for endpoint in 0..2 {
            let identity = endpoints[endpoint];
            let aliases = edge.uses.iter().filter_map(|use_| {
                use_.canonical_vertex_aliases
                    .as_ref()
                    .map(|aliases| aliases[endpoint].clone())
            });
            if let Some(vertex) = vertices
                .iter_mut()
                .find(|vertex| vertex.identity == identity)
            {
                vertex.symbolic_aliases.extend(aliases);
            } else {
                vertices.push(MixedShellComponentVertex {
                    identity,
                    symbolic_aliases: aliases.collect(),
                });
            }
        }
    }
    for vertex in &mut vertices {
        vertex.symbolic_aliases.sort();
        vertex.symbolic_aliases.dedup();
    }
    vertices.sort_by(|left, right| left.symbolic_aliases[0].cmp(&right.symbolic_aliases[0]));
    vertices
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(stable_ordinal: usize, uses: &[(usize, bool)]) -> CoreEdge {
        CoreEdge {
            stable_ordinal,
            uses: uses
                .iter()
                .map(|&(face, forward)| CoreUse { face, forward })
                .collect(),
        }
    }

    #[test]
    fn adversarial_physical_boundaries_fail_closed_with_typed_errors() {
        struct Case {
            name: &'static str,
            face_count: usize,
            edges: Vec<CoreEdge>,
            expected: MixedShellComponentError,
        }
        let cases = [
            Case {
                name: "empty",
                face_count: 0,
                edges: vec![],
                expected: MixedShellComponentError::EmptyPlan,
            },
            Case {
                name: "open",
                face_count: 2,
                edges: vec![edge(4, &[(0, true)])],
                expected: MixedShellComponentError::OpenBoundary { edge: 4, uses: 1 },
            },
            Case {
                name: "nonmanifold",
                face_count: 3,
                edges: vec![edge(7, &[(0, true), (1, false), (2, true)])],
                expected: MixedShellComponentError::NonManifoldBoundary { edge: 7, uses: 3 },
            },
            Case {
                name: "self_adjacent",
                face_count: 1,
                edges: vec![edge(2, &[(0, true), (0, false)])],
                expected: MixedShellComponentError::SelfAdjacentEdge { edge: 2 },
            },
            Case {
                name: "same_direction",
                face_count: 2,
                edges: vec![edge(3, &[(0, true), (1, true)])],
                expected: MixedShellComponentError::EdgeUsesNotOpposed { edge: 3 },
            },
            Case {
                name: "unknown_face",
                face_count: 2,
                edges: vec![edge(5, &[(0, true), (2, false)])],
                expected: MixedShellComponentError::UnknownFaceUse { edge: 5, face: 2 },
            },
            Case {
                name: "isolated_face",
                face_count: 3,
                edges: vec![edge(0, &[(0, true), (1, false)])],
                expected: MixedShellComponentError::EmptyFaceBoundary { face: 2 },
            },
        ];
        for case in cases {
            assert_eq!(
                partition_core(case.face_count, &case.edges),
                Err(case.expected),
                "{}",
                case.name
            );
        }
    }

    #[test]
    fn components_and_membership_are_ordered_by_stable_symbolic_edge_minimum() {
        let expected = vec![
            CoreComponent {
                faces: vec![0, 1],
                edges: vec![1, 4],
                minimum_edge: 1,
            },
            CoreComponent {
                faces: vec![2, 3],
                edges: vec![7, 9],
                minimum_edge: 7,
            },
        ];
        let first = vec![
            edge(9, &[(2, false), (3, true)]),
            edge(4, &[(1, true), (0, false)]),
            edge(7, &[(3, false), (2, true)]),
            edge(1, &[(0, true), (1, false)]),
        ];
        let mut reversed = first.clone();
        reversed.reverse();
        assert_eq!(partition_core(4, &first).unwrap(), expected);
        assert_eq!(partition_core(4, &reversed).unwrap(), expected);
    }
}
