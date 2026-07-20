//! Exact shell winding and convex-containment proposals.
//!
//! This stage uses only symbolic face rings, exact source-plane side evidence,
//! and interval enclosures of ideal plane-triple vertices. It proposes body
//! ownership for allocation; persisted semantic Full checking remains the
//! authority for shell embedding and region containment.

use std::collections::{BTreeMap, BTreeSet};

use kcore::interval::Interval;
use ktopo::planar::PlanarVertexKey;

use super::components::SelectedShellComponent;
use super::planar_bsp::{PlaneTripleVertexKey, SourcePlane, SourcePlaneRef};
use super::realize::RealizedPlaneTriple;
use super::select::SelectedPlanarFragment;

/// Honest refusal from exact shell winding and containment proposal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ComponentLayoutError {
    /// Distinct edge-components share a symbolic vertex, which is contact.
    SharedVertex,
    /// The exact plane-triple interval volume does not exclude zero.
    IndeterminateWinding,
    /// No positive shell was available to bound material.
    MissingPositiveShell,
    /// A negative shell did not have one uniquely certified convex container.
    UncertifiedContainment,
    /// More than one cavity targeted a body beyond the current checker slice.
    MultipleCavities,
}

/// One stable topology key paired with exact ideal-vertex evidence.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct RealizedPlanarVertex {
    key: PlanarVertexKey,
    evidence: RealizedPlaneTriple,
}

impl RealizedPlanarVertex {
    pub(crate) const fn new(key: PlanarVertexKey, evidence: RealizedPlaneTriple) -> Self {
        Self { key, evidence }
    }

    pub(crate) const fn key(&self) -> PlanarVertexKey {
        self.key
    }

    pub(crate) const fn evidence(&self) -> &RealizedPlaneTriple {
        &self.evidence
    }
}

pub(crate) type RealizedVertexMap = BTreeMap<PlaneTripleVertexKey, RealizedPlanarVertex>;

/// Exact selected-size upper bound charged before component work starts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PostSelectionPrecharge {
    pub(crate) work: u64,
    pub(crate) ring_uses: u64,
}

/// One connected material-body proposal in stable outer-component order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ComponentBodyProposal {
    outer: SelectedShellComponent,
    cavity: Option<SelectedShellComponent>,
}

impl ComponentBodyProposal {
    pub(crate) fn into_parts(self) -> (SelectedShellComponent, Option<SelectedShellComponent>) {
        (self.outer, self.cavity)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ComponentWinding {
    Positive,
    Negative,
}

/// Precharge symbolic-edge visits, realization, winding, and worst-case
/// convex-containment comparisons using only selected boundary sizes.
pub(crate) fn post_selection_precharge(
    selected: &[SelectedPlanarFragment],
    plane_count: usize,
) -> Option<PostSelectionPrecharge> {
    let face_count = u64::try_from(selected.len()).ok()?;
    let plane_count = u64::try_from(plane_count).ok()?;
    let mut ring_uses = 0_u64;
    let mut triangles = 0_u64;
    for face in selected {
        let vertices = u64::try_from(face.fragment().vertices().len()).ok()?;
        ring_uses = ring_uses.checked_add(vertices)?;
        triangles = triangles.checked_add(vertices.checked_sub(2)?)?;
    }

    // Per ring use: all-plane realization, symbolic edge insertion/check,
    // shared-vertex scan, component traversal, and every positive/negative
    // component-pair vertex-by-support comparison. There are at most
    // `face_count` components and support faces per positive component.
    let containment = face_count.checked_mul(face_count)?;
    let per_ring = plane_count
        .checked_add(containment)?
        .checked_add(face_count)?
        .checked_add(6)?;
    let work = ring_uses
        .checked_mul(per_ring)?
        .checked_add(face_count.checked_mul(2)?)?
        .checked_add(triangles)?;
    Some(PostSelectionPrecharge { work, ring_uses })
}

/// Refuse point-contact between edge-disconnected shells before realization.
pub(crate) fn validate_disjoint_component_vertices(
    components: &[SelectedShellComponent],
) -> Result<(), ComponentLayoutError> {
    let mut owners = BTreeMap::new();
    for (component, shell) in components.iter().enumerate() {
        for vertex in component_vertices(shell) {
            if owners.insert(vertex, component).is_some() {
                return Err(ComponentLayoutError::SharedVertex);
            }
        }
    }
    Ok(())
}

/// Classify exact shell winding and assign each negative shell to one unique
/// certified-convex positive container.
pub(crate) fn propose_component_bodies(
    components: Vec<SelectedShellComponent>,
    realized: &RealizedVertexMap,
    planes: &[SourcePlane],
) -> Result<Vec<ComponentBodyProposal>, ComponentLayoutError> {
    let mut positives = Vec::new();
    let mut negatives = Vec::new();
    for component in components {
        match component_winding(&component, realized)? {
            ComponentWinding::Positive => positives.push(component),
            ComponentWinding::Negative => negatives.push(component),
        }
    }
    if positives.is_empty() {
        return Err(ComponentLayoutError::MissingPositiveShell);
    }

    let mut cavities = (0..positives.len()).map(|_| None).collect::<Vec<_>>();
    for negative in negatives {
        let owners = positives
            .iter()
            .enumerate()
            .filter_map(|(index, positive)| {
                certified_convex_contains(positive, &negative, realized, planes).then_some(index)
            })
            .collect::<Vec<_>>();
        let [owner] = owners.as_slice() else {
            return Err(ComponentLayoutError::UncertifiedContainment);
        };
        if cavities[*owner].replace(negative).is_some() {
            return Err(ComponentLayoutError::MultipleCavities);
        }
    }

    Ok(positives
        .into_iter()
        .zip(cavities)
        .map(|(outer, cavity)| ComponentBodyProposal { outer, cavity })
        .collect())
}

fn component_winding(
    component: &SelectedShellComponent,
    realized: &RealizedVertexMap,
) -> Result<ComponentWinding, ComponentLayoutError> {
    let reference = component_vertices(component)
        .into_iter()
        .next()
        .and_then(|vertex| realized.get(&vertex))
        .map(RealizedPlanarVertex::evidence)
        .map(RealizedPlaneTriple::coordinates)
        .ok_or(ComponentLayoutError::IndeterminateWinding)?;
    let mut six_volume = Interval::point(0.0);
    for face in component.faces() {
        let ring = face.oriented_vertices();
        let [first, rest @ ..] = ring.as_slice() else {
            return Err(ComponentLayoutError::IndeterminateWinding);
        };
        if rest.len() < 2 {
            return Err(ComponentLayoutError::IndeterminateWinding);
        }
        let origin = coordinates(realized, *first)
            .map(|point| subtract(point, reference))
            .ok_or(ComponentLayoutError::IndeterminateWinding)?;
        for index in 0..rest.len() - 1 {
            let second = coordinates(realized, rest[index])
                .map(|point| subtract(point, reference))
                .ok_or(ComponentLayoutError::IndeterminateWinding)?;
            let third = coordinates(realized, rest[index + 1])
                .map(|point| subtract(point, reference))
                .ok_or(ComponentLayoutError::IndeterminateWinding)?;
            six_volume = six_volume + determinant(origin, second, third);
            if !finite(six_volume) {
                return Err(ComponentLayoutError::IndeterminateWinding);
            }
        }
    }
    if six_volume.lo() > 0.0 {
        Ok(ComponentWinding::Positive)
    } else if six_volume.hi() < 0.0 {
        Ok(ComponentWinding::Negative)
    } else {
        Err(ComponentLayoutError::IndeterminateWinding)
    }
}

fn certified_convex_contains(
    positive: &SelectedShellComponent,
    negative: &SelectedShellComponent,
    realized: &RealizedVertexMap,
    planes: &[SourcePlane],
) -> bool {
    let supports = positive
        .faces()
        .iter()
        .map(|face| face.fragment().source_face())
        .collect::<BTreeSet<_>>();
    let positive_vertices = component_vertices(positive);
    let negative_vertices = component_vertices(negative);

    // First prove the proposed outer itself belongs to every retained source
    // halfspace. A mixed non-convex Boolean boundary therefore cannot become
    // containment authority merely because another shell lies nearby.
    for vertex in positive_vertices {
        for &support in &supports {
            if !vertex.planes().contains(&support)
                && !certified_interior_side(realized, planes, vertex, support)
            {
                return false;
            }
        }
    }
    for vertex in negative_vertices {
        for &support in &supports {
            if vertex.planes().contains(&support)
                || !certified_interior_side(realized, planes, vertex, support)
            {
                return false;
            }
        }
    }
    true
}

fn certified_interior_side(
    realized: &RealizedVertexMap,
    planes: &[SourcePlane],
    vertex: PlaneTripleVertexKey,
    support: SourcePlaneRef,
) -> bool {
    let mut sources = planes.iter().copied().filter(|plane| plane.id() == support);
    let Some(source) = sources.next() else {
        return false;
    };
    if sources.next().is_some() {
        return false;
    }
    realized.get(&vertex).is_some_and(|vertex| {
        vertex
            .evidence()
            .certified_sides()
            .iter()
            .find(|side| side.plane() == support)
            .is_some_and(|side| side.side() == source.interior_side())
    })
}

fn component_vertices(component: &SelectedShellComponent) -> BTreeSet<PlaneTripleVertexKey> {
    component
        .faces()
        .iter()
        .flat_map(|face| face.fragment().vertices().iter().copied())
        .collect()
}

fn coordinates(
    realized: &RealizedVertexMap,
    vertex: PlaneTripleVertexKey,
) -> Option<[Interval; 3]> {
    realized
        .get(&vertex)
        .map(RealizedPlanarVertex::evidence)
        .map(RealizedPlaneTriple::coordinates)
}

fn subtract(left: [Interval; 3], right: [Interval; 3]) -> [Interval; 3] {
    core::array::from_fn(|axis| left[axis] - right[axis])
}

fn determinant(a: [Interval; 3], b: [Interval; 3], c: [Interval; 3]) -> Interval {
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
}

fn finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}
