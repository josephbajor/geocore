//! Deterministic stitching of certified fragments from closed section carriers.
//!
//! A whole-period carrier that survives trimming intact is represented by
//! [`ClosedFragmentSpan::Whole`]: it is already a closed chain and has no
//! physical endpoint.  A carrier cut at its parameter seam or by exact trim
//! events is represented by directed [`ClosedFragmentSpan::Arc`] values.
//! Arc endpoints are interned only by proof-owned identities:
//!
//! - a trim site combines exact operand topology with certified source-edge
//!   root identities, or
//! - an intentional period seam combines the source branch and its graph-owned
//!   seam-site index.
//!
//! Numeric points never participate in a join.  Conservative source-edge
//! parameter intervals are intersected only after the proof identities match;
//! disjoint evidence is a typed defect.  The stitcher never reverses a
//! fragment, searches alternative configurations, or repairs ambiguous
//! incidence.  Such input remains inspectable partial evidence with
//! [`ClosedStitchCompletion::Indeterminate`].
//!
//! The narrow facade seam is [`ClosedBranchSource::from_section_branch`].  An
//! exact curved trim clipper can derive source provenance and the intentional
//! seam identity from an existing [`super::SectionBranch`], then emit `Whole`
//! or `Arc` fragments.  This adapter deliberately does not claim that the
//! untrimmed branch lies inside either trimmed face.

use std::collections::HashMap;
use std::collections::hash_map::Entry;

use kcore::interval::Interval;
use ktopo::entity::{EdgeId as RawEdgeId, FaceId as RawFaceId};

use super::stitch::{SiteKey, VertexKey};
use super::{SectionBranch, SectionBranchTopology};

/// Deterministic identity of a closed branch in one section operation.
///
/// The value is the branch's index in `BodySectionGraph::branches()`; it is
/// operation-local and is never persisted as topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ClosedBranchKey(usize);

impl ClosedBranchKey {
    pub(crate) const fn new(index: usize) -> Self {
        Self(index)
    }

    pub(crate) const fn index(self) -> usize {
        self.0
    }
}

/// Source carrier and face-pair provenance shared by fragments of one branch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClosedBranchSource {
    pub(crate) branch: ClosedBranchKey,
    pub(crate) faces: [RawFaceId; 2],
    period_seam_site: usize,
}

impl ClosedBranchSource {
    /// Retain topology-free source metadata from a verified closed branch.
    ///
    /// This checks only the structural period-seam representation.  Exact
    /// trimming must separately prove whether it emits a whole fragment or
    /// bounded arcs.
    pub(crate) fn from_section_branch(branch_index: usize, branch: &SectionBranch) -> Option<Self> {
        let endpoints = branch.endpoint_sites();
        if branch.topology() != SectionBranchTopology::Closed
            || endpoints[0] != endpoints[1]
            || endpoints[0] >= branch.fragment_sites().len()
        {
            return None;
        }
        Some(Self {
            branch: ClosedBranchKey::new(branch_index),
            faces: [branch.faces()[0].raw(), branch.faces()[1].raw()],
            period_seam_site: endpoints[0],
        })
    }

    /// Exact endpoint identity for the graph-owned parameter seam.
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) const fn period_seam(self) -> CertifiedClosedEndpoint {
        CertifiedClosedEndpoint {
            key: CertifiedClosedEndpointKey::PeriodSeam {
                branch: self.branch,
                site: self.period_seam_site,
            },
            edge_parameters: [None, None],
        }
    }

    /// Attach one clipper-owned deterministic ordinal to a source fragment.
    pub(crate) const fn fragment(self, ordinal: usize) -> ClosedFragmentSource {
        ClosedFragmentSource {
            branch: self.branch,
            faces: self.faces,
            ordinal,
            period_seam_site: self.period_seam_site,
        }
    }
}

/// Source provenance retained on every stitched fragment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClosedFragmentSource {
    /// Source branch in the section operation.
    pub(crate) branch: ClosedBranchKey,
    /// Carrier faces in operand order.
    pub(crate) faces: [RawFaceId; 2],
    /// Deterministic fragment ordinal assigned by the exact clipper.
    pub(crate) ordinal: usize,
    /// Graph-owned parameter-seam site inherited from the source branch.
    period_seam_site: usize,
}

/// Direction of a fragment relative to its source carrier.
///
/// `start` and `end` always follow the actual loop traversal.  This marker
/// retains whether that traversal agrees with the source carrier; the
/// stitcher never changes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) enum ClosedFragmentOrientation {
    AlongCarrier,
    AgainstCarrier,
}

/// Certified identity of one root on a source topology edge.
///
/// `root_ordinal` is assigned only after the curved clipper has certifiably
/// ordered the isolated roots along the source edge.  It distinguishes, for
/// example, two intersections of one circle with the same edge without
/// comparing floating-point representatives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct CertifiedSourceParameterKey {
    edge: RawEdgeId,
    root_ordinal: usize,
}

impl CertifiedSourceParameterKey {
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) const fn new(edge: RawEdgeId, root_ordinal: usize) -> Self {
        Self { edge, root_ordinal }
    }

    pub(crate) const fn edge(self) -> RawEdgeId {
        self.edge
    }

    pub(crate) const fn root_ordinal(self) -> usize {
        self.root_ordinal
    }
}

/// Proof-owned endpoint identity.  Equality is the only admission path for a
/// graph join.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) enum CertifiedClosedEndpointKey {
    /// An exact trimmed-face event.  Slots contain root identities exactly
    /// where the corresponding site is an edge interior.
    TrimSite {
        site: VertexKey,
        edge_parameter_keys: [Option<CertifiedSourceParameterKey>; 2],
    },
    /// The intentional chart cut of a complete-period source branch.
    PeriodSeam {
        branch: ClosedBranchKey,
        site: usize,
    },
}

/// One directed fragment endpoint and its metric consistency evidence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedClosedEndpoint {
    pub(crate) key: CertifiedClosedEndpointKey,
    /// Conservative intrinsic source-edge parameter enclosures in operand
    /// order.  Slots are present exactly for trim sites on edge interiors.
    pub(crate) edge_parameters: [Option<Interval>; 2],
}

impl CertifiedClosedEndpoint {
    #[cfg_attr(not(test), allow(dead_code))]
    pub(crate) const fn trim_site(
        site: VertexKey,
        edge_parameter_keys: [Option<CertifiedSourceParameterKey>; 2],
        edge_parameters: [Option<Interval>; 2],
    ) -> Self {
        Self {
            key: CertifiedClosedEndpointKey::TrimSite {
                site,
                edge_parameter_keys,
            },
            edge_parameters,
        }
    }
}

/// Coverage of one certified curved fragment.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::large_enum_variant)]
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) enum ClosedFragmentSpan {
    /// The complete closed carrier survived exact trimming.  It has no
    /// physical or artificial endpoint.
    Whole,
    /// A directed arc bounded by exact trim events and/or an intentional
    /// parameter seam.
    Arc {
        start: CertifiedClosedEndpoint,
        end: CertifiedClosedEndpoint,
    },
}

/// One proof-bearing curved fragment awaiting combinatorial stitching.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClosedCurveFragment {
    pub(crate) source: ClosedFragmentSource,
    pub(crate) orientation: ClosedFragmentOrientation,
    pub(crate) span: ClosedFragmentSpan,
}

/// One exact stitched vertex.  No representative point is needed or used.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ClosedStitchVertex {
    pub(crate) key: CertifiedClosedEndpointKey,
    pub(crate) edge_parameters: [Option<Interval>; 2],
    pub(crate) incoming: usize,
    pub(crate) outgoing: usize,
    pub(crate) edge_parameters_compatible: bool,
}

/// One fragment occurrence in cyclic traversal order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ClosedStitchedFragment {
    /// Index into the caller's fragment sequence.
    pub(crate) input_fragment: usize,
    pub(crate) source: ClosedFragmentSource,
    pub(crate) orientation: ClosedFragmentOrientation,
    /// Vertex indices in traversal order.  `None` is reserved for a whole
    /// closed carrier, which has no endpoints by construction.
    pub(crate) endpoints: Option<[usize; 2]>,
}

/// One maximal directed chain discovered from the lowest unused input index.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ClosedStitchChain {
    pub(crate) fragments: Vec<ClosedStitchedFragment>,
    pub(crate) closed: bool,
}

/// Which endpoint failed its proof-shape validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClosedFragmentEnd {
    Start,
    End,
}

/// Structural or proof-consistency defects.  None are repaired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClosedStitchDefect {
    /// One branch key was paired with different source faces.
    InconsistentBranchFaces {
        first_fragment: usize,
        fragment: usize,
    },
    /// Fragments of one source branch disagree on canonical orientation.
    InconsistentBranchOrientation {
        first_fragment: usize,
        fragment: usize,
    },
    /// Two inputs claim the same branch-local fragment ordinal.
    DuplicateSourceOrdinal {
        first_fragment: usize,
        fragment: usize,
    },
    /// One branch contains both an endpoint-free whole fragment and arcs.
    MixedWholeAndArc {
        first_fragment: usize,
        fragment: usize,
    },
    /// One source branch supplied more than one whole fragment.
    DuplicateWholeFragment {
        first_fragment: usize,
        fragment: usize,
    },
    /// Endpoint key, source, or parameter evidence is malformed.
    InvalidEndpointEvidence {
        fragment: usize,
        end: ClosedFragmentEnd,
    },
    /// Matching exact endpoint identities carried disjoint source-parameter
    /// enclosures (index into `ClosedStitchResult::vertices`).
    IncompatibleEndpointParameter(usize),
    /// A vertex has other than one arriving directed fragment.
    IncomingDegree { vertex: usize, degree: usize },
    /// A vertex has other than one leaving directed fragment.
    OutgoingDegree { vertex: usize, degree: usize },
    /// A maximal directed chain did not return to its origin.
    OpenChain(usize),
}

/// Whether every supplied fragment was certified into closed directed loops.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ClosedStitchCompletion {
    Complete,
    Indeterminate,
}

/// Deterministic closed-fragment stitch evidence.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ClosedStitchResult {
    pub(crate) vertices: Vec<ClosedStitchVertex>,
    pub(crate) chains: Vec<ClosedStitchChain>,
    pub(crate) defects: Vec<ClosedStitchDefect>,
    pub(crate) completion: ClosedStitchCompletion,
}

#[derive(Debug, Clone, Copy)]
struct BranchState {
    source: ClosedFragmentSource,
    first_fragment: usize,
    orientation: ClosedFragmentOrientation,
    first_whole: Option<usize>,
    first_arc: Option<usize>,
    mixed_reported: bool,
}

/// Stitch certified whole-period and seam-split curved fragments.
///
/// Input order is the deterministic authority: vertices number in endpoint
/// first-appearance order, each chain starts at the lowest unused fragment,
/// and traversal always follows stored `start -> end` orientation.  The
/// algorithm is one directed-incidence walk independent of geometric family
/// and fragment count; no configuration layouts are enumerated.
pub(crate) fn stitch_closed_fragments(fragments: &[ClosedCurveFragment]) -> ClosedStitchResult {
    let mut defects = Vec::new();
    validate_sources(fragments, &mut defects);

    let mut vertices = Vec::new();
    let mut vertex_index = HashMap::new();
    let mut incoming: Vec<Vec<usize>> = Vec::new();
    let mut outgoing: Vec<Vec<usize>> = Vec::new();
    let mut endpoints = vec![None; fragments.len()];
    let mut endpoint_valid = vec![true; fragments.len()];

    for (fragment_index, fragment) in fragments.iter().enumerate() {
        let ClosedFragmentSpan::Arc { start, end } = fragment.span else {
            continue;
        };
        let start_valid = endpoint_is_valid(start, fragment.source);
        let end_valid = endpoint_is_valid(end, fragment.source);
        if !start_valid {
            defects.push(ClosedStitchDefect::InvalidEndpointEvidence {
                fragment: fragment_index,
                end: ClosedFragmentEnd::Start,
            });
        }
        if !end_valid {
            defects.push(ClosedStitchDefect::InvalidEndpointEvidence {
                fragment: fragment_index,
                end: ClosedFragmentEnd::End,
            });
        }
        if !(start_valid && end_valid) {
            endpoint_valid[fragment_index] = false;
            continue;
        }

        let start_index = intern_endpoint(
            &mut vertices,
            &mut vertex_index,
            &mut incoming,
            &mut outgoing,
            start,
        );
        let end_index = intern_endpoint(
            &mut vertices,
            &mut vertex_index,
            &mut incoming,
            &mut outgoing,
            end,
        );
        outgoing[start_index].push(fragment_index);
        incoming[end_index].push(fragment_index);
        vertices[start_index].outgoing += 1;
        vertices[end_index].incoming += 1;
        endpoints[fragment_index] = Some([start_index, end_index]);
    }

    for (vertex, evidence) in vertices.iter().enumerate() {
        if !evidence.edge_parameters_compatible {
            defects.push(ClosedStitchDefect::IncompatibleEndpointParameter(vertex));
        }
        if evidence.incoming != 1 {
            defects.push(ClosedStitchDefect::IncomingDegree {
                vertex,
                degree: evidence.incoming,
            });
        }
        if evidence.outgoing != 1 {
            defects.push(ClosedStitchDefect::OutgoingDegree {
                vertex,
                degree: evidence.outgoing,
            });
        }
    }

    let mut used = vec![false; fragments.len()];
    let mut chains = Vec::new();
    for first in 0..fragments.len() {
        if used[first] {
            continue;
        }
        match fragments[first].span {
            ClosedFragmentSpan::Whole => {
                used[first] = true;
                chains.push(ClosedStitchChain {
                    fragments: vec![stitched_fragment(first, fragments[first], None)],
                    closed: true,
                });
            }
            ClosedFragmentSpan::Arc { .. } if !endpoint_valid[first] => {
                used[first] = true;
                chains.push(ClosedStitchChain {
                    fragments: vec![stitched_fragment(first, fragments[first], None)],
                    closed: false,
                });
            }
            ClosedFragmentSpan::Arc { .. } => {
                chains.push(walk_chain(
                    first, fragments, &endpoints, &outgoing, &mut used,
                ));
            }
        }
    }

    for (chain, evidence) in chains.iter().enumerate() {
        if !evidence.closed {
            defects.push(ClosedStitchDefect::OpenChain(chain));
        }
    }
    let completion = if defects.is_empty() {
        ClosedStitchCompletion::Complete
    } else {
        ClosedStitchCompletion::Indeterminate
    };
    ClosedStitchResult {
        vertices,
        chains,
        defects,
        completion,
    }
}

fn validate_sources(fragments: &[ClosedCurveFragment], defects: &mut Vec<ClosedStitchDefect>) {
    let mut states: Vec<BranchState> = Vec::new();
    let mut source_ordinals: HashMap<(ClosedBranchKey, usize), usize> = HashMap::new();
    for (fragment_index, fragment) in fragments.iter().enumerate() {
        match source_ordinals.entry((fragment.source.branch, fragment.source.ordinal)) {
            Entry::Occupied(slot) => {
                defects.push(ClosedStitchDefect::DuplicateSourceOrdinal {
                    first_fragment: *slot.get(),
                    fragment: fragment_index,
                });
            }
            Entry::Vacant(slot) => {
                slot.insert(fragment_index);
            }
        }

        let state_index = states
            .iter()
            .position(|state| state.source.branch == fragment.source.branch);
        let Some(state_index) = state_index else {
            states.push(BranchState {
                source: fragment.source,
                first_fragment: fragment_index,
                orientation: fragment.orientation,
                first_whole: matches!(fragment.span, ClosedFragmentSpan::Whole)
                    .then_some(fragment_index),
                first_arc: matches!(fragment.span, ClosedFragmentSpan::Arc { .. })
                    .then_some(fragment_index),
                mixed_reported: false,
            });
            continue;
        };
        let state = &mut states[state_index];
        if state.source.faces != fragment.source.faces {
            defects.push(ClosedStitchDefect::InconsistentBranchFaces {
                first_fragment: state.first_fragment,
                fragment: fragment_index,
            });
        }
        if state.orientation != fragment.orientation {
            defects.push(ClosedStitchDefect::InconsistentBranchOrientation {
                first_fragment: state.first_fragment,
                fragment: fragment_index,
            });
        }

        match fragment.span {
            ClosedFragmentSpan::Whole => {
                if let Some(first_whole) = state.first_whole {
                    defects.push(ClosedStitchDefect::DuplicateWholeFragment {
                        first_fragment: first_whole,
                        fragment: fragment_index,
                    });
                } else {
                    state.first_whole = Some(fragment_index);
                }
            }
            ClosedFragmentSpan::Arc { .. } => {
                state.first_arc.get_or_insert(fragment_index);
            }
        }
        if !state.mixed_reported
            && let (Some(whole), Some(arc)) = (state.first_whole, state.first_arc)
        {
            defects.push(ClosedStitchDefect::MixedWholeAndArc {
                first_fragment: whole.min(arc),
                fragment: whole.max(arc),
            });
            state.mixed_reported = true;
        }
    }
}

fn endpoint_is_valid(endpoint: CertifiedClosedEndpoint, source: ClosedFragmentSource) -> bool {
    match endpoint.key {
        CertifiedClosedEndpointKey::PeriodSeam { branch, site } => {
            branch == source.branch
                && site == source.period_seam_site
                && endpoint.edge_parameters == [None, None]
        }
        CertifiedClosedEndpointKey::TrimSite {
            site,
            edge_parameter_keys,
        } => {
            let sites = [site.a, site.b];
            let has_boundary_site = sites.iter().any(|site| !matches!(site, SiteKey::Face(_)));
            has_boundary_site
                && sites
                    .into_iter()
                    .zip(edge_parameter_keys)
                    .zip(endpoint.edge_parameters)
                    .enumerate()
                    .all(
                        |(operand, ((site, key), interval))| match (site, key, interval) {
                            (SiteKey::Edge(edge), Some(key), Some(interval)) => {
                                key.edge == edge
                                    && interval.lo().is_finite()
                                    && interval.hi().is_finite()
                            }
                            (SiteKey::Face(face), None, None) => face == source.faces[operand],
                            (SiteKey::Vertex(_), None, None) => true,
                            _ => false,
                        },
                    )
        }
    }
}

fn intern_endpoint(
    vertices: &mut Vec<ClosedStitchVertex>,
    index_of: &mut HashMap<CertifiedClosedEndpointKey, usize>,
    incoming: &mut Vec<Vec<usize>>,
    outgoing: &mut Vec<Vec<usize>>,
    endpoint: CertifiedClosedEndpoint,
) -> usize {
    match index_of.entry(endpoint.key) {
        Entry::Occupied(slot) => {
            let index = *slot.get();
            match intersect_parameter_evidence(
                endpoint.key,
                vertices[index].edge_parameters,
                endpoint.edge_parameters,
            ) {
                Some(merged) => vertices[index].edge_parameters = merged,
                None => vertices[index].edge_parameters_compatible = false,
            }
            index
        }
        Entry::Vacant(slot) => {
            let index = vertices.len();
            slot.insert(index);
            vertices.push(ClosedStitchVertex {
                key: endpoint.key,
                edge_parameters: endpoint.edge_parameters,
                incoming: 0,
                outgoing: 0,
                edge_parameters_compatible: true,
            });
            incoming.push(Vec::new());
            outgoing.push(Vec::new());
            index
        }
    }
}

fn intersect_parameter_evidence(
    key: CertifiedClosedEndpointKey,
    current: [Option<Interval>; 2],
    incoming: [Option<Interval>; 2],
) -> Option<[Option<Interval>; 2]> {
    match key {
        CertifiedClosedEndpointKey::PeriodSeam { .. } => {
            (current == [None, None] && incoming == [None, None]).then_some([None, None])
        }
        CertifiedClosedEndpointKey::TrimSite { site, .. } => {
            let mut merged = [None, None];
            for (operand, site) in [site.a, site.b].into_iter().enumerate() {
                match site {
                    SiteKey::Edge(_) => {
                        let x = current[operand]?;
                        let y = incoming[operand]?;
                        let lo = x.lo().max(y.lo());
                        let hi = x.hi().min(y.hi());
                        if lo > hi {
                            return None;
                        }
                        merged[operand] = Some(Interval::new(lo, hi));
                    }
                    SiteKey::Face(_) | SiteKey::Vertex(_) => {
                        if current[operand].is_some() || incoming[operand].is_some() {
                            return None;
                        }
                    }
                }
            }
            Some(merged)
        }
    }
}

fn walk_chain(
    first: usize,
    fragments: &[ClosedCurveFragment],
    endpoints: &[Option<[usize; 2]>],
    outgoing: &[Vec<usize>],
    used: &mut [bool],
) -> ClosedStitchChain {
    used[first] = true;
    let first_endpoints = endpoints[first].expect("validated arc has endpoint indices");
    let origin = first_endpoints[0];
    let mut at = first_endpoints[1];
    let mut ordered = vec![stitched_fragment(
        first,
        fragments[first],
        Some(first_endpoints),
    )];
    let closed = loop {
        if at == origin {
            break true;
        }
        let departures = &outgoing[at];
        if departures.len() != 1 {
            break false;
        }
        let next = departures[0];
        if used[next] {
            break false;
        }
        let next_endpoints = endpoints[next].expect("interned departure has endpoint indices");
        used[next] = true;
        ordered.push(stitched_fragment(
            next,
            fragments[next],
            Some(next_endpoints),
        ));
        at = next_endpoints[1];
    };
    ClosedStitchChain {
        fragments: ordered,
        closed,
    }
}

const fn stitched_fragment(
    input_fragment: usize,
    fragment: ClosedCurveFragment,
    endpoints: Option<[usize; 2]>,
) -> ClosedStitchedFragment {
    ClosedStitchedFragment {
        input_fragment,
        source: fragment.source,
        orientation: fragment.orientation,
        endpoints,
    }
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use ktopo::entity::VertexId as RawVertexId;
    use ktopo::store::Store;

    use super::*;

    struct Ids {
        faces: Vec<RawFaceId>,
        edges: Vec<RawEdgeId>,
        vertices: Vec<RawVertexId>,
    }

    fn ids() -> Ids {
        let mut store = Store::new();
        let body = ktopo::make::block(&mut store, &Frame::world(), [2.0, 2.0, 2.0])
            .expect("block construction succeeds");
        Ids {
            faces: store.faces_of_body(body).expect("block has faces"),
            edges: store.edges_of_body(body).expect("block has edges"),
            vertices: store.vertices_of_body(body).expect("block has vertices"),
        }
    }

    fn branch(ids: &Ids, branch: usize, faces: [usize; 2]) -> ClosedBranchSource {
        ClosedBranchSource {
            branch: ClosedBranchKey::new(branch),
            faces: [ids.faces[faces[0]], ids.faces[faces[1]]],
            period_seam_site: 0,
        }
    }

    fn source(branch: ClosedBranchSource, ordinal: usize) -> ClosedFragmentSource {
        branch.fragment(ordinal)
    }

    fn whole(
        source: ClosedFragmentSource,
        orientation: ClosedFragmentOrientation,
    ) -> ClosedCurveFragment {
        ClosedCurveFragment {
            source,
            orientation,
            span: ClosedFragmentSpan::Whole,
        }
    }

    fn arc(
        source: ClosedFragmentSource,
        orientation: ClosedFragmentOrientation,
        start: CertifiedClosedEndpoint,
        end: CertifiedClosedEndpoint,
    ) -> ClosedCurveFragment {
        ClosedCurveFragment {
            source,
            orientation,
            span: ClosedFragmentSpan::Arc { start, end },
        }
    }

    fn vertex_site(ids: &Ids, vertex: usize) -> CertifiedClosedEndpoint {
        CertifiedClosedEndpoint::trim_site(
            VertexKey {
                a: SiteKey::Vertex(ids.vertices[vertex]),
                b: SiteKey::Face(ids.faces[1]),
            },
            [None, None],
            [None, None],
        )
    }

    fn edge_site(
        ids: &Ids,
        edge: usize,
        root: usize,
        parameter: Interval,
    ) -> CertifiedClosedEndpoint {
        let raw = ids.edges[edge];
        CertifiedClosedEndpoint::trim_site(
            VertexKey {
                a: SiteKey::Edge(raw),
                b: SiteKey::Face(ids.faces[1]),
            },
            [Some(CertifiedSourceParameterKey::new(raw, root)), None],
            [Some(parameter), None],
        )
    }

    #[test]
    fn whole_closed_fragment_has_no_invented_vertex() {
        let ids = ids();
        let source = source(branch(&ids, 0, [0, 1]), 0);
        let result =
            stitch_closed_fragments(&[whole(source, ClosedFragmentOrientation::AlongCarrier)]);

        assert_eq!(result.completion, ClosedStitchCompletion::Complete);
        assert!(result.vertices.is_empty());
        assert!(result.defects.is_empty());
        assert_eq!(result.chains.len(), 1);
        assert!(result.chains[0].closed);
        assert_eq!(result.chains[0].fragments[0].input_fragment, 0);
        assert_eq!(result.chains[0].fragments[0].source, source);
        assert_eq!(result.chains[0].fragments[0].endpoints, None);
    }

    #[test]
    fn seam_split_cycle_preserves_direction_and_source_face_provenance() {
        let ids = ids();
        let branch_a = branch(&ids, 0, [0, 1]);
        let branch_b = branch(&ids, 1, [2, 1]);
        let seam = branch_a.period_seam();
        let edge_a_from_first = edge_site(&ids, 0, 0, Interval::new(0.75, 1.25));
        let edge_a_from_second = edge_site(&ids, 0, 0, Interval::new(1.0, 1.5));
        let vertex = vertex_site(&ids, 0);
        let fragments = [
            arc(
                source(branch_a, 0),
                ClosedFragmentOrientation::AlongCarrier,
                seam,
                edge_a_from_first,
            ),
            arc(
                source(branch_b, 0),
                ClosedFragmentOrientation::AgainstCarrier,
                edge_a_from_second,
                vertex,
            ),
            arc(
                source(branch_a, 1),
                ClosedFragmentOrientation::AlongCarrier,
                vertex,
                seam,
            ),
        ];

        let result = stitch_closed_fragments(&fragments);
        assert_eq!(result.completion, ClosedStitchCompletion::Complete);
        assert!(result.defects.is_empty());
        assert_eq!(result.chains.len(), 1);
        let loop_fragments = &result.chains[0].fragments;
        assert_eq!(
            loop_fragments
                .iter()
                .map(|fragment| fragment.input_fragment)
                .collect::<Vec<_>>(),
            [0, 1, 2]
        );
        assert_eq!(
            loop_fragments
                .iter()
                .map(|fragment| fragment.source.faces)
                .collect::<Vec<_>>(),
            [branch_a.faces, branch_b.faces, branch_a.faces]
        );
        assert_eq!(
            loop_fragments
                .iter()
                .map(|fragment| fragment.orientation)
                .collect::<Vec<_>>(),
            [
                ClosedFragmentOrientation::AlongCarrier,
                ClosedFragmentOrientation::AgainstCarrier,
                ClosedFragmentOrientation::AlongCarrier,
            ]
        );
        let edge_vertex = result
            .vertices
            .iter()
            .find(|vertex| matches!(vertex.key, CertifiedClosedEndpointKey::TrimSite { site, .. } if matches!(site.a, SiteKey::Edge(_))))
            .expect("edge trim site was interned");
        assert_eq!(
            edge_vertex.edge_parameters[0],
            Some(Interval::new(1.0, 1.25))
        );
    }

    #[test]
    fn distinct_certified_roots_on_one_edge_never_merge() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let vertex = vertex_site(&ids, 0);
        // The metric enclosures overlap deliberately.  Distinct certified
        // root identities remain distinct graph vertices regardless.
        let root_zero = edge_site(&ids, 0, 0, Interval::new(0.0, 2.0));
        let root_one = edge_site(&ids, 0, 1, Interval::new(1.0, 3.0));
        let fragments = [
            arc(
                source.fragment(0),
                ClosedFragmentOrientation::AlongCarrier,
                root_zero,
                vertex,
            ),
            arc(
                source.fragment(1),
                ClosedFragmentOrientation::AlongCarrier,
                vertex,
                root_one,
            ),
        ];

        let result = stitch_closed_fragments(&fragments);
        assert_eq!(result.vertices.len(), 3);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert!(
            result.defects.iter().any(|defect| matches!(
                defect,
                ClosedStitchDefect::IncomingDegree { degree: 0, .. }
            ))
        );
        assert!(
            result.defects.iter().any(|defect| matches!(
                defect,
                ClosedStitchDefect::OutgoingDegree { degree: 0, .. }
            ))
        );
    }

    #[test]
    fn disjoint_parameter_evidence_for_one_exact_root_is_indeterminate() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let seam = source.period_seam();
        let first = edge_site(&ids, 0, 0, Interval::new(0.0, 0.5));
        let second = edge_site(&ids, 0, 0, Interval::new(1.0, 1.5));
        let fragments = [
            arc(
                source.fragment(0),
                ClosedFragmentOrientation::AlongCarrier,
                seam,
                first,
            ),
            arc(
                source.fragment(1),
                ClosedFragmentOrientation::AlongCarrier,
                second,
                seam,
            ),
        ];

        let result = stitch_closed_fragments(&fragments);
        assert!(result.chains[0].closed);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert!(
            result.defects.iter().any(|defect| matches!(
                defect,
                ClosedStitchDefect::IncompatibleEndpointParameter(_)
            ))
        );
    }

    #[test]
    fn ambiguous_directed_incidence_is_not_reversed_or_repaired() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let start = vertex_site(&ids, 0);
        let end = vertex_site(&ids, 1);
        let fragments = [
            arc(
                source.fragment(0),
                ClosedFragmentOrientation::AlongCarrier,
                start,
                end,
            ),
            arc(
                source.fragment(1),
                ClosedFragmentOrientation::AlongCarrier,
                start,
                end,
            ),
        ];

        let result = stitch_closed_fragments(&fragments);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert_eq!(result.chains.len(), 2);
        assert!(result.chains.iter().all(|chain| !chain.closed));
        assert!(
            result.defects.iter().any(|defect| matches!(
                defect,
                ClosedStitchDefect::OutgoingDegree { degree: 2, .. }
            ))
        );
        assert!(
            result.defects.iter().any(|defect| matches!(
                defect,
                ClosedStitchDefect::IncomingDegree { degree: 2, .. }
            ))
        );
    }

    #[test]
    fn mixed_whole_and_arc_for_one_branch_is_typed_indeterminate() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let seam = source.period_seam();
        let fragments = [
            whole(source.fragment(0), ClosedFragmentOrientation::AlongCarrier),
            arc(
                source.fragment(1),
                ClosedFragmentOrientation::AlongCarrier,
                seam,
                seam,
            ),
        ];

        let result = stitch_closed_fragments(&fragments);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert!(
            result
                .defects
                .iter()
                .any(|defect| matches!(defect, ClosedStitchDefect::MixedWholeAndArc { .. }))
        );
        // Both pieces remain visible; the stitcher does not discard or merge
        // either interpretation of the contradictory source branch.
        assert_eq!(result.chains.len(), 2);
        assert!(result.chains.iter().all(|chain| chain.closed));
    }

    #[test]
    fn malformed_edge_parameter_shape_fails_closed() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let raw_edge = ids.edges[0];
        let malformed = CertifiedClosedEndpoint::trim_site(
            VertexKey {
                a: SiteKey::Edge(raw_edge),
                b: SiteKey::Face(ids.faces[1]),
            },
            [Some(CertifiedSourceParameterKey::new(raw_edge, 0)), None],
            // Missing the required source-edge interval.
            [None, None],
        );
        let fragment = arc(
            source.fragment(0),
            ClosedFragmentOrientation::AlongCarrier,
            malformed,
            source.period_seam(),
        );

        let result = stitch_closed_fragments(&[fragment]);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert!(result.vertices.is_empty());
        assert_eq!(result.chains.len(), 1);
        assert!(!result.chains[0].closed);
        assert!(
            result
                .defects
                .contains(&ClosedStitchDefect::InvalidEndpointEvidence {
                    fragment: 0,
                    end: ClosedFragmentEnd::Start,
                })
        );
    }

    #[test]
    fn unowned_parameter_seam_site_fails_closed() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let wrong_seam = CertifiedClosedEndpoint {
            key: CertifiedClosedEndpointKey::PeriodSeam {
                branch: source.branch,
                site: source.period_seam_site + 1,
            },
            edge_parameters: [None, None],
        };
        let fragment = arc(
            source.fragment(0),
            ClosedFragmentOrientation::AlongCarrier,
            wrong_seam,
            wrong_seam,
        );

        let result = stitch_closed_fragments(&[fragment]);
        assert_eq!(result.completion, ClosedStitchCompletion::Indeterminate);
        assert_eq!(
            result.defects[..2],
            [
                ClosedStitchDefect::InvalidEndpointEvidence {
                    fragment: 0,
                    end: ClosedFragmentEnd::Start,
                },
                ClosedStitchDefect::InvalidEndpointEvidence {
                    fragment: 0,
                    end: ClosedFragmentEnd::End,
                },
            ]
        );
    }

    #[test]
    fn identical_input_reproduces_identical_evidence() {
        let ids = ids();
        let source = branch(&ids, 0, [0, 1]);
        let seam = source.period_seam();
        let vertex = vertex_site(&ids, 0);
        let fragments = [
            arc(
                source.fragment(0),
                ClosedFragmentOrientation::AlongCarrier,
                seam,
                vertex,
            ),
            arc(
                source.fragment(1),
                ClosedFragmentOrientation::AlongCarrier,
                vertex,
                seam,
            ),
        ];

        assert_eq!(
            stitch_closed_fragments(&fragments),
            stitch_closed_fragments(&fragments)
        );
    }
}
