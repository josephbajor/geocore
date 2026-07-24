//! Exact-topology assembly for finite non-wrapping skew-cylinder sheet spans.
//!
//! This module consumes four already-complete 64-work axial-bound queries. It
//! performs no new root solve and depends on no whole-sheet carrier
//! certificate. The merge is purely topological: projective root corridors
//! establish a strict cyclic order, exact open-cell relations establish finite
//! occupancy, and exact-equal physical roots from independent bound equations
//! are grouped before their state changes are applied atomically. Numeric
//! range endpoints are the representable inside sides of those exact-source
//! corridors; every active bound root remains part of the endpoint proof.

use core::cmp::Ordering;

use super::{
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderAngularRootBracket,
    SkewCylinderAxialBoundProvenance, SkewCylinderAxialBoundTopology, SkewCylinderAxialBoundary,
    SkewCylinderAxialRelation, SkewCylinderAxialRoot, SkewCylinderHalfAngleChart,
    SkewCylinderHalfAngleRootBracket, SkewCylinderSheet,
};
use crate::exact::bounded_polynomial::RootBracket;
use crate::exact::bounded_root_relation::{ExactRootRelation, classify_exact_root_relation};
use kcore::interval::Interval;
use kgeom::param::ParamRange;

const TAU: f64 = core::f64::consts::TAU;
// Deterministic interval-rounding headroom for both the stored and exact-source
// residual enclosures. The merged-corridor checks below still refuse any
// authored chart where these steps could cross another root.
const ENDPOINT_GUARD_ULPS: usize = 2 * SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS;

/// Analytic bound on axial-bound events attached to one sheet at one physical
/// root: each of the four authored bounds contributes at most one event.
pub const SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER: usize = 4;

/// Fixed logical work for one exact bound-pair/common-chart root query. The
/// query covers every owned quartic root in that pair and chart.
pub const SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK: u64 = 32;

/// Six unordered bound pairs times the two owned projective charts.
pub const SKEW_CYLINDER_ROOT_CLUSTER_MAX_QUERY_COUNT: usize = 12;

/// Maximum exact root-cluster work for one complete four-bound family.
pub const SKEW_CYLINDER_ROOT_CLUSTER_MAX_EXACT_WORK: u64 =
    SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
        * SKEW_CYLINDER_ROOT_CLUSTER_MAX_QUERY_COUNT as u64;

/// Exact set of bound-pair/chart common-root queries required by one family.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkewCylinderRootClusterQueryPlan {
    bits: u16,
}

impl SkewCylinderRootClusterQueryPlan {
    /// Stable 12-bit query mask in unordered-bound-pair then Tangent/Cotangent
    /// order.
    pub const fn bits(self) -> u16 {
        self.bits
    }

    /// Number of exact pair/chart queries in this family.
    pub const fn query_count(self) -> usize {
        self.bits.count_ones() as usize
    }

    /// Exact logical work represented by this query plan.
    pub const fn work(self) -> u64 {
        self.query_count() as u64 * SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
    }
}

/// Four exact bound topologies and the authored windows they must describe.
///
/// `ranges` is in canonical solver order; provenance remains in caller order
/// through `canonical_to_source`.
#[derive(Debug, Clone, Copy)]
pub struct SkewCylinderOpenSpanTopologyInput<'a> {
    /// Four sealed exact bound outcomes.
    pub topologies: &'a [SkewCylinderAxialBoundTopology; 4],
    /// Exact authored windows in formula order.
    pub ranges: [[ParamRange; 2]; 2],
    /// Formula-slot to caller/source-slot permutation.
    pub canonical_to_source: [usize; 2],
}

/// The strict side of a source-root corridor retained by one finite span.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderRootInsideSide {
    /// Increasing-longitude side immediately before the exact root.
    Before,
    /// Increasing-longitude side immediately after the exact root.
    After,
}

/// Role of one physical root in the closed finite-window intersection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderFiniteWindowRootEventKind {
    /// Open occupancy changes across the root, so it bounds a one-dimensional
    /// component.
    Boundary,
    /// Open occupancy remains inside on both sides and merely touches one or
    /// more authored bounds at the root.
    Contact,
    /// Open occupancy is outside on both sides but the root itself satisfies
    /// every closed axial bound.
    Isolated,
}

/// All exact axial-bound events owned by one sheet at one physical root.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderFiniteWindowRootEvent {
    sheet: SkewCylinderSheet,
    kind: SkewCylinderFiniteWindowRootEventKind,
    roots: [Option<SkewCylinderAxialRoot>; SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER],
    root_count: u8,
    carrier_parameter: f64,
}

impl SkewCylinderFiniteWindowRootEvent {
    /// Ordered quadratic sheet owning this physical event.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.sheet
    }

    /// Closed-set role of this event.
    pub const fn kind(self) -> SkewCylinderFiniteWindowRootEventKind {
        self.kind
    }

    /// Number of exact bound roots active at the physical event.
    pub const fn root_count(self) -> usize {
        self.root_count as usize
    }

    /// Exact bound root in canonical bound order.
    pub const fn root(self, index: usize) -> Option<SkewCylinderAxialRoot> {
        if index < self.root_count as usize {
            self.roots[index]
        } else {
            None
        }
    }

    /// Deterministic authored-chart representative of the physical root.
    pub const fn carrier_parameter(self) -> f64 {
        self.carrier_parameter
    }
}

/// Exact-source endpoint cluster plus the representable parameter on its
/// proven finite-window side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderOpenSpanEndpointProof {
    /// Exact physical root and every active axial-bound event.
    pub event: SkewCylinderFiniteWindowRootEvent,
    /// Retained side of that root.
    pub inside_side: SkewCylinderRootInsideSide,
    /// Representable inside-side carrier parameter.
    pub carrier_parameter: f64,
}

/// One proper finite component in increasing authored carrier order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderOpenSpan {
    /// Ordered quadratic sheet.
    pub sheet: SkewCylinderSheet,
    /// Strict guarded open range.
    pub range: ParamRange,
    /// Lower canonical endpoint proof.
    pub start: SkewCylinderOpenSpanEndpointProof,
    /// Upper canonical endpoint proof.
    pub end: SkewCylinderOpenSpanEndpointProof,
}

impl SkewCylinderOpenSpan {
    /// Lift both exact-source projective root brackets into the canonical
    /// longitude chart that owns this non-wrapping span.
    ///
    /// The returned order is `[lower, upper]` in increasing carrier
    /// parameter, independent of caller operand order. The exact projective
    /// provenance remains attached to `start` and `end`; these intervals are
    /// only its monotone longitude image for pcurve corridor certification.
    pub fn root_longitude_intervals(self, authored_longitude: ParamRange) -> Option<[Interval; 2]> {
        if self.start.inside_side != SkewCylinderRootInsideSide::After
            || self.end.inside_side != SkewCylinderRootInsideSide::Before
            || self.start.carrier_parameter.to_bits() != self.range.lo.to_bits()
            || self.end.carrier_parameter.to_bits() != self.range.hi.to_bits()
        {
            return None;
        }
        let lower = lift_root_longitude_interval(self.start, authored_longitude)?;
        let upper = lift_root_longitude_interval(self.end, authored_longitude)?;
        (lower.hi() < self.range.lo && upper.lo() > self.range.hi).then_some([lower, upper])
    }
}

/// Complete finite occupancy for one ordered strict-positive sheet.
#[derive(Debug, Clone, PartialEq)]
pub enum SkewCylinderFiniteSheetTopology {
    /// No point of this sheet lies inside all four axial bounds.
    Outside,
    /// The complete full-period sheet lies inside all four axial bounds.
    Whole,
    /// Every retained non-wrapping finite component in increasing longitude.
    Open(Vec<SkewCylinderOpenSpan>),
}

/// Conservative refusal causes for the topology-only merge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderOpenSpanFailure {
    /// Formula slots do not map bijectively to the two caller/source slots.
    InvalidSourcePermutation,
    /// At least one authored range is nonfinite or reverses its endpoints.
    InvalidRange,
    /// A bound identity or full-period longitude differs from its header.
    RangeMismatch,
    /// The four lower/upper source bounds are not represented exactly once.
    DuplicateOrMissingBound,
    /// Sealed roots and open-cell relations do not form one valid cyclic sweep.
    InconsistentTopology,
    /// A repeated or non-transverse axial root cannot bound an open component.
    ContactRoot,
    /// A projective root bracket straddles a chart sector boundary.
    AmbiguousRootSector,
    /// Two exact-source root corridors cannot be strictly ordered.
    CoincidentOrOverlappingRoots,
    /// Exact common-root arithmetic could not prove equality for overlapping
    /// source corridors inside its fixed envelope.
    ExactRootRelationIndeterminate,
    /// A root-to-guard continuation leaves the authored longitude chart.
    RootCorridorCrossesSeam,
    /// An occupied component wraps through the authored longitude seam.
    SeamWrappingSpan,
}

/// Sealed complete two-sheet finite-window topology.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderFiniteWindowTopologyCertificate {
    formula_cylinders: [kgeom::surface::Cylinder; 2],
    formula_ranges: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    bound_topologies: [SkewCylinderAxialBoundTopology; 4],
    root_cluster_query_plan: SkewCylinderRootClusterQueryPlan,
    sheets: [SkewCylinderFiniteSheetTopology; 2],
    sheet_root_events: [Vec<SkewCylinderFiniteWindowRootEvent>; 2],
}

impl SkewCylinderFiniteWindowTopologyCertificate {
    /// Exact cylinders in formula/ruling order.
    pub const fn formula_cylinders(&self) -> [kgeom::surface::Cylinder; 2] {
        self.formula_cylinders
    }

    /// Exact authored windows in formula/ruling order.
    pub const fn formula_ranges(&self) -> [[ParamRange; 2]; 2] {
        self.formula_ranges
    }

    /// Formula-slot to caller/source-slot permutation.
    pub const fn formula_to_source(&self) -> [usize; 2] {
        self.formula_to_source
    }

    /// The four sealed bound outcomes used by the sweep.
    pub const fn bound_topologies(&self) -> &[SkewCylinderAxialBoundTopology; 4] {
        &self.bound_topologies
    }

    /// Exact bound-pair/chart work plan used to prove coincident cuts.
    pub const fn root_cluster_query_plan(&self) -> SkewCylinderRootClusterQueryPlan {
        self.root_cluster_query_plan
    }

    /// Complete occupancy for Lower or Upper.
    pub const fn sheet(&self, sheet: SkewCylinderSheet) -> &SkewCylinderFiniteSheetTopology {
        &self.sheets[sheet_index(sheet)]
    }

    /// Every closed-set root event on one sheet in increasing physical-root
    /// order. Boundary events also appear on the adjacent open spans; Contact
    /// and Isolated events let consumers fail closed until they publish the
    /// corresponding zero-dimensional topology.
    pub fn root_events(&self, sheet: SkewCylinderSheet) -> &[SkewCylinderFiniteWindowRootEvent] {
        &self.sheet_root_events[sheet_index(sheet)]
    }
}

#[derive(Debug, Clone, Copy)]
struct RootCut {
    topology_index: usize,
    cyclic_ordinal: usize,
    bracket: SkewCylinderHalfAngleRootBracket,
    events: [Option<SkewCylinderAxialRoot>; 2],
}

#[derive(Debug, Clone)]
struct RootCluster {
    cuts: Vec<RootCut>,
}

#[derive(Debug, Clone, Copy)]
struct AuthoredRootCut {
    root_parameter: f64,
    before_parameter: f64,
    after_parameter: f64,
}

#[derive(Debug, Clone)]
struct AuthoredRootCluster {
    source: RootCluster,
    root_parameter: f64,
    before_parameter: f64,
    after_parameter: f64,
}

#[derive(Debug, Clone)]
struct CutTransition {
    cluster: AuthoredRootCluster,
    before_inside: [bool; 2],
    at_inside: [bool; 2],
    after_inside: [bool; 2],
}

fn lift_root_longitude_interval(
    proof: SkewCylinderOpenSpanEndpointProof,
    authored_longitude: ParamRange,
) -> Option<Interval> {
    if !authored_longitude.is_finite() || authored_longitude.width() != TAU {
        return None;
    }
    if proof.event.kind != SkewCylinderFiniteWindowRootEventKind::Boundary
        || proof.event.root_count == 0
    {
        return None;
    }
    let mut angular = proof.event.root(0)?.angular_bracket();
    for index in 1..proof.event.root_count() {
        let root = proof.event.root(index)?.angular_bracket();
        angular = SkewCylinderAngularRootBracket {
            lo: angular.lo.min(root.lo),
            hi: angular.hi.max(root.hi),
        };
    }
    let representative = angular.representative();
    let lifted_representative =
        fit_periodic_parameter(representative, authored_longitude, TAU, 0.0)?;
    let shift = lifted_representative - representative;
    let mut lo = angular.lo + shift;
    let mut hi = angular.hi + shift;
    match proof.inside_side {
        SkewCylinderRootInsideSide::After if hi >= proof.carrier_parameter => {
            lo -= TAU;
            hi -= TAU;
        }
        SkewCylinderRootInsideSide::Before if lo <= proof.carrier_parameter => {
            lo += TAU;
            hi += TAU;
        }
        SkewCylinderRootInsideSide::After | SkewCylinderRootInsideSide::Before => {}
    }
    if !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || lo <= authored_longitude.lo
        || hi >= authored_longitude.hi
    {
        return None;
    }
    match proof.inside_side {
        SkewCylinderRootInsideSide::After if hi < proof.carrier_parameter => {
            Some(Interval::new(lo, hi))
        }
        SkewCylinderRootInsideSide::Before if lo > proof.carrier_parameter => {
            Some(Interval::new(lo, hi))
        }
        SkewCylinderRootInsideSide::After | SkewCylinderRootInsideSide::Before => None,
    }
}

fn fit_periodic_parameter(
    candidate: f64,
    range: ParamRange,
    period: f64,
    tolerance: f64,
) -> Option<f64> {
    let k_min = ((range.lo - tolerance - candidate) / period).ceil() as i64;
    let k_max = ((range.hi + tolerance - candidate) / period).floor() as i64;
    if k_min > k_max {
        return None;
    }
    Some((candidate + k_min as f64 * period).clamp(range.lo, range.hi))
}

/// Plan every exact common-root query needed by the four-bound merge.
///
/// This validates and normalizes the sealed axial-bound inputs but performs no
/// polynomial GCD or common-root isolation.
pub fn plan_skew_cylinder_root_clusters(
    input: SkewCylinderOpenSpanTopologyInput<'_>,
) -> Result<SkewCylinderRootClusterQueryPlan, SkewCylinderOpenSpanFailure> {
    validate_ranges(input.ranges, input.canonical_to_source)?;
    let formula_cylinders = input.topologies[0].formula_cylinders();
    if input.topologies.iter().any(|topology| {
        topology.formula_cylinders() != formula_cylinders
            || topology.formula_to_source() != input.canonical_to_source
    }) {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    let topologies = normalize_topologies(input)?;
    let mut cuts = Vec::new();
    for (topology_index, topology) in topologies.iter().enumerate() {
        cuts.extend(validate_topology(topology_index, topology)?);
    }
    build_root_cluster_query_plan(&cuts)
}

/// Merge four exact source-bound topologies into complete Lower/Upper finite
/// occupancy. The result order is always `[Lower, Upper]`.
///
/// Callers that account exact work separately can first call
/// [`plan_skew_cylinder_root_clusters`]. This classifier reconstructs the same
/// plan before executing any planned common-root queries.
pub fn classify_skew_cylinder_open_spans(
    input: SkewCylinderOpenSpanTopologyInput<'_>,
) -> Result<SkewCylinderFiniteWindowTopologyCertificate, SkewCylinderOpenSpanFailure> {
    validate_ranges(input.ranges, input.canonical_to_source)?;
    let formula_cylinders = input.topologies[0].formula_cylinders();
    if input.topologies.iter().any(|topology| {
        topology.formula_cylinders() != formula_cylinders
            || topology.formula_to_source() != input.canonical_to_source
    }) {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    let topologies = normalize_topologies(input)?;
    let mut cuts = Vec::new();
    for (topology_index, topology) in topologies.iter().enumerate() {
        cuts.extend(validate_topology(topology_index, topology)?);
    }
    let root_cluster_query_plan = build_root_cluster_query_plan(&cuts)?;
    let clusters = sort_and_cluster_projective_roots(&topologies, &mut cuts)?;

    let carrier_range = input.ranges[0][0];
    let mut authored = clusters
        .into_iter()
        .map(|cluster| contextualize_root_cluster(cluster, carrier_range))
        .collect::<Result<Vec<_>, _>>()?;
    authored.sort_by(|lhs, rhs| lhs.root_parameter.total_cmp(&rhs.root_parameter));
    validate_authored_root_order(&authored, carrier_range)?;

    let (initial_inside, transitions) = sweep_topologies(&topologies, &authored)?;
    let (lower, lower_events) = classify_sheet(
        SkewCylinderSheet::Lower,
        initial_inside[0],
        &transitions,
        carrier_range,
    )?;
    let (upper, upper_events) = classify_sheet(
        SkewCylinderSheet::Upper,
        initial_inside[1],
        &transitions,
        carrier_range,
    )?;
    Ok(SkewCylinderFiniteWindowTopologyCertificate {
        formula_cylinders,
        formula_ranges: input.ranges,
        formula_to_source: input.canonical_to_source,
        bound_topologies: core::array::from_fn(|index| topologies[index].clone()),
        root_cluster_query_plan,
        sheets: [lower, upper],
        sheet_root_events: [lower_events, upper_events],
    })
}

fn build_root_cluster_query_plan(
    cuts: &[RootCut],
) -> Result<SkewCylinderRootClusterQueryPlan, SkewCylinderOpenSpanFailure> {
    let mut bits = 0_u16;
    for (index, first) in cuts.iter().copied().enumerate() {
        for second in cuts.iter().copied().skip(index + 1) {
            if first.topology_index == second.topology_index
                || first.bracket.chart != second.bracket.chart
            {
                continue;
            }
            match compare_projective_corridors(first.bracket, second.bracket) {
                Ok(_) => {}
                Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots) => {
                    let pair = bound_pair_ordinal(first.topology_index, second.topology_index)
                        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
                    let chart = match first.bracket.chart {
                        SkewCylinderHalfAngleChart::Tangent => 0,
                        SkewCylinderHalfAngleChart::Cotangent => 1,
                    };
                    bits |= 1_u16 << (2 * pair + chart);
                }
                Err(failure) => return Err(failure),
            }
        }
    }
    Ok(SkewCylinderRootClusterQueryPlan { bits })
}

fn bound_pair_ordinal(first: usize, second: usize) -> Option<usize> {
    let (first, second) = if first < second {
        (first, second)
    } else {
        (second, first)
    };
    match (first, second) {
        (0, 1) => Some(0),
        (0, 2) => Some(1),
        (0, 3) => Some(2),
        (1, 2) => Some(3),
        (1, 3) => Some(4),
        (2, 3) => Some(5),
        _ => None,
    }
}

fn validate_ranges(
    ranges: [[ParamRange; 2]; 2],
    canonical_to_source: [usize; 2],
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if !matches!(canonical_to_source, [0, 1] | [1, 0]) {
        return Err(SkewCylinderOpenSpanFailure::InvalidSourcePermutation);
    }
    if ranges
        .iter()
        .flatten()
        .any(|range| !range.is_finite() || range.width() < 0.0)
    {
        return Err(SkewCylinderOpenSpanFailure::InvalidRange);
    }
    if ranges
        .iter()
        .any(|range| range[0].width() != TAU || range[1].width() < 0.0)
    {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    Ok(())
}

fn normalize_topologies<'a>(
    input: SkewCylinderOpenSpanTopologyInput<'a>,
) -> Result<[&'a SkewCylinderAxialBoundTopology; 4], SkewCylinderOpenSpanFailure> {
    let mut normalized: [Option<&SkewCylinderAxialBoundTopology>; 4] = [None; 4];
    for topology in input.topologies {
        let source_operand = topology.provenance().source_operand;
        let canonical_operand = input
            .canonical_to_source
            .iter()
            .position(|source| *source == source_operand)
            .ok_or(SkewCylinderOpenSpanFailure::RangeMismatch)?;
        let boundary_index = match topology.provenance().boundary {
            SkewCylinderAxialBoundary::Lower => 0,
            SkewCylinderAxialBoundary::Upper => 1,
        };
        let expected_value = if boundary_index == 0 {
            input.ranges[canonical_operand][1].lo
        } else {
            input.ranges[canonical_operand][1].hi
        };
        if topology.provenance().value.to_bits() != expected_value.to_bits() {
            return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
        }
        let slot = 2 * canonical_operand + boundary_index;
        if normalized[slot].replace(topology).is_some() {
            return Err(SkewCylinderOpenSpanFailure::DuplicateOrMissingBound);
        }
    }
    let [
        Some(first_lower),
        Some(first_upper),
        Some(second_lower),
        Some(second_upper),
    ] = normalized
    else {
        return Err(SkewCylinderOpenSpanFailure::DuplicateOrMissingBound);
    };
    Ok([first_lower, first_upper, second_lower, second_upper])
}

fn validate_topology(
    topology_index: usize,
    topology: &SkewCylinderAxialBoundTopology,
) -> Result<Vec<RootCut>, SkewCylinderOpenSpanFailure> {
    if topology.open_cell_relations().is_empty() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if topology.roots().is_empty() {
        return if topology.open_cell_relations().len() == 1 {
            Ok(Vec::new())
        } else {
            Err(SkewCylinderOpenSpanFailure::InconsistentTopology)
        };
    }

    let root_count = topology.open_cell_relations().len();
    let mut cuts = (0..root_count)
        .map(|cyclic_ordinal| RootCut {
            topology_index,
            cyclic_ordinal,
            bracket: SkewCylinderHalfAngleRootBracket {
                chart: SkewCylinderHalfAngleChart::Tangent,
                lo: 0.0,
                hi: 0.0,
            },
            events: [None; 2],
        })
        .collect::<Vec<_>>();
    let mut initialized = vec![false; root_count];

    for root in topology.roots() {
        validate_root(topology, *root)?;
        let cut = cuts
            .get_mut(root.cyclic_ordinal)
            .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
        if initialized[root.cyclic_ordinal] {
            if cut.bracket != root.bracket {
                return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
            }
        } else {
            validate_root_bracket(root.bracket)?;
            cut.bracket = root.bracket;
            initialized[root.cyclic_ordinal] = true;
        }
        let sheet = sheet_index(root.sheet);
        if cut.events[sheet].replace(*root).is_some() {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }
    if initialized.contains(&false) {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }

    for cut in &cuts {
        let before =
            topology.open_cell_relations()[(cut.cyclic_ordinal + root_count - 1) % root_count];
        let after = topology.open_cell_relations()[cut.cyclic_ordinal];
        for sheet in 0..2 {
            let event_is_valid = match cut.events[sheet] {
                Some(root) => root.repeated || before[sheet] != after[sheet],
                None => before[sheet] == after[sheet],
            };
            if !event_is_valid {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
        }
    }
    for pair in cuts.windows(2) {
        if compare_projective_corridors(pair[0].bracket, pair[1].bracket)? != Ordering::Less {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }
    Ok(cuts)
}

fn validate_root(
    topology: &SkewCylinderAxialBoundTopology,
    root: SkewCylinderAxialRoot,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if root.provenance != topology.provenance() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if !root.repeated && root.before == root.after {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let root_count = topology.open_cell_relations().len();
    if root.cyclic_ordinal >= root_count {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let sheet = sheet_index(root.sheet);
    let before =
        topology.open_cell_relations()[(root.cyclic_ordinal + root_count - 1) % root_count][sheet];
    let after = topology.open_cell_relations()[root.cyclic_ordinal][sheet];
    if (root.before, root.after) != (before, after) {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok(())
}

fn validate_root_bracket(
    bracket: SkewCylinderHalfAngleRootBracket,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    if !bracket.lo.is_finite() || !bracket.hi.is_finite() || bracket.lo > bracket.hi {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let owned = match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent => bracket.lo >= -1.0 && bracket.hi <= 1.0,
        SkewCylinderHalfAngleChart::Cotangent => bracket.lo > -1.0 && bracket.hi < 1.0,
    };
    if !owned {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    if bracket.lo < 0.0 && bracket.hi > 0.0 {
        return Err(SkewCylinderOpenSpanFailure::AmbiguousRootSector);
    }
    Ok(())
}

fn sort_and_cluster_projective_roots(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    cuts: &mut [RootCut],
) -> Result<Vec<RootCluster>, SkewCylinderOpenSpanFailure> {
    // At most sixteen cuts exist. Insertion sort keeps fallible exact-root
    // comparison explicit rather than hiding a refusal in an infallible sort.
    for index in 1..cuts.len() {
        let mut cursor = index;
        while cursor > 0 {
            match compare_projective_roots(topologies, cuts[cursor - 1], cuts[cursor])? {
                Ordering::Less => break,
                Ordering::Equal => {
                    if cut_identity(cuts[cursor - 1]) <= cut_identity(cuts[cursor]) {
                        break;
                    }
                    cuts.swap(cursor - 1, cursor);
                    cursor -= 1;
                }
                Ordering::Greater => {
                    cuts.swap(cursor - 1, cursor);
                    cursor -= 1;
                }
            }
        }
    }

    let mut clusters: Vec<RootCluster> = Vec::with_capacity(cuts.len());
    for cut in cuts.iter().copied() {
        let Some(previous) = clusters.last_mut() else {
            clusters.push(RootCluster { cuts: vec![cut] });
            continue;
        };
        match compare_projective_roots(topologies, previous.cuts[0], cut)? {
            Ordering::Less => clusters.push(RootCluster { cuts: vec![cut] }),
            Ordering::Equal => {
                if previous
                    .cuts
                    .iter()
                    .any(|member| member.topology_index == cut.topology_index)
                {
                    return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
                }
                previous.cuts.push(cut);
            }
            Ordering::Greater => {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
        }
    }
    Ok(clusters)
}

fn compare_projective_roots(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    lhs: RootCut,
    rhs: RootCut,
) -> Result<Ordering, SkewCylinderOpenSpanFailure> {
    match compare_projective_corridors(lhs.bracket, rhs.bracket) {
        Ok(order) => Ok(order),
        Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots) => {
            let lhs_polynomial = topologies[lhs.topology_index]
                .exact_root_polynomial(lhs.bracket.chart)
                .map_err(|_| SkewCylinderOpenSpanFailure::ExactRootRelationIndeterminate)?;
            let rhs_polynomial = topologies[rhs.topology_index]
                .exact_root_polynomial(rhs.bracket.chart)
                .map_err(|_| SkewCylinderOpenSpanFailure::ExactRootRelationIndeterminate)?;
            let relation = classify_exact_root_relation(
                &lhs_polynomial,
                RootBracket {
                    lo: lhs.bracket.lo,
                    hi: lhs.bracket.hi,
                },
                &rhs_polynomial,
                RootBracket {
                    lo: rhs.bracket.lo,
                    hi: rhs.bracket.hi,
                },
            )
            .map_err(|_| SkewCylinderOpenSpanFailure::ExactRootRelationIndeterminate)?;
            match relation {
                ExactRootRelation::Same => Ok(Ordering::Equal),
                ExactRootRelation::Distinct => {
                    Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
                }
            }
        }
        Err(failure) => Err(failure),
    }
}

const fn cut_identity(cut: RootCut) -> (usize, usize) {
    (cut.topology_index, cut.cyclic_ordinal)
}

fn compare_projective_corridors(
    lhs: SkewCylinderHalfAngleRootBracket,
    rhs: SkewCylinderHalfAngleRootBracket,
) -> Result<Ordering, SkewCylinderOpenSpanFailure> {
    let lhs_sector = root_sector(lhs)?;
    let rhs_sector = root_sector(rhs)?;
    if lhs_sector != rhs_sector {
        return Ok(lhs_sector.cmp(&rhs_sector));
    }
    let disjoint_order = if matches!(lhs_sector, 0 | 3) {
        if lhs.hi < rhs.lo {
            Some(Ordering::Less)
        } else if rhs.hi < lhs.lo {
            Some(Ordering::Greater)
        } else {
            None
        }
    } else if lhs.lo > rhs.hi {
        Some(Ordering::Less)
    } else if rhs.lo > lhs.hi {
        Some(Ordering::Greater)
    } else {
        None
    };
    disjoint_order.ok_or(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
}

fn root_sector(
    bracket: SkewCylinderHalfAngleRootBracket,
) -> Result<u8, SkewCylinderOpenSpanFailure> {
    validate_root_bracket(bracket)?;
    Ok(match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent if bracket.lo >= 0.0 => 0,
        SkewCylinderHalfAngleChart::Cotangent if bracket.lo >= 0.0 => 1,
        SkewCylinderHalfAngleChart::Cotangent => 2,
        SkewCylinderHalfAngleChart::Tangent => 3,
    })
}

fn contextualize_root_cut(
    source: RootCut,
    range: ParamRange,
) -> Result<AuthoredRootCut, SkewCylinderOpenSpanFailure> {
    let angular = source
        .events
        .iter()
        .flatten()
        .next()
        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?
        .angular_bracket();
    let root = angular.representative();
    let root_parameter = fit_periodic_parameter(root, range, TAU, 0.0)
        .ok_or(SkewCylinderOpenSpanFailure::RangeMismatch)?;
    let before_distance = backward_cyclic_distance(root, angular.strict_before_side());
    let after_distance = forward_cyclic_distance(root, angular.strict_after_side());
    if !before_distance.is_finite()
        || !after_distance.is_finite()
        || before_distance < 0.0
        || after_distance < 0.0
        || before_distance + after_distance <= 0.0
    {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    let mut before_parameter = root_parameter - before_distance;
    let mut after_parameter = root_parameter + after_distance;
    let collapsed = angular.lo == angular.hi;
    if before_parameter < range.lo || after_parameter > range.hi {
        if !collapsed || root_parameter != range.lo {
            return Err(SkewCylinderOpenSpanFailure::RootCorridorCrossesSeam);
        }
        before_parameter = range.hi - before_distance;
        after_parameter = range.lo + after_distance;
    }
    before_parameter = guard_parameter(before_parameter, Ordering::Less);
    after_parameter = guard_parameter(after_parameter, Ordering::Greater);
    if !(range.lo < after_parameter
        && after_parameter < range.hi
        && range.lo < before_parameter
        && before_parameter < range.hi)
    {
        return Err(SkewCylinderOpenSpanFailure::RootCorridorCrossesSeam);
    }
    Ok(AuthoredRootCut {
        root_parameter,
        before_parameter,
        after_parameter,
    })
}

fn guard_parameter(parameter: f64, direction: Ordering) -> f64 {
    let local_step = match direction {
        Ordering::Less => parameter - parameter.next_down(),
        Ordering::Greater => parameter.next_up() - parameter,
        Ordering::Equal => return parameter,
    };
    let distance = local_step * ENDPOINT_GUARD_ULPS as f64;
    match direction {
        Ordering::Less => (parameter - distance).next_down(),
        Ordering::Greater => (parameter + distance).next_up(),
        Ordering::Equal => parameter,
    }
}

fn contextualize_root_cluster(
    mut source: RootCluster,
    range: ParamRange,
) -> Result<AuthoredRootCluster, SkewCylinderOpenSpanFailure> {
    source.cuts.sort_by_key(|cut| cut_identity(*cut));
    let authored = source
        .cuts
        .iter()
        .copied()
        .map(|cut| contextualize_root_cut(cut, range))
        .collect::<Result<Vec<_>, _>>()?;
    let first = authored
        .first()
        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
    let before_parameter = authored
        .iter()
        .map(|cut| cut.before_parameter)
        .min_by(f64::total_cmp)
        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
    let after_parameter = authored
        .iter()
        .map(|cut| cut.after_parameter)
        .max_by(f64::total_cmp)
        .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
    Ok(AuthoredRootCluster {
        source,
        root_parameter: first.root_parameter,
        before_parameter,
        after_parameter,
    })
}

fn backward_cyclic_distance(root: f64, before: f64) -> f64 {
    if before <= root {
        root - before
    } else {
        root + (TAU - before)
    }
}

fn forward_cyclic_distance(root: f64, after: f64) -> f64 {
    if after >= root {
        after - root
    } else {
        (TAU - root) + after
    }
}

fn validate_authored_root_order(
    cuts: &[AuthoredRootCluster],
    range: ParamRange,
) -> Result<(), SkewCylinderOpenSpanFailure> {
    for pair in cuts.windows(2) {
        if pair[0].root_parameter == pair[1].root_parameter
            || pair[0].after_parameter >= pair[1].before_parameter
        {
            return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
        }
    }
    if cuts.iter().any(|cut| {
        !range.contains(cut.root_parameter)
            || !range.contains(cut.before_parameter)
            || !range.contains(cut.after_parameter)
    }) {
        return Err(SkewCylinderOpenSpanFailure::RangeMismatch);
    }
    Ok(())
}

fn sweep_topologies(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    cuts: &[AuthoredRootCluster],
) -> Result<([bool; 2], Vec<CutTransition>), SkewCylinderOpenSpanFailure> {
    let mut states = [[SkewCylinderAxialRelation::Below; 2]; 4];
    for (topology_index, topology) in topologies.iter().enumerate() {
        states[topology_index] = if topology.roots().is_empty() {
            topology.open_cell_relations()[0]
        } else {
            let first = cuts
                .iter()
                .flat_map(|cluster| cluster.source.cuts.iter())
                .find(|cut| cut.topology_index == topology_index)
                .ok_or(SkewCylinderOpenSpanFailure::InconsistentTopology)?;
            let count = topology.open_cell_relations().len();
            topology.open_cell_relations()[(first.cyclic_ordinal + count - 1) % count]
        };
    }
    let initial_states = states;
    let initial_inside = sheet_inside(topologies, &states);
    let mut transitions = Vec::with_capacity(cuts.len());
    for cluster in cuts {
        let before_inside = sheet_inside(topologies, &states);
        let at_inside = sheet_inside_at_cluster(topologies, &states, &cluster.source);
        for cut in &cluster.source.cuts {
            let topology = topologies[cut.topology_index];
            let count = topology.open_cell_relations().len();
            let expected_before =
                topology.open_cell_relations()[(cut.cyclic_ordinal + count - 1) % count];
            if states[cut.topology_index] != expected_before {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
            states[cut.topology_index] = topology.open_cell_relations()[cut.cyclic_ordinal];
        }
        let after_inside = sheet_inside(topologies, &states);
        transitions.push(CutTransition {
            cluster: cluster.clone(),
            before_inside,
            at_inside,
            after_inside,
        });
    }
    if states != initial_states {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok((initial_inside, transitions))
}

fn sheet_inside_at_cluster(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    states: &[[SkewCylinderAxialRelation; 2]; 4],
    cluster: &RootCluster,
) -> [bool; 2] {
    core::array::from_fn(|sheet| {
        topologies.iter().enumerate().all(|(index, topology)| {
            let on_bound = cluster
                .cuts
                .iter()
                .any(|cut| cut.topology_index == index && cut.events[sheet].is_some());
            on_bound || states[index][sheet] == required_relation(topology.provenance())
        })
    })
}

fn sheet_inside(
    topologies: &[&SkewCylinderAxialBoundTopology; 4],
    states: &[[SkewCylinderAxialRelation; 2]; 4],
) -> [bool; 2] {
    core::array::from_fn(|sheet| {
        topologies.iter().enumerate().all(|(index, topology)| {
            states[index][sheet] == required_relation(topology.provenance())
        })
    })
}

fn classify_sheet(
    sheet: SkewCylinderSheet,
    initial_inside: bool,
    transitions: &[CutTransition],
    carrier_range: ParamRange,
) -> Result<
    (
        SkewCylinderFiniteSheetTopology,
        Vec<SkewCylinderFiniteWindowRootEvent>,
    ),
    SkewCylinderOpenSpanFailure,
> {
    let sheet_index = sheet_index(sheet);
    let mut root_events = Vec::with_capacity(transitions.len());
    for transition in transitions {
        let before = transition.before_inside[sheet_index];
        let at = transition.at_inside[sheet_index];
        let after = transition.after_inside[sheet_index];
        let kind = if before != after {
            if !at {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
            Some(SkewCylinderFiniteWindowRootEventKind::Boundary)
        } else if before {
            if !at {
                return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
            }
            Some(SkewCylinderFiniteWindowRootEventKind::Contact)
        } else if at {
            Some(SkewCylinderFiniteWindowRootEventKind::Isolated)
        } else {
            None
        };
        if let Some(kind) = kind {
            root_events.push(sheet_root_event(transition, sheet, kind)?);
        }
    }

    let changed = transitions
        .iter()
        .enumerate()
        .filter(|(_, transition)| {
            transition.before_inside[sheet_index] != transition.after_inside[sheet_index]
        })
        .collect::<Vec<_>>();
    if changed.is_empty() {
        let topology = if initial_inside {
            SkewCylinderFiniteSheetTopology::Whole
        } else {
            SkewCylinderFiniteSheetTopology::Outside
        };
        return Ok((topology, root_events));
    }
    if changed.len() % 2 != 0 {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    for (current, next) in changed
        .iter()
        .zip(changed.iter().cycle().skip(1))
        .take(changed.len())
    {
        if current.1.after_inside[sheet_index] == next.1.after_inside[sheet_index] {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
    }

    let mut spans = Vec::with_capacity(changed.len() / 2);
    for (position, (start_index, start_transition)) in changed.iter().enumerate() {
        if start_transition.before_inside[sheet_index]
            || !start_transition.after_inside[sheet_index]
        {
            continue;
        }
        let (end_index, end_transition) = changed[(position + 1) % changed.len()];
        if !end_transition.before_inside[sheet_index] || end_transition.after_inside[sheet_index] {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
        let start_event = sheet_root_event(
            start_transition,
            sheet,
            SkewCylinderFiniteWindowRootEventKind::Boundary,
        )?;
        let end_event = sheet_root_event(
            end_transition,
            sheet,
            SkewCylinderFiniteWindowRootEventKind::Boundary,
        )?;
        let start_roots_valid = (0..start_event.root_count()).all(|index| {
            start_event
                .root(index)
                .is_some_and(|root| root.after == required_relation(root.provenance))
        });
        let end_roots_valid = (0..end_event.root_count()).all(|index| {
            end_event
                .root(index)
                .is_some_and(|root| root.before == required_relation(root.provenance))
        });
        if !start_roots_valid || !end_roots_valid {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        }
        let root_guarded_start = start_transition.cluster.after_parameter;
        let root_guarded_end = end_transition.cluster.before_parameter;
        if end_index <= *start_index && end_transition.cluster.root_parameter != carrier_range.lo {
            return Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan);
        }
        if !root_guarded_start.is_finite()
            || !root_guarded_end.is_finite()
            || root_guarded_start >= root_guarded_end
        {
            return Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots);
        }
        // Reserve one fixed proof-cell fraction at both ends. The two root
        // corridor certificates own those omitted continuations, while this
        // margin keeps the 256-cell residual enclosure strictly inside every
        // authored axial window even at exact endpoint roots.
        let proof_guard =
            (root_guarded_end - root_guarded_start) / SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as f64;
        let start_parameter = (root_guarded_start + proof_guard).next_up();
        let end_parameter = (root_guarded_end - proof_guard).next_down();
        let range = ParamRange::new(start_parameter, end_parameter);
        if !range.is_finite() || range.width() <= 0.0 || range.width() >= TAU {
            return Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan);
        }
        spans.push(SkewCylinderOpenSpan {
            sheet,
            range,
            start: SkewCylinderOpenSpanEndpointProof {
                event: start_event,
                inside_side: SkewCylinderRootInsideSide::After,
                carrier_parameter: start_parameter,
            },
            end: SkewCylinderOpenSpanEndpointProof {
                event: end_event,
                inside_side: SkewCylinderRootInsideSide::Before,
                carrier_parameter: end_parameter,
            },
        });
    }
    spans.sort_by(|lhs, rhs| lhs.range.lo.total_cmp(&rhs.range.lo));
    if spans.is_empty() {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok((SkewCylinderFiniteSheetTopology::Open(spans), root_events))
}

fn sheet_root_event(
    transition: &CutTransition,
    sheet: SkewCylinderSheet,
    kind: SkewCylinderFiniteWindowRootEventKind,
) -> Result<SkewCylinderFiniteWindowRootEvent, SkewCylinderOpenSpanFailure> {
    let sheet_index = sheet_index(sheet);
    let mut roots = [None; SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER];
    let mut root_count = 0_usize;
    for cut in &transition.cluster.source.cuts {
        let Some(root) = cut.events[sheet_index] else {
            continue;
        };
        let Some(slot) = roots.get_mut(root_count) else {
            return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
        };
        *slot = Some(root);
        root_count += 1;
    }
    if root_count == 0 {
        return Err(SkewCylinderOpenSpanFailure::InconsistentTopology);
    }
    Ok(SkewCylinderFiniteWindowRootEvent {
        sheet,
        kind,
        roots,
        root_count: root_count as u8,
        carrier_parameter: transition.cluster.root_parameter,
    })
}

fn required_relation(provenance: SkewCylinderAxialBoundProvenance) -> SkewCylinderAxialRelation {
    match provenance.boundary {
        SkewCylinderAxialBoundary::Lower => SkewCylinderAxialRelation::Above,
        SkewCylinderAxialBoundary::Upper => SkewCylinderAxialRelation::Below,
    }
}

const fn sheet_index(sheet: SkewCylinderSheet) -> usize {
    match sheet {
        SkewCylinderSheet::Lower => 0,
        SkewCylinderSheet::Upper => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::super::{SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, classify_skew_cylinder_axial_bound};
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;
    use kgeom::vec::{Point3, Vec3};

    fn perpendicular_pair(offset: f64) -> [Cylinder; 2] {
        [
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(0.0, offset, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
                2.0,
            )
            .unwrap(),
        ]
    }

    fn topologies(
        cylinders: [Cylinder; 2],
        ranges: [[ParamRange; 2]; 2],
    ) -> [SkewCylinderAxialBoundTopology; 4] {
        [
            (0, SkewCylinderAxialBoundary::Lower, ranges[0][1].lo),
            (0, SkewCylinderAxialBoundary::Upper, ranges[0][1].hi),
            (1, SkewCylinderAxialBoundary::Lower, ranges[1][1].lo),
            (1, SkewCylinderAxialBoundary::Upper, ranges[1][1].hi),
        ]
        .map(|(source_operand, boundary, value)| {
            classify_skew_cylinder_axial_bound(
                cylinders,
                [0, 1],
                SkewCylinderAxialBoundProvenance {
                    source_operand,
                    boundary,
                    value,
                },
                SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            )
            .unwrap()
        })
    }

    fn clipped_ranges(longitude: ParamRange) -> [[ParamRange; 2]; 2] {
        [
            [longitude, ParamRange::new(1.8, 2.1)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 0.0)],
        ]
    }

    fn classify(
        topologies: &[SkewCylinderAxialBoundTopology; 4],
        ranges: [[ParamRange; 2]; 2],
    ) -> Result<SkewCylinderFiniteWindowTopologyCertificate, SkewCylinderOpenSpanFailure> {
        classify_skew_cylinder_open_spans(SkewCylinderOpenSpanTopologyInput {
            topologies,
            ranges,
            canonical_to_source: [0, 1],
        })
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= 1.0e-12,
            "{actual:.17e} != {expected:.17e}"
        );
    }

    fn primary_root(endpoint: SkewCylinderOpenSpanEndpointProof) -> SkewCylinderAxialRoot {
        endpoint
            .event
            .root(0)
            .expect("every span endpoint owns a physical root")
    }

    #[test]
    fn clipped_perpendicular_window_has_one_upper_nonwrapping_span() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, ranges);
        let result = classify(&source, ranges).unwrap();
        assert_eq!(result.root_cluster_query_plan().bits(), 0);

        assert_eq!(
            result.sheet(SkewCylinderSheet::Lower),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let SkewCylinderFiniteSheetTopology::Open(spans) = result.sheet(SkewCylinderSheet::Upper)
        else {
            panic!("{result:?}");
        };
        assert_eq!(spans.len(), 1);
        let span = spans[0];
        assert_eq!(span.sheet, SkewCylinderSheet::Upper);
        assert_close(span.range.lo, 2.091041074522298);
        assert_close(span.range.hi, 4.192144232657288);
        assert_eq!(span.start.event.root_count(), 1);
        assert_eq!(
            primary_root(span.start).provenance,
            SkewCylinderAxialBoundProvenance {
                source_operand: 0,
                boundary: SkewCylinderAxialBoundary::Lower,
                value: 1.8,
            }
        );
        assert_eq!(
            primary_root(span.end).provenance,
            primary_root(span.start).provenance
        );
        assert_eq!(span.start.inside_side, SkewCylinderRootInsideSide::After);
        assert_eq!(span.end.inside_side, SkewCylinderRootInsideSide::Before);
        assert_eq!(span.start.carrier_parameter, span.range.lo);
        assert_eq!(span.end.carrier_parameter, span.range.hi);
        assert!(!primary_root(span.start).repeated && !primary_root(span.end).repeated);
        assert_ne!(
            primary_root(span.start).before,
            primary_root(span.start).after
        );
        assert_ne!(primary_root(span.end).before, primary_root(span.end).after);
        let [lower_root, upper_root] = span
            .root_longitude_intervals(ranges[0][0])
            .expect("both projective roots must lift into the retained longitude chart");
        assert!(ranges[0][0].lo < lower_root.lo());
        assert!(lower_root.hi() < span.range.lo);
        assert!(span.range.hi < upper_root.lo());
        assert!(upper_root.hi() < ranges[0][0].hi);
    }

    #[test]
    fn two_active_axial_windows_have_four_upper_nonwrapping_spans() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(1.8, 1.9)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 1.25)],
        ];
        let mut source = topologies(cylinders, ranges);
        let plan = plan_skew_cylinder_root_clusters(SkewCylinderOpenSpanTopologyInput {
            topologies: &source,
            ranges,
            canonical_to_source: [0, 1],
        })
        .unwrap();
        assert_eq!(plan.bits(), 0);
        let result = classify(&source, ranges).unwrap();
        assert_eq!(result.root_cluster_query_plan(), plan);
        source.reverse();
        assert_eq!(
            plan_skew_cylinder_root_clusters(SkewCylinderOpenSpanTopologyInput {
                topologies: &source,
                ranges,
                canonical_to_source: [0, 1],
            })
            .unwrap(),
            plan
        );
        assert_eq!(classify(&source, ranges).unwrap(), result);

        assert_eq!(
            result.sheet(SkewCylinderSheet::Lower),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let SkewCylinderFiniteSheetTopology::Open(spans) = result.sheet(SkewCylinderSheet::Upper)
        else {
            panic!("{result:?}");
        };
        assert_eq!(spans.len(), 4);
        for (ordinal, span) in spans.iter().enumerate() {
            assert_eq!(span.sheet, SkewCylinderSheet::Upper);
            assert!(0.0 < span.range.lo && span.range.hi < TAU);
            assert!(0.0 < span.range.width() && span.range.width() < TAU);
            assert_eq!(primary_root(span.start).cyclic_ordinal, ordinal);
            assert_eq!(primary_root(span.end).cyclic_ordinal, ordinal);
            assert_eq!(
                primary_root(span.start).provenance.source_operand,
                primary_root(span.end).provenance.source_operand
            );
            assert_eq!(primary_root(span.start).provenance.source_operand, 0);
            assert_ne!(
                primary_root(span.start).provenance.boundary,
                primary_root(span.end).provenance.boundary
            );
            assert_eq!(span.start.carrier_parameter, span.range.lo);
            assert_eq!(span.end.carrier_parameter, span.range.hi);
            assert_eq!(span.start.inside_side, SkewCylinderRootInsideSide::After);
            assert_eq!(span.end.inside_side, SkewCylinderRootInsideSide::Before);
            assert!(!primary_root(span.start).repeated && !primary_root(span.end).repeated);
            assert_ne!(
                primary_root(span.start).before,
                primary_root(span.start).after
            );
            assert_ne!(primary_root(span.end).before, primary_root(span.end).after);
            let [lower_root, upper_root] = span
                .root_longitude_intervals(ranges[0][0])
                .expect("each projective root pair must lift without wrapping");
            assert!(lower_root.hi() < span.range.lo);
            assert!(span.range.hi < upper_root.lo());
        }
        for pair in spans.windows(2) {
            assert!(pair[0].range.hi < pair[1].range.lo);
        }
    }

    #[test]
    fn two_exact_corner_clusters_retain_two_spans_and_two_isolated_points() {
        let cylinders = [
            Cylinder::new(Frame::world(), 13.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(0.0, 0.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                )
                .unwrap(),
                20.0,
            )
            .unwrap(),
        ];
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(16.0, 17.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-14.0, 5.0)],
        ];
        let mut source = topologies(cylinders, ranges);
        let plan = plan_skew_cylinder_root_clusters(SkewCylinderOpenSpanTopologyInput {
            topologies: &source,
            ranges,
            canonical_to_source: [0, 1],
        })
        .unwrap();
        assert_eq!(plan.query_count(), 1);
        assert_eq!(
            plan.work(),
            SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
        );
        let result = classify(&source, ranges).unwrap();
        assert_eq!(result.root_cluster_query_plan(), plan);
        source.reverse();
        assert_eq!(
            plan_skew_cylinder_root_clusters(SkewCylinderOpenSpanTopologyInput {
                topologies: &source,
                ranges,
                canonical_to_source: [0, 1],
            })
            .unwrap(),
            plan
        );
        assert_eq!(classify(&source, ranges).unwrap(), result);
        assert_eq!(
            result.sheet(SkewCylinderSheet::Lower),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let SkewCylinderFiniteSheetTopology::Open(spans) = result.sheet(SkewCylinderSheet::Upper)
        else {
            panic!("{result:#?}");
        };
        assert_eq!(spans.len(), 2);
        assert!(spans.iter().all(|span| {
            span.start.event.root_count() == 1
                && span.end.event.root_count() == 1
                && span.start.event.kind() == SkewCylinderFiniteWindowRootEventKind::Boundary
                && span.end.event.kind() == SkewCylinderFiniteWindowRootEventKind::Boundary
        }));

        let events = result.root_events(SkewCylinderSheet::Upper);
        assert_eq!(events.len(), 6);
        let isolated = events
            .iter()
            .copied()
            .filter(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Isolated)
            .collect::<Vec<_>>();
        assert_eq!(isolated.len(), 2);
        for event in isolated {
            assert_eq!(event.root_count(), 2);
            let roots = [event.root(0).unwrap(), event.root(1).unwrap()];
            assert_eq!(
                roots.map(|root| (root.provenance.source_operand, root.provenance.boundary)),
                [
                    (0, SkewCylinderAxialBoundary::Lower),
                    (1, SkewCylinderAxialBoundary::Upper),
                ]
            );
            assert!(roots.iter().all(|root| {
                root.bracket.chart == SkewCylinderHalfAngleChart::Tangent
                    && ((0.5 * root.bracket.lo + 0.5 * root.bracket.hi).abs() - 2.0 / 3.0).abs()
                        <= 1.0e-12
            }));
        }
    }

    #[test]
    fn occupied_component_crossing_the_authored_seam_is_refused() {
        let cylinders = perpendicular_pair(0.0);
        let canonical_ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, canonical_ranges);
        let shifted_ranges = clipped_ranges(ParamRange::new(
            -core::f64::consts::PI,
            core::f64::consts::PI,
        ));

        assert_eq!(
            classify(&source, shifted_ranges),
            Err(SkewCylinderOpenSpanFailure::SeamWrappingSpan)
        );
    }

    #[test]
    fn repeated_axial_contact_roots_are_retained_without_splitting_whole_sheets() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(-3.0, 3.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
        ];
        let source = topologies(cylinders, ranges);
        let result = classify(&source, ranges).unwrap();
        for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
            assert_eq!(result.sheet(sheet), &SkewCylinderFiniteSheetTopology::Whole);
            let events = result.root_events(sheet);
            assert_eq!(events.len(), 2);
            assert!(events.iter().all(|event| {
                event.kind() == SkewCylinderFiniteWindowRootEventKind::Contact
                    && event.root_count() == 1
                    && event.root(0).is_some_and(|root| root.repeated)
            }));
        }
    }

    #[test]
    fn source_bound_range_mismatch_is_refused() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = clipped_ranges(ParamRange::new(0.0, TAU));
        let source = topologies(cylinders, ranges);
        let mut mismatched_ranges = ranges;
        mismatched_ranges[0][1].lo = 1.8_f64.next_up();
        assert_eq!(
            classify(&source, mismatched_ranges),
            Err(SkewCylinderOpenSpanFailure::RangeMismatch)
        );
    }

    #[test]
    fn coincident_zero_width_bounds_become_exact_isolated_events() {
        let cylinders = perpendicular_pair(0.0);
        let ranges = [
            [ParamRange::new(0.0, TAU), ParamRange::new(1.8, 1.8)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.25, 0.0)],
        ];
        let source = topologies(cylinders, ranges);
        let result = classify(&source, ranges).unwrap();
        assert_eq!(
            result.sheet(SkewCylinderSheet::Upper),
            &SkewCylinderFiniteSheetTopology::Outside
        );
        let events = result.root_events(SkewCylinderSheet::Upper);
        assert_eq!(events.len(), 2);
        assert!(events.iter().all(|event| {
            event.kind() == SkewCylinderFiniteWindowRootEventKind::Isolated
                && event.root_count() == 2
                && event.root(0).is_some_and(|root| {
                    root.provenance.boundary == SkewCylinderAxialBoundary::Lower
                })
                && event.root(1).is_some_and(|root| {
                    root.provenance.boundary == SkewCylinderAxialBoundary::Upper
                })
        }));
    }

    #[test]
    fn expanded_guard_overlap_is_refused_before_range_construction() {
        let provenance = SkewCylinderAxialBoundProvenance {
            source_operand: 0,
            boundary: SkewCylinderAxialBoundary::Lower,
            value: 1.8,
        };
        let root = |cyclic_ordinal, before, after| SkewCylinderAxialRoot {
            provenance,
            sheet: SkewCylinderSheet::Upper,
            cyclic_ordinal,
            bracket: SkewCylinderHalfAngleRootBracket {
                chart: SkewCylinderHalfAngleChart::Tangent,
                lo: 0.25,
                hi: 0.25,
            },
            repeated: false,
            before,
            after,
        };
        let authored_cluster =
            |root: SkewCylinderAxialRoot, root_parameter, before_parameter, after_parameter| {
                AuthoredRootCluster {
                    source: RootCluster {
                        cuts: vec![RootCut {
                            topology_index: 0,
                            cyclic_ordinal: root.cyclic_ordinal,
                            bracket: root.bracket,
                            events: [None, Some(root)],
                        }],
                    },
                    root_parameter,
                    before_parameter,
                    after_parameter,
                }
            };
        let transitions = [
            CutTransition {
                cluster: authored_cluster(
                    root(
                        0,
                        SkewCylinderAxialRelation::Below,
                        SkewCylinderAxialRelation::Above,
                    ),
                    0.9,
                    0.8,
                    2.0,
                ),
                before_inside: [false, false],
                at_inside: [false, true],
                after_inside: [false, true],
            },
            CutTransition {
                cluster: authored_cluster(
                    root(
                        1,
                        SkewCylinderAxialRelation::Above,
                        SkewCylinderAxialRelation::Below,
                    ),
                    2.1,
                    1.0,
                    2.2,
                ),
                before_inside: [false, true],
                at_inside: [false, true],
                after_inside: [false, false],
            },
        ];

        assert_eq!(
            classify_sheet(
                SkewCylinderSheet::Upper,
                false,
                &transitions,
                ParamRange::new(0.0, TAU),
            ),
            Err(SkewCylinderOpenSpanFailure::CoincidentOrOverlappingRoots)
        );
    }
}
