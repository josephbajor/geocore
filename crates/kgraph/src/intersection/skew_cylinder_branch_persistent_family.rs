//! Complete finite-window families for persistent skew-cylinder spans.
//!
//! Four exact axial-bound equations determine the complete finite occupancy
//! of the two strict-positive sheets.  Each equation is cyclic second
//! harmonic and therefore has at most four distinct cyclic cuts.  The v2
//! sweep therefore has at most sixteen physical cuts. Each cut can contribute
//! at most two sheet-owned events, while every non-wrapping open
//! component consumes two events. The v2 representation consequently has
//! room for exactly sixteen open members; this is an analytic degree bound,
//! not a sampling or defensive limit.

use kcore::interval::Interval;
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgeom::vec::{Vec2, Vec3};

use super::*;

#[path = "skew_cylinder_branch_persistent_family_reissue.rs"]
mod reissue;
pub use reissue::{
    PersistentSkewCylinderFiniteWindowFamilyReissue,
    reissue_persistent_skew_cylinder_finite_window_family,
};

/// Schema version for strict-positive finite-window skew-cylinder families.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_FAMILY_VERSION: u16 = 2;

/// Maximum open members across both sheets under the analytic event bound.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS: usize = 16;

/// Maximum sheet-owned root events retained by one four-cut bound.
///
/// A shared-height cut may belong to both sheets, hence `2 * 4`.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_BOUND: usize = 8;

/// Maximum distinct physical root events retained on one sheet.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_SHEET: usize = 16;

/// Maximum open cells retained by one second-harmonic axial-bound outcome.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND: usize = 4;

/// Already-paid proof work before any bounded open member is certified.
///
/// This is `2*64` strict-positive admission work, `2*256` attempted whole
/// sheet proofs, and `4*64` complete axial-bound occupancy work.
pub const PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_FAMILY_BASE_WORK: u64 = 896;

/// Caller-authored axial side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderAxialBoundary {
    /// Low end of a source cylinder's axial window.
    Lower,
    /// High end of a source cylinder's axial window.
    Upper,
}

/// Exact source-window slot and side.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentSkewCylinderAxialBoundTag {
    source_slot: u8,
    boundary: PersistentSkewCylinderAxialBoundary,
}

impl PersistentSkewCylinderAxialBoundTag {
    /// Construct one caller-order source-window tag.
    pub const fn new(
        source_slot: usize,
        boundary: PersistentSkewCylinderAxialBoundary,
    ) -> Option<Self> {
        if source_slot < 2 {
            Some(Self {
                source_slot: source_slot as u8,
                boundary,
            })
        } else {
            None
        }
    }

    /// Source cylinder slot in caller/live dependency order.
    pub const fn source_slot(self) -> usize {
        self.source_slot as usize
    }

    /// Authored side of that source's axial window.
    pub const fn boundary(self) -> PersistentSkewCylinderAxialBoundary {
        self.boundary
    }
}

/// Strict sheet relation to one authored axial bound.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderAxialRelation {
    /// Sheet height is strictly below the bound.
    Below,
    /// Sheet height is strictly above the bound.
    Above,
}

/// Projective half-angle chart owning an exact root enclosure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderHalfAngleChart {
    /// Tangent half-angle chart.
    Tangent,
    /// Cotangent half-angle chart.
    Cotangent,
}

/// Side of a physical-root corridor retained by an open member.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderRootInsideSide {
    /// Increasing-longitude side before the root.
    Before,
    /// Increasing-longitude side after the root.
    After,
}

/// One exact-source root event supplied by the exact axial-bound classifier.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderAxialRootEventInput {
    /// Caller-order axial-bound identity.
    pub tag: PersistentSkewCylinderAxialBoundTag,
    /// Exact caller-authored bound.
    pub bound: f64,
    /// Sheet owning this root.
    pub sheet: SkewCylinderSheet,
    /// Ordinal among the bound equation's distinct cyclic cuts.
    pub cyclic_ordinal: usize,
    /// Projective chart retaining exact-source identity.
    pub half_angle_chart: PersistentSkewCylinderHalfAngleChart,
    /// Isolating interval in that projective chart.
    pub half_angle_bracket: [f64; 2],
    /// Strict relation immediately before the cut.
    pub before: PersistentSkewCylinderAxialRelation,
    /// Strict relation immediately after the cut.
    pub after: PersistentSkewCylinderAxialRelation,
    /// Whether this sheet root has even multiplicity and preserves its strict
    /// relation across the cut.
    pub repeated: bool,
}

/// Closed-set role of one persistent physical finite-window event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderFiniteWindowRootEventKind {
    /// Open occupancy changes across the event.
    Boundary,
    /// Open occupancy remains inside on both sides.
    Contact,
    /// Open occupancy is outside on both sides but the event itself is inside
    /// every closed authored bound.
    Isolated,
}

/// Persistent exact-source roots grouped at one physical sheet event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowRootEvent {
    sheet: SkewCylinderSheet,
    kind: PersistentSkewCylinderFiniteWindowRootEventKind,
    roots: [PersistentSkewCylinderRootReference;
        SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER],
    root_count: u8,
    carrier_parameter: f64,
}

impl PersistentSkewCylinderFiniteWindowRootEvent {
    /// Ordered quadratic sheet owning this event.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.sheet
    }

    /// Closed-set role of this event.
    pub const fn kind(self) -> PersistentSkewCylinderFiniteWindowRootEventKind {
        self.kind
    }

    /// Number of exact bound roots grouped at this event.
    pub const fn root_count(self) -> usize {
        self.root_count as usize
    }

    const fn root_reference(self, index: usize) -> Option<PersistentSkewCylinderRootReference> {
        if index < self.root_count as usize {
            Some(self.roots[index])
        } else {
            None
        }
    }

    /// Deterministic carrier-chart representative of the physical root.
    pub const fn carrier_parameter(self) -> f64 {
        self.carrier_parameter
    }
}

/// Compact reference into one already-owned persistent bound outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PersistentSkewCylinderRootReference {
    outcome: u8,
    root: u8,
}

const EMPTY_ROOT_REFERENCE: PersistentSkewCylinderRootReference =
    PersistentSkewCylinderRootReference {
        outcome: 0,
        root: 0,
    };

/// Exact root references plus the retained finite-window side.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowEndpointProof {
    roots: [PersistentSkewCylinderRootReference;
        SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER],
    root_count: u8,
    sheet: SkewCylinderSheet,
    inside_side: PersistentSkewCylinderRootInsideSide,
    inside_parameter: f64,
}

impl PersistentSkewCylinderFiniteWindowEndpointProof {
    const fn new(
        roots: [PersistentSkewCylinderRootReference;
            SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER],
        root_count: usize,
        sheet: SkewCylinderSheet,
        inside_side: PersistentSkewCylinderRootInsideSide,
        inside_parameter: f64,
    ) -> Self {
        Self {
            roots,
            root_count: root_count as u8,
            sheet,
            inside_side,
            inside_parameter,
        }
    }

    /// Number of exact bound roots active at this physical endpoint.
    pub const fn root_count(self) -> usize {
        self.root_count as usize
    }

    const fn root_reference(self, index: usize) -> Option<PersistentSkewCylinderRootReference> {
        if index < self.root_count as usize {
            Some(self.roots[index])
        } else {
            None
        }
    }

    /// Sheet owning the endpoint.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.sheet
    }

    /// Retained side of the exact root.
    pub const fn inside_side(self) -> PersistentSkewCylinderRootInsideSide {
        self.inside_side
    }

    /// Representable parameter on the retained side.
    pub const fn inside_parameter(self) -> f64 {
        self.inside_parameter
    }
}

/// Operation-owned input for one fully certified open member.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowMemberInput {
    /// Compact whole-span residual proof.
    pub residual: PairedSkewCylinderBranchResidualCertificate,
    /// Physical-root continuation proofs in lower/upper canonical order.
    pub root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
}

/// Sealed complete outcome for one axial bound.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderAxialBoundOutcome {
    tag: PersistentSkewCylinderAxialBoundTag,
    bound: f64,
    roots: [Option<PersistentSkewCylinderAxialRootEventInput>;
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_BOUND],
    open_cell_relations: [Option<[PersistentSkewCylinderAxialRelation; 2]>;
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND],
    root_count: u8,
    open_cell_count: u8,
}

impl PersistentSkewCylinderAxialBoundOutcome {
    /// Caller-order bound identity.
    pub const fn tag(self) -> PersistentSkewCylinderAxialBoundTag {
        self.tag
    }

    /// Exact caller-authored bound.
    pub const fn bound(self) -> f64 {
        self.bound
    }

    /// Number of sheet-owned root events.
    pub const fn root_count(self) -> usize {
        self.root_count as usize
    }

    /// Root event by packed index.
    pub const fn root(self, index: usize) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        if index < self.root_count as usize {
            self.roots[index]
        } else {
            None
        }
    }

    /// Number of distinct cyclic open cells.
    pub const fn open_cell_count(self) -> usize {
        self.open_cell_count as usize
    }

    /// Two-sheet relation on one cyclic open cell.
    pub const fn open_cell_relations(
        self,
        index: usize,
    ) -> Option<[PersistentSkewCylinderAxialRelation; 2]> {
        if index < self.open_cell_count as usize {
            self.open_cell_relations[index]
        } else {
            None
        }
    }
}

/// Sealed compact evidence for one deterministic open family member.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowMemberCertificate {
    ordinal: u8,
    sheet: SkewCylinderSheet,
    guarded_range: ParamRange,
    root_parameter_enclosures: [Interval; 2],
    endpoints: [PersistentSkewCylinderFiniteWindowEndpointProof; 2],
    carrier_box: Aabb3,
    pcurve_boxes: [Aabb2; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
}

impl PersistentSkewCylinderFiniteWindowMemberCertificate {
    /// Immutable ordinal in `(sheet, guarded/root range)` order.
    pub const fn ordinal(self) -> usize {
        self.ordinal as usize
    }

    /// Ordered quadratic sheet.
    pub const fn sheet(self) -> SkewCylinderSheet {
        self.sheet
    }

    /// Complete guarded span range.
    pub const fn guarded_range(self) -> ParamRange {
        self.guarded_range
    }

    /// Full physical-root enclosures in lower/upper canonical order.
    pub const fn root_parameter_enclosures(self) -> [Interval; 2] {
        self.root_parameter_enclosures
    }

    /// Exact endpoint tags and retained slab sides.
    pub const fn endpoints(self) -> [PersistentSkewCylinderFiniteWindowEndpointProof; 2] {
        self.endpoints
    }

    /// Complete carrier enclosure including both root corridors.
    pub const fn carrier_box(self) -> Aabb3 {
        self.carrier_box
    }

    /// Complete pcurve boxes including both root corridors, in source order.
    pub const fn pcurve_boxes(self) -> [Aabb2; 2] {
        self.pcurve_boxes
    }

    /// Whole guarded/corridor paired residual envelopes in source order.
    pub const fn residual_bounds(self) -> [f64; 2] {
        self.residual_bounds
    }

    /// Sealed model-space proof tolerance.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }
}

/// Sealed finite occupancy of one strict-positive sheet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderFiniteWindowSheetOccupancy {
    /// No member lies inside all four axial windows.
    Outside,
    /// The complete periodic sheet lies inside all four axial windows.
    Whole,
    /// Deterministic contiguous ordinal range of open members.
    Open {
        /// First family ordinal on this sheet.
        first_member_ordinal: usize,
        /// Exact member count on this sheet.
        member_count: usize,
    },
}

/// Complete, sealed finite-window intersection family.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowFamilyCertificate {
    admission: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    formula_cylinders: [Cylinder; 2],
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    axial_bound_outcomes: [PersistentSkewCylinderAxialBoundOutcome; 4],
    root_cluster_query_plan: SkewCylinderRootClusterQueryPlan,
    root_events: [[Option<PersistentSkewCylinderFiniteWindowRootEvent>;
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_SHEET]; 2],
    root_event_counts: [u8; 2],
    sheet_occupancy: [PersistentSkewCylinderFiniteWindowSheetOccupancy; 2],
    members: [Option<PersistentSkewCylinderFiniteWindowMemberCertificate>;
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS],
    member_count: u8,
    tolerance: f64,
}

impl PersistentSkewCylinderFiniteWindowFamilyCertificate {
    /// Certificate schema version.
    pub const fn version(self) -> u16 {
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_FAMILY_VERSION
    }

    /// Exact strict-positive admission retained by this family.
    pub const fn admission(self) -> SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
        self.admission
    }

    /// Exact cylinders in formula/ruling order.
    pub const fn formula_cylinders(self) -> [Cylinder; 2] {
        self.formula_cylinders
    }

    /// Exact authored windows in formula/ruling order.
    pub const fn formula_windows(self) -> [[ParamRange; 2]; 2] {
        self.formula_windows
    }

    /// Formula-slot to caller/live-source-slot permutation.
    pub const fn formula_to_source(self) -> [usize; 2] {
        self.formula_to_source
    }

    /// Exact cylinders in caller/live dependency order.
    pub fn source_cylinders(self) -> [Cylinder; 2] {
        permute_formula_to_source(self.formula_cylinders, self.formula_to_source)
    }

    /// Exact authored windows in caller/live dependency order.
    pub fn source_windows(self) -> [[ParamRange; 2]; 2] {
        permute_formula_to_source(self.formula_windows, self.formula_to_source)
    }

    /// Complete bound outcome in formula-window order:
    /// `[first lower, first upper, second lower, second upper]`.
    pub const fn axial_bound_outcome(
        self,
        index: usize,
    ) -> Option<PersistentSkewCylinderAxialBoundOutcome> {
        if index < 4 {
            Some(self.axial_bound_outcomes[index])
        } else {
            None
        }
    }

    /// Exact bound-pair/chart work plan represented by this family.
    pub const fn root_cluster_query_plan(&self) -> SkewCylinderRootClusterQueryPlan {
        self.root_cluster_query_plan
    }

    /// Number of persistent physical root events on one sheet.
    pub const fn root_event_count(&self, sheet: SkewCylinderSheet) -> usize {
        self.root_event_counts[sheet_index(sheet)] as usize
    }

    /// Persistent physical root event by sheet-local ordinal.
    pub const fn root_event(
        &self,
        sheet: SkewCylinderSheet,
        ordinal: usize,
    ) -> Option<PersistentSkewCylinderFiniteWindowRootEvent> {
        let sheet = sheet_index(sheet);
        if ordinal < self.root_event_counts[sheet] as usize {
            self.root_events[sheet][ordinal]
        } else {
            None
        }
    }

    /// Exact bound root grouped into one persistent physical event.
    pub fn root_event_root(
        &self,
        sheet: SkewCylinderSheet,
        event_ordinal: usize,
        root_ordinal: usize,
    ) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        let event = self.root_event(sheet, event_ordinal)?;
        self.resolve_root_reference(event.root_reference(root_ordinal)?)
    }

    /// Exact bound root grouped into one member endpoint.
    pub fn member_endpoint_root(
        &self,
        member_ordinal: usize,
        endpoint_ordinal: usize,
        root_ordinal: usize,
    ) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        let member = self.members.get(member_ordinal).copied().flatten()?;
        let endpoint = member.endpoints.get(endpoint_ordinal)?;
        self.resolve_root_reference(endpoint.root_reference(root_ordinal)?)
    }

    fn resolve_root_reference(
        &self,
        reference: PersistentSkewCylinderRootReference,
    ) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        self.axial_bound_outcomes
            .get(reference.outcome as usize)?
            .root(reference.root as usize)
    }

    /// Complete occupancy for Lower or Upper.
    pub const fn sheet_occupancy(
        self,
        sheet: SkewCylinderSheet,
    ) -> PersistentSkewCylinderFiniteWindowSheetOccupancy {
        self.sheet_occupancy[sheet_index(sheet)]
    }

    /// Exact number of open members.
    pub const fn member_count(self) -> usize {
        self.member_count as usize
    }

    /// Deterministic member by ordinal.
    pub const fn member(
        self,
        ordinal: usize,
    ) -> Option<PersistentSkewCylinderFiniteWindowMemberCertificate> {
        if ordinal < self.member_count as usize {
            self.members[ordinal]
        } else {
            None
        }
    }

    /// Common model-space certification tolerance.
    pub const fn tolerance(self) -> f64 {
        self.tolerance
    }

    /// Existing logical work represented by the complete family.
    pub const fn work(self) -> u64 {
        PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_FAMILY_BASE_WORK
            + self.root_cluster_query_plan.work()
            + self.member_count as u64 * PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
    }

    /// Bind one exact member ordinal for carriage through persistence.
    pub const fn membership(
        self,
        ordinal: usize,
    ) -> Option<PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate> {
        if ordinal < self.member_count as usize {
            Some(
                PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate {
                    family: self,
                    ordinal: ordinal as u8,
                },
            )
        } else {
            None
        }
    }
}

/// Complete family plus one immutable represented ordinal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate {
    family: PersistentSkewCylinderFiniteWindowFamilyCertificate,
    ordinal: u8,
}

impl PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate {
    /// Complete finite-window family.
    pub const fn family(self) -> PersistentSkewCylinderFiniteWindowFamilyCertificate {
        self.family
    }

    /// Immutable represented ordinal.
    pub const fn ordinal(self) -> usize {
        self.ordinal as usize
    }

    /// Compact member evidence selected by the ordinal.
    pub const fn member(self) -> PersistentSkewCylinderFiniteWindowMemberCertificate {
        match self.family.member(self.ordinal as usize) {
            Some(member) => member,
            None => panic!("sealed family membership always names one member"),
        }
    }

    /// Exact bound root grouped into one endpoint of this member.
    pub fn endpoint_root(
        &self,
        endpoint_ordinal: usize,
        root_ordinal: usize,
    ) -> Option<PersistentSkewCylinderAxialRootEventInput> {
        self.family
            .member_endpoint_root(self.ordinal as usize, endpoint_ordinal, root_ordinal)
    }
}

/// Mint a complete finite-window family without adding proof work.
///
/// The operation layer supplies the four already-certified exact bound
/// outcomes and every already-certified open branch. This function validates
/// their exact formula/source identity, deterministic ordering, endpoint-slab
/// tags, occupancy counts, and compact whole-member enclosures.
pub fn certify_persistent_skew_cylinder_finite_window_family(
    admission: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    finite_topology: &SkewCylinderFiniteWindowTopologyCertificate,
    members: &[PersistentSkewCylinderFiniteWindowMemberInput],
    tolerance: f64,
) -> Result<PersistentSkewCylinderFiniteWindowFamilyCertificate, IntersectionCertificateError> {
    let formula_cylinders = finite_topology.formula_cylinders();
    let formula_windows = finite_topology.formula_ranges();
    let formula_to_source = finite_topology.formula_to_source();
    validate_family_header(
        admission,
        formula_cylinders,
        formula_windows,
        formula_to_source,
        tolerance,
    )?;
    let source_cylinders = permute_formula_to_source(formula_cylinders, formula_to_source);
    let source_windows = permute_formula_to_source(formula_windows, formula_to_source);
    let outcomes = certify_bound_outcomes(
        finite_topology.bound_topologies(),
        formula_cylinders,
        formula_to_source,
        source_windows,
    )?;
    let derived_members = derived_open_members(finite_topology);
    if members.is_empty()
        || members.len() != derived_members.len()
        || members.len() > PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }

    let geometry = FamilyMemberGeometry {
        formula_cylinders,
        formula_windows,
        source_cylinders,
    };
    let mut certified_members = [None; PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_MEMBERS];
    for (ordinal, (input, derived)) in members.iter().copied().zip(derived_members).enumerate() {
        certified_members[ordinal] = Some(certify_family_member(
            ordinal, input, derived, geometry, &outcomes, tolerance,
        )?);
    }
    validate_member_order(&certified_members[..members.len()])?;
    let occupancy = certify_sheet_occupancy(finite_topology, members.len())?;
    let (root_events, root_event_counts) = certify_root_events(finite_topology, &outcomes)?;

    Ok(PersistentSkewCylinderFiniteWindowFamilyCertificate {
        admission,
        formula_cylinders,
        formula_windows,
        formula_to_source,
        axial_bound_outcomes: outcomes,
        root_cluster_query_plan: finite_topology.root_cluster_query_plan(),
        root_events,
        root_event_counts,
        sheet_occupancy: occupancy,
        members: certified_members,
        member_count: members.len() as u8,
        tolerance,
    })
}

pub(super) fn validate_finite_window_family_membership(
    membership: PersistentSkewCylinderFiniteWindowFamilyMembershipCertificate,
    residual: PairedSkewCylinderBranchResidualCertificate,
    root_corridors: [SkewCylinderBranchPcurveRootCorridorCertificate; 2],
) -> Result<(), IntersectionCertificateError> {
    let family = membership.family();
    let member = membership.member();
    let input = PersistentSkewCylinderFiniteWindowMemberInput {
        residual,
        root_corridors,
    };
    validate_member_input_identity(
        input,
        FamilyMemberGeometry {
            formula_cylinders: family.formula_cylinders,
            formula_windows: family.formula_windows,
            source_cylinders: family.source_cylinders(),
        },
        family.tolerance,
    )?;
    let guarded = residual.carrier_range();
    let root_parameters = root_corridors.map(|corridor| corridor.root_parameter());
    let (carrier_box, pcurve_boxes, residual_bounds) = member_enclosures(input)?;
    if member.ordinal() != membership.ordinal()
        || member.sheet() != residual.sheet()
        || !exact_range(member.guarded_range(), guarded)
        || !exact_intervals(member.root_parameter_enclosures(), root_parameters)
        || !exact_aabb3(member.carrier_box(), carrier_box)
        || !exact_aabb2s(member.pcurve_boxes(), pcurve_boxes)
        || !exact_f64s(member.residual_bounds(), residual_bounds)
        || member.tolerance().to_bits() != residual.tolerance().to_bits()
        || !member
            .endpoints()
            .into_iter()
            .zip(root_corridors)
            .all(|(endpoint, corridor)| {
                endpoint_root_pcurves_contain_bounds(
                    corridor,
                    endpoint,
                    &family.axial_bound_outcomes,
                )
            })
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(())
}

fn validate_family_header(
    admission: SkewCylinderStrictPositiveTwoSheetAdmissionCertificate,
    cylinders: [Cylinder; 2],
    windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
) -> Result<(), IntersectionCertificateError> {
    if !exact_cylinders(admission.formula_cylinders(), cylinders)
        || !matches!(formula_to_source, [0, 1] | [1, 0])
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(IntersectionCertificateError::InvalidTolerance);
    }
    if !cylinders.into_iter().all(finite_cylinder)
        || !windows
            .into_iter()
            .flatten()
            .all(|range| range.is_finite() && range.width() > 0.0)
        || windows.into_iter().any(|window| window[0].width() != TAU)
    {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    Ok(())
}

fn certify_bound_outcomes(
    inputs: &[SkewCylinderAxialBoundTopology; 4],
    formula_cylinders: [Cylinder; 2],
    formula_to_source: [usize; 2],
    source_windows: [[ParamRange; 2]; 2],
) -> Result<[PersistentSkewCylinderAxialBoundOutcome; 4], IntersectionCertificateError> {
    let expected_tags = [
        bound_tag(
            formula_to_source[0],
            PersistentSkewCylinderAxialBoundary::Lower,
        ),
        bound_tag(
            formula_to_source[0],
            PersistentSkewCylinderAxialBoundary::Upper,
        ),
        bound_tag(
            formula_to_source[1],
            PersistentSkewCylinderAxialBoundary::Lower,
        ),
        bound_tag(
            formula_to_source[1],
            PersistentSkewCylinderAxialBoundary::Upper,
        ),
    ];
    let mut normalized = [None; 4];
    for input in inputs {
        if !exact_cylinders(input.formula_cylinders(), formula_cylinders)
            || input.formula_to_source() != formula_to_source
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        let provenance = input.provenance();
        let tag = PersistentSkewCylinderAxialBoundTag::new(
            provenance.source_operand,
            persistent_boundary(provenance.boundary),
        )
        .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let slot = expected_tags
            .iter()
            .position(|expected| *expected == tag)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        if normalized[slot].is_some() {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        normalized[slot] = Some(certify_bound_outcome(input, tag, source_windows)?);
    }
    normalized
        .map(|outcome| outcome.ok_or(IntersectionCertificateError::InvalidTraceFamily))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?
        .try_into()
        .map_err(|_| IntersectionCertificateError::InvalidTraceFamily)
}

fn certify_bound_outcome(
    input: &SkewCylinderAxialBoundTopology,
    tag: PersistentSkewCylinderAxialBoundTag,
    source_windows: [[ParamRange; 2]; 2],
) -> Result<PersistentSkewCylinderAxialBoundOutcome, IntersectionCertificateError> {
    let expected_bound = match tag.boundary() {
        PersistentSkewCylinderAxialBoundary::Lower => source_windows[tag.source_slot()][1].lo,
        PersistentSkewCylinderAxialBoundary::Upper => source_windows[tag.source_slot()][1].hi,
    };
    if input.provenance().value.to_bits() != expected_bound.to_bits() {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let root_count = input.roots().len();
    let open_cell_count = input.open_cell_relations().len();
    if open_cell_count == 0
        || (root_count == 0 && open_cell_count != 1)
        || root_count > PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_BOUND
        || open_cell_count > PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let distinct_cut_count = input
        .roots()
        .iter()
        .map(|root| root.cyclic_ordinal)
        .max()
        .map_or(0, |ordinal| ordinal + 1);
    if root_count > 0 && distinct_cut_count != open_cell_count {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let mut roots = [None; PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_BOUND];
    for (slot, root) in input.roots().iter().copied().enumerate() {
        let root = persistent_root_event(root);
        validate_root_event(root, tag, expected_bound, open_cell_count)?;
        roots[slot] = Some(root);
    }
    let mut open_cell_relations =
        [None; PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_CELLS_PER_BOUND];
    for (slot, relations) in input.open_cell_relations().iter().copied().enumerate() {
        open_cell_relations[slot] = Some(relations.map(persistent_relation));
    }
    Ok(PersistentSkewCylinderAxialBoundOutcome {
        tag,
        bound: expected_bound,
        roots,
        open_cell_relations,
        root_count: root_count as u8,
        open_cell_count: open_cell_count as u8,
    })
}

fn validate_root_event(
    root: PersistentSkewCylinderAxialRootEventInput,
    tag: PersistentSkewCylinderAxialBoundTag,
    bound: f64,
    cell_count: usize,
) -> Result<(), IntersectionCertificateError> {
    let [lo, hi] = root.half_angle_bracket;
    let chart_owned = match root.half_angle_chart {
        PersistentSkewCylinderHalfAngleChart::Tangent => lo >= -1.0 && hi <= 1.0,
        PersistentSkewCylinderHalfAngleChart::Cotangent => lo > -1.0 && hi < 1.0,
    };
    if root.tag != tag
        || root.bound.to_bits() != bound.to_bits()
        || root.cyclic_ordinal >= cell_count
        || !lo.is_finite()
        || !hi.is_finite()
        || lo > hi
        || !chart_owned
        || (lo < 0.0 && hi > 0.0)
        || (!root.repeated && root.before == root.after)
        || (root.repeated && root.before != root.after)
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(())
}

/// Per-sheet ordinal tables of certified persistent root events.
type PersistentRootEventTables = [[Option<PersistentSkewCylinderFiniteWindowRootEvent>;
    PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_SHEET];
    2];

fn certify_root_events(
    finite_topology: &SkewCylinderFiniteWindowTopologyCertificate,
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
) -> Result<(PersistentRootEventTables, [u8; 2]), IntersectionCertificateError> {
    validate_root_event_partition(finite_topology)?;
    let mut result = [[None; PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_SHEET]; 2];
    let mut counts = [0_u8; 2];
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        let sheet_slot = sheet_index(sheet);
        let source = finite_topology.root_events(sheet);
        if source.len() > PERSISTENT_SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_SHEET {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        let mut previous_parameter = None;
        for (ordinal, event) in source.iter().copied().enumerate() {
            if event.sheet() != sheet
                || event.root_count() == 0
                || event.root_count() > SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER
                || !event.carrier_parameter().is_finite()
                || previous_parameter.is_some_and(|previous| previous >= event.carrier_parameter())
            {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
            let persistent = persistent_finite_window_root_event(event, outcomes)?;
            for root_index in 0..persistent.root_count() {
                let root = persistent
                    .root_reference(root_index)
                    .and_then(|reference| resolve_root_reference(outcomes, reference))
                    .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
                if root.sheet != sheet
                    || (0..root_index).any(|previous| {
                        persistent
                            .root_reference(previous)
                            .and_then(|reference| resolve_root_reference(outcomes, reference))
                            .is_some_and(|candidate| candidate.tag == root.tag)
                    })
                    || !outcomes.iter().any(|outcome| {
                        outcome.tag == root.tag
                            && outcome.roots[..outcome.root_count as usize]
                                .iter()
                                .flatten()
                                .any(|candidate| *candidate == root)
                    })
                {
                    return Err(IntersectionCertificateError::InvalidTraceFamily);
                }
            }
            result[sheet_slot][ordinal] = Some(persistent);
            previous_parameter = Some(event.carrier_parameter());
        }
        counts[sheet_slot] = source.len() as u8;
    }
    Ok((result, counts))
}

fn validate_root_event_partition(
    finite_topology: &SkewCylinderFiniteWindowTopologyCertificate,
) -> Result<(), IntersectionCertificateError> {
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        let mut endpoint_events = Vec::new();
        if let SkewCylinderFiniteSheetTopology::Open(spans) = finite_topology.sheet(sheet) {
            for span in spans {
                if span.sheet != sheet {
                    return Err(IntersectionCertificateError::InvalidTraceFamily);
                }
                endpoint_events.extend([span.start.event, span.end.event]);
            }
        }
        let boundary_events = finite_topology
            .root_events(sheet)
            .iter()
            .copied()
            .filter(|event| event.kind() == SkewCylinderFiniteWindowRootEventKind::Boundary)
            .collect::<Vec<_>>();
        if boundary_events.len() != endpoint_events.len()
            || boundary_events.iter().any(|event| {
                endpoint_events
                    .iter()
                    .filter(|candidate| *candidate == event)
                    .count()
                    != 1
            })
            || endpoint_events.iter().any(|event| {
                event.kind() != SkewCylinderFiniteWindowRootEventKind::Boundary
                    || boundary_events
                        .iter()
                        .filter(|candidate| *candidate == event)
                        .count()
                        != 1
            })
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
    }
    Ok(())
}

fn persistent_finite_window_root_event(
    event: SkewCylinderFiniteWindowRootEvent,
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
) -> Result<PersistentSkewCylinderFiniteWindowRootEvent, IntersectionCertificateError> {
    let mut roots = [EMPTY_ROOT_REFERENCE; SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER];
    for (index, slot) in roots.iter_mut().enumerate().take(event.root_count()) {
        let root = persistent_root_event(
            event
                .root(index)
                .ok_or(IntersectionCertificateError::InvalidTraceFamily)?,
        );
        *slot = find_root_reference(outcomes, root)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    }
    Ok(PersistentSkewCylinderFiniteWindowRootEvent {
        sheet: event.sheet(),
        kind: match event.kind() {
            SkewCylinderFiniteWindowRootEventKind::Boundary => {
                PersistentSkewCylinderFiniteWindowRootEventKind::Boundary
            }
            SkewCylinderFiniteWindowRootEventKind::Contact => {
                PersistentSkewCylinderFiniteWindowRootEventKind::Contact
            }
            SkewCylinderFiniteWindowRootEventKind::Isolated => {
                PersistentSkewCylinderFiniteWindowRootEventKind::Isolated
            }
        },
        roots,
        root_count: event.root_count() as u8,
        carrier_parameter: event.carrier_parameter(),
    })
}

/// Shared exact formula/source geometry validated for every family member.
#[derive(Clone, Copy)]
struct FamilyMemberGeometry {
    formula_cylinders: [Cylinder; 2],
    formula_windows: [[ParamRange; 2]; 2],
    source_cylinders: [Cylinder; 2],
}

fn certify_family_member(
    ordinal: usize,
    input: PersistentSkewCylinderFiniteWindowMemberInput,
    derived: SkewCylinderOpenSpan,
    geometry: FamilyMemberGeometry,
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
    tolerance: f64,
) -> Result<PersistentSkewCylinderFiniteWindowMemberCertificate, IntersectionCertificateError> {
    validate_member_input_identity(input, geometry, tolerance)?;
    let residual = input.residual;
    let guarded = residual.carrier_range();
    let [lower, upper] = input.root_corridors;
    let expected_roots = derived
        .root_longitude_intervals(geometry.formula_windows[0][0])
        .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    if derived.sheet != residual.sheet()
        || !exact_range(derived.range, guarded)
        || !exact_intervals(
            [lower.root_parameter(), upper.root_parameter()],
            expected_roots,
        )
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let endpoints = [
        persistent_endpoint(derived.start, outcomes)?,
        persistent_endpoint(derived.end, outcomes)?,
    ];
    validate_derived_endpoints(endpoints, outcomes, input)?;
    let (carrier_box, pcurve_boxes, residual_bounds) = member_enclosures(input)?;
    Ok(PersistentSkewCylinderFiniteWindowMemberCertificate {
        ordinal: ordinal as u8,
        sheet: residual.sheet(),
        guarded_range: guarded,
        root_parameter_enclosures: [lower.root_parameter(), upper.root_parameter()],
        endpoints,
        carrier_box,
        pcurve_boxes,
        residual_bounds,
        tolerance,
    })
}

fn validate_member_input_identity(
    input: PersistentSkewCylinderFiniteWindowMemberInput,
    geometry: FamilyMemberGeometry,
    tolerance: f64,
) -> Result<(), IntersectionCertificateError> {
    let FamilyMemberGeometry {
        formula_cylinders,
        formula_windows,
        source_cylinders,
    } = geometry;
    let residual = input.residual;
    let guarded = residual.carrier_range();
    let [lower, upper] = input.root_corridors;
    let recertified_lower = residual
        .certify_lower_pcurve_root_corridor(lower.root_parameter())
        .map_err(|_| IntersectionCertificateError::InvalidTraceFamily)?;
    let recertified_upper = residual
        .certify_upper_pcurve_root_corridor(upper.root_parameter())
        .map_err(|_| IntersectionCertificateError::InvalidTraceFamily)?;
    if !exact_cylinders(residual.carrier().cylinders(), formula_cylinders)
        || !exact_cylinders(
            residual.traces().map(|trace| trace.surface()),
            source_cylinders,
        )
        || !exact_ranges(
            residual.chart_windows(),
            formula_windows.map(|window| window[0]),
        )
        || residual.tolerance().to_bits() != tolerance.to_bits()
        || lower.guarded_end() != SkewCylinderBranchGuardedEnd::Lower
        || upper.guarded_end() != SkewCylinderBranchGuardedEnd::Upper
        || recertified_lower != lower
        || recertified_upper != upper
        || lower.root_parameter().hi() >= guarded.lo
        || upper.root_parameter().lo() <= guarded.hi
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(())
}

fn validate_derived_endpoints(
    endpoints: [PersistentSkewCylinderFiniteWindowEndpointProof; 2],
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
    input: PersistentSkewCylinderFiniteWindowMemberInput,
) -> Result<(), IntersectionCertificateError> {
    let guarded = input.residual.carrier_range();
    let expected_sides = [
        PersistentSkewCylinderRootInsideSide::After,
        PersistentSkewCylinderRootInsideSide::Before,
    ];
    for (index, endpoint) in endpoints.into_iter().enumerate() {
        let expected_parameter = if index == 0 { guarded.lo } else { guarded.hi };
        if endpoint.root_count() == 0
            || endpoint.sheet() != input.residual.sheet()
            || endpoint.inside_side() != expected_sides[index]
            || endpoint.inside_parameter().to_bits() != expected_parameter.to_bits()
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        for root_index in 0..endpoint.root_count() {
            let root = endpoint
                .root_reference(root_index)
                .and_then(|reference| resolve_root_reference(outcomes, reference))
                .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
            let required_relation = match root.tag.boundary() {
                PersistentSkewCylinderAxialBoundary::Lower => {
                    PersistentSkewCylinderAxialRelation::Above
                }
                PersistentSkewCylinderAxialBoundary::Upper => {
                    PersistentSkewCylinderAxialRelation::Below
                }
            };
            let inside_relation = match endpoint.inside_side() {
                PersistentSkewCylinderRootInsideSide::Before => root.before,
                PersistentSkewCylinderRootInsideSide::After => root.after,
            };
            let outcome_contains_root = outcomes.iter().any(|outcome| {
                outcome.tag == root.tag
                    && outcome.roots[..outcome.root_count as usize]
                        .iter()
                        .flatten()
                        .any(|candidate| *candidate == root)
            });
            if root.sheet != input.residual.sheet()
                || inside_relation != required_relation
                || !outcome_contains_root
                || !root_pcurves_contain_bound(input.root_corridors[index], root)
            {
                return Err(IntersectionCertificateError::InvalidTraceFamily);
            }
        }
    }
    Ok(())
}

fn root_pcurves_contain_bound(
    corridor: SkewCylinderBranchPcurveRootCorridorCertificate,
    root: PersistentSkewCylinderAxialRootEventInput,
) -> bool {
    let pcurve = corridor.root_pcurves()[root.tag.source_slot()];
    pcurve.stored_uv()[1].contains(root.bound) && pcurve.source_uv()[1].contains(root.bound)
}

fn endpoint_root_pcurves_contain_bounds(
    corridor: SkewCylinderBranchPcurveRootCorridorCertificate,
    endpoint: PersistentSkewCylinderFiniteWindowEndpointProof,
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
) -> bool {
    endpoint.root_count() > 0
        && (0..endpoint.root_count()).all(|index| {
            endpoint
                .root_reference(index)
                .and_then(|reference| resolve_root_reference(outcomes, reference))
                .is_some_and(|root| root_pcurves_contain_bound(corridor, root))
        })
}

fn member_enclosures(
    input: PersistentSkewCylinderFiniteWindowMemberInput,
) -> Result<(Aabb3, [Aabb2; 2], [f64; 2]), IntersectionCertificateError> {
    let residual = input.residual;
    let guarded = residual.carrier_range();
    let mut carrier_box = residual.carrier().bounding_box(guarded);
    let mut pcurve_boxes = residual
        .traces()
        .map(|trace| trace.pcurve().bounding_box(guarded));
    let mut residual_bounds = residual.residual_bounds();
    for corridor in input.root_corridors {
        carrier_box = carrier_box.union(corridor.corridor().carrier_box());
        for index in 0..2 {
            pcurve_boxes[index] = pcurve_boxes[index]
                .union(uv_box(corridor.root_pcurves()[index].stored_uv()))
                .union(uv_box(corridor.corridor().pcurves()[index].stored_uv()));
            residual_bounds[index] =
                residual_bounds[index].max(corridor.corridor().residual_bounds()[index]);
        }
    }
    if !carrier_box.is_finite()
        || pcurve_boxes.iter().any(|bounds| {
            bounds.is_empty()
                || !bounds.min.x.is_finite()
                || !bounds.min.y.is_finite()
                || !bounds.max.x.is_finite()
                || !bounds.max.y.is_finite()
        })
        || residual_bounds
            .into_iter()
            .any(|bound| !bound.is_finite() || bound < 0.0 || bound > residual.tolerance())
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok((carrier_box, pcurve_boxes, residual_bounds))
}

fn validate_member_order(
    members: &[Option<PersistentSkewCylinderFiniteWindowMemberCertificate>],
) -> Result<(), IntersectionCertificateError> {
    let members = members
        .iter()
        .copied()
        .collect::<Option<Vec<_>>>()
        .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    for pair in members.windows(2) {
        let first = pair[0];
        let second = pair[1];
        let ordered = sheet_index(first.sheet) < sheet_index(second.sheet)
            || (first.sheet == second.sheet
                && first.root_parameter_enclosures[0].hi()
                    < second.root_parameter_enclosures[0].lo());
        if !ordered || first.ordinal + 1 != second.ordinal {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
    }
    Ok(())
}

fn certify_sheet_occupancy(
    finite_topology: &SkewCylinderFiniteWindowTopologyCertificate,
    member_count: usize,
) -> Result<[PersistentSkewCylinderFiniteWindowSheetOccupancy; 2], IntersectionCertificateError> {
    let mut result = [PersistentSkewCylinderFiniteWindowSheetOccupancy::Outside; 2];
    let mut next_ordinal = 0;
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        let sheet_slot = sheet_index(sheet);
        result[sheet_slot] = match finite_topology.sheet(sheet) {
            SkewCylinderFiniteSheetTopology::Outside => {
                PersistentSkewCylinderFiniteWindowSheetOccupancy::Outside
            }
            SkewCylinderFiniteSheetTopology::Whole => {
                PersistentSkewCylinderFiniteWindowSheetOccupancy::Whole
            }
            SkewCylinderFiniteSheetTopology::Open(spans)
                if !spans.is_empty() && spans.iter().all(|span| span.sheet == sheet) =>
            {
                let occupancy = PersistentSkewCylinderFiniteWindowSheetOccupancy::Open {
                    first_member_ordinal: next_ordinal,
                    member_count: spans.len(),
                };
                next_ordinal += spans.len();
                occupancy
            }
            _ => return Err(IntersectionCertificateError::InvalidTraceFamily),
        };
    }
    if next_ordinal != member_count {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(result)
}

fn derived_open_members(
    finite_topology: &SkewCylinderFiniteWindowTopologyCertificate,
) -> Vec<SkewCylinderOpenSpan> {
    let mut members = Vec::new();
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        if let SkewCylinderFiniteSheetTopology::Open(spans) = finite_topology.sheet(sheet) {
            members.extend(spans.iter().copied());
        }
    }
    members
}

fn permute_formula_to_source<T: Copy>(values: [T; 2], permutation: [usize; 2]) -> [T; 2] {
    let mut result = values;
    for formula_slot in 0..2 {
        result[permutation[formula_slot]] = values[formula_slot];
    }
    result
}

fn bound_tag(
    source_slot: usize,
    boundary: PersistentSkewCylinderAxialBoundary,
) -> PersistentSkewCylinderAxialBoundTag {
    PersistentSkewCylinderAxialBoundTag::new(source_slot, boundary)
        .expect("built-in source slot is valid")
}

const fn persistent_boundary(
    boundary: SkewCylinderAxialBoundary,
) -> PersistentSkewCylinderAxialBoundary {
    match boundary {
        SkewCylinderAxialBoundary::Lower => PersistentSkewCylinderAxialBoundary::Lower,
        SkewCylinderAxialBoundary::Upper => PersistentSkewCylinderAxialBoundary::Upper,
    }
}

const fn persistent_relation(
    relation: SkewCylinderAxialRelation,
) -> PersistentSkewCylinderAxialRelation {
    match relation {
        SkewCylinderAxialRelation::Below => PersistentSkewCylinderAxialRelation::Below,
        SkewCylinderAxialRelation::Above => PersistentSkewCylinderAxialRelation::Above,
    }
}

const fn persistent_half_angle_chart(
    chart: SkewCylinderHalfAngleChart,
) -> PersistentSkewCylinderHalfAngleChart {
    match chart {
        SkewCylinderHalfAngleChart::Tangent => PersistentSkewCylinderHalfAngleChart::Tangent,
        SkewCylinderHalfAngleChart::Cotangent => PersistentSkewCylinderHalfAngleChart::Cotangent,
    }
}

fn persistent_root_event(root: SkewCylinderAxialRoot) -> PersistentSkewCylinderAxialRootEventInput {
    PersistentSkewCylinderAxialRootEventInput {
        tag: bound_tag(
            root.provenance.source_operand,
            persistent_boundary(root.provenance.boundary),
        ),
        bound: root.provenance.value,
        sheet: root.sheet,
        cyclic_ordinal: root.cyclic_ordinal,
        half_angle_chart: persistent_half_angle_chart(root.bracket.chart),
        half_angle_bracket: [root.bracket.lo, root.bracket.hi],
        before: persistent_relation(root.before),
        after: persistent_relation(root.after),
        repeated: root.repeated,
    }
}

fn find_root_reference(
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
    root: PersistentSkewCylinderAxialRootEventInput,
) -> Option<PersistentSkewCylinderRootReference> {
    for (outcome_index, outcome) in outcomes.iter().enumerate() {
        for root_index in 0..outcome.root_count() {
            if outcome.root(root_index) == Some(root) {
                return Some(PersistentSkewCylinderRootReference {
                    outcome: outcome_index as u8,
                    root: root_index as u8,
                });
            }
        }
    }
    None
}

fn resolve_root_reference(
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
    reference: PersistentSkewCylinderRootReference,
) -> Option<PersistentSkewCylinderAxialRootEventInput> {
    outcomes
        .get(reference.outcome as usize)?
        .root(reference.root as usize)
}

fn persistent_endpoint(
    endpoint: SkewCylinderOpenSpanEndpointProof,
    outcomes: &[PersistentSkewCylinderAxialBoundOutcome; 4],
) -> Result<PersistentSkewCylinderFiniteWindowEndpointProof, IntersectionCertificateError> {
    let mut roots = [EMPTY_ROOT_REFERENCE; SKEW_CYLINDER_FINITE_WINDOW_MAX_ROOT_EVENTS_PER_CLUSTER];
    for (index, slot) in roots
        .iter_mut()
        .enumerate()
        .take(endpoint.event.root_count())
    {
        let root = persistent_root_event(
            endpoint
                .event
                .root(index)
                .ok_or(IntersectionCertificateError::InvalidTraceFamily)?,
        );
        *slot = find_root_reference(outcomes, root)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    }
    Ok(PersistentSkewCylinderFiniteWindowEndpointProof::new(
        roots,
        endpoint.event.root_count(),
        endpoint.event.sheet(),
        match endpoint.inside_side {
            SkewCylinderRootInsideSide::Before => PersistentSkewCylinderRootInsideSide::Before,
            SkewCylinderRootInsideSide::After => PersistentSkewCylinderRootInsideSide::After,
        },
        endpoint.carrier_parameter,
    ))
}

const fn sheet_index(sheet: SkewCylinderSheet) -> usize {
    match sheet {
        SkewCylinderSheet::Lower => 0,
        SkewCylinderSheet::Upper => 1,
    }
}

fn finite_cylinder(cylinder: Cylinder) -> bool {
    let frame = cylinder.frame();
    [frame.origin(), frame.x(), frame.y(), frame.z()]
        .into_iter()
        .all(|value| value.x.is_finite() && value.y.is_finite() && value.z.is_finite())
        && cylinder.radius().is_finite()
        && cylinder.radius() > 0.0
}

fn exact_cylinders(lhs: [Cylinder; 2], rhs: [Cylinder; 2]) -> bool {
    lhs.into_iter()
        .zip(rhs)
        .all(|(lhs, rhs)| exact_cylinder(lhs, rhs))
}

fn exact_cylinder(lhs: Cylinder, rhs: Cylinder) -> bool {
    let lhs_frame = lhs.frame();
    let rhs_frame = rhs.frame();
    [
        lhs_frame.origin(),
        lhs_frame.x(),
        lhs_frame.y(),
        lhs_frame.z(),
    ]
    .into_iter()
    .zip([
        rhs_frame.origin(),
        rhs_frame.x(),
        rhs_frame.y(),
        rhs_frame.z(),
    ])
    .all(|(lhs, rhs)| exact_vec3(lhs, rhs))
        && lhs.radius().to_bits() == rhs.radius().to_bits()
}

fn exact_vec3(lhs: Vec3, rhs: Vec3) -> bool {
    lhs.x.to_bits() == rhs.x.to_bits()
        && lhs.y.to_bits() == rhs.y.to_bits()
        && lhs.z.to_bits() == rhs.z.to_bits()
}

fn exact_range(lhs: ParamRange, rhs: ParamRange) -> bool {
    lhs.lo.to_bits() == rhs.lo.to_bits() && lhs.hi.to_bits() == rhs.hi.to_bits()
}

fn exact_ranges(lhs: [ParamRange; 2], rhs: [ParamRange; 2]) -> bool {
    lhs.into_iter()
        .zip(rhs)
        .all(|(lhs, rhs)| exact_range(lhs, rhs))
}

fn exact_interval(lhs: Interval, rhs: Interval) -> bool {
    lhs.lo().to_bits() == rhs.lo().to_bits() && lhs.hi().to_bits() == rhs.hi().to_bits()
}

fn exact_intervals(lhs: [Interval; 2], rhs: [Interval; 2]) -> bool {
    lhs.into_iter()
        .zip(rhs)
        .all(|(lhs, rhs)| exact_interval(lhs, rhs))
}

fn exact_f64s(lhs: [f64; 2], rhs: [f64; 2]) -> bool {
    lhs.into_iter()
        .zip(rhs)
        .all(|(lhs, rhs)| lhs.to_bits() == rhs.to_bits())
}

fn exact_aabb2s(lhs: [Aabb2; 2], rhs: [Aabb2; 2]) -> bool {
    lhs.into_iter()
        .zip(rhs)
        .all(|(lhs, rhs)| exact_aabb2(lhs, rhs))
}

fn exact_aabb2(lhs: Aabb2, rhs: Aabb2) -> bool {
    lhs.min.x.to_bits() == rhs.min.x.to_bits()
        && lhs.min.y.to_bits() == rhs.min.y.to_bits()
        && lhs.max.x.to_bits() == rhs.max.x.to_bits()
        && lhs.max.y.to_bits() == rhs.max.y.to_bits()
}

fn exact_aabb3(lhs: Aabb3, rhs: Aabb3) -> bool {
    exact_vec3(lhs.min, rhs.min) && exact_vec3(lhs.max, rhs.max)
}

fn uv_box(uv: [Interval; 2]) -> Aabb2 {
    Aabb2 {
        min: Vec2::new(uv[0].lo(), uv[1].lo()),
        max: Vec2::new(uv[0].hi(), uv[1].hi()),
    }
}

#[cfg(test)]
#[path = "skew_cylinder_branch_persistent_family_tests.rs"]
mod tests;
