//! Persistent exact-root carriers for bounded skew-cylinder support contact.
//!
//! The cyclic second-harmonic theorem seals isolated support points, folded
//! support curves, and one- or two-repeated-root touching curves before this
//! module binds them to finite full-period cylinder windows. Outward interval
//! evaluation owns support incidence and strict-positive member corridors;
//! rounded analytic evaluation supplies only deterministic representatives.

use kcore::interval::Interval;
use kgeom::curve::Curve;
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::Vec3;

use super::{
    BranchAlgebra, SkewCylinderDiscriminantContactTopologyCertificate,
    SkewCylinderDiscriminantRoot, SkewCylinderHalfAngleChart, SkewCylinderSheet, build_algebra,
    coefficient_proof, cotangent_projective_interval, finite_interval, longitude_interval,
    tangent_projective_interval,
};
use super::{
    PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK, PairedSkewCylinderBranchResidualCertificate,
    SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK,
    SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK, SkewCylinderAxialBoundProvenance,
    SkewCylinderAxialBoundary, SkewCylinderFoldedSupportCellLocation,
    SkewCylinderFoldedSupportTopologyCertificate, SkewCylinderTouchingSupportTopologyCertificate,
};
use crate::IntersectionCertificateError;

const TAU: f64 = core::f64::consts::TAU;
const UNSUPPORTED_REASON: &str = "skew Cylinder/Cylinder support contact is not a certified one- or two-repeated-root layout inside or exactly on the boundary of two full-period finite windows";

type FormulaFoldedEndpointParameters = [[[f64; 2]; 2]; 2];
type FormulaFoldedSeamEvidence = (FormulaFoldedEndpointParameters, [Vec3; 2]);

/// Exact logical work for both guarded sheet members of one folded support
/// curve. The endpoint topology was already paid by the discriminant query;
/// each sheet reuses the established open-span allowance.
pub const SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK: u64 =
    2 * PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK;

/// Exact logical work for the four guarded members needed when the positive
/// folded cell is split by the canonical/authored longitude seam.
pub const SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK: u64 =
    4 * PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK;

/// Exact guarded-member work selected by one sealed folded-support layout.
pub fn persistent_skew_cylinder_folded_support_exact_work(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
) -> u64 {
    if (topology.positive_cell() == SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        && !is_short_across_seam_root_layout(topology))
        || is_seam_root_folded_chart_layout(topology)
    {
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
    } else {
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    }
}

/// Exact logical work for the six guarded members of one repeated-root
/// touching-support family. Each member owns two 256-cell exact Bernstein
/// allowances for its source and stored-evaluator positive-radicand margins in
/// addition to the established guarded open-span proof.
pub const SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK: u64 = 6
    * (PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
        + SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK);

/// Exact logical work for the eight guarded members required when one
/// repeated root lies at the opposite half-angle pole. Both sides of that
/// root use the finite cotangent chart and meet the tangent chart at π/2 and
/// 3π/2 respectively.
pub const SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK: u64 = 8
    * (PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
        + SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK);

/// Exact guarded-member work selected by one sealed touching-support layout.
pub fn persistent_skew_cylinder_touching_support_exact_work(
    topology: &SkewCylinderTouchingSupportTopologyCertificate,
) -> u64 {
    let [root] = topology.roots() else {
        return SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK;
    };
    let angular = root.angular_bracket();
    if angular.lo.to_bits() == core::f64::consts::PI.to_bits()
        && angular.hi.to_bits() == core::f64::consts::PI.to_bits()
    {
        SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK
    } else {
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK
    }
}

/// Exact topology owning one end of a guarded folded-support member.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderFoldedSupportEndpoint {
    /// One of the two exact simple discriminant roots.
    Root(usize),
    /// The exact authored periodic seam on one ordered sheet.
    Seam(SkewCylinderSheet),
    /// One exact regular tangent/cotangent transition on an ordered sheet.
    ChartJoin(SkewCylinderSheet),
}

/// Proof-owned ordinal of a regular tangent/cotangent transition in one
/// touching-support partition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderTouchingSupportChartJoin {
    /// First transition in increasing authored longitude.
    First,
    /// Second transition in increasing authored longitude.
    Second,
}

/// Proof-owned ordinal in a one-or-two-root touching-support cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderTouchingSupportRoot {
    /// First repeated root in canonical cyclic order.
    First,
    /// Second repeated root in canonical cyclic order.
    Second,
}

impl PersistentSkewCylinderTouchingSupportRoot {
    /// Zero-based canonical root ordinal.
    pub const fn ordinal(self) -> u8 {
        match self {
            Self::First => 0,
            Self::Second => 1,
        }
    }

    const fn index(self) -> usize {
        self.ordinal() as usize
    }
}

impl PersistentSkewCylinderTouchingSupportChartJoin {
    const fn ordinal(self) -> usize {
        match self {
            Self::First => 0,
            Self::Second => 1,
        }
    }
}

/// Exact topology owning one end of a guarded touching-support member.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderTouchingSupportEndpoint {
    /// The shared repeated support root plus its smooth-continuation port.
    Root {
        /// Identity in the exact canonical repeated-root cycle.
        root: PersistentSkewCylinderTouchingSupportRoot,
        /// Exact smooth-continuation port pairing opposite sheets.
        continuation: u8,
    },
    /// The exact authored periodic seam on one ordered sheet.
    Seam(SkewCylinderSheet),
    /// One exact tangent/cotangent transition longitude on an ordered sheet.
    ChartJoin {
        /// Ordered touching-support sheet owning the join.
        sheet: SkewCylinderSheet,
        /// Identity in the certificate's ordered regular chart joins.
        join: PersistentSkewCylinderTouchingSupportChartJoin,
    },
}

/// Persistent exact-topology-owned carrier proof for one folded support curve
/// wholly inside two finite cylinder windows.
#[derive(Debug, Clone, PartialEq)]
pub struct PersistentSkewCylinderFoldedSupportCertificate {
    topology: SkewCylinderFoldedSupportTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    formula_root_longitudes: [Interval; 2],
    guarded_ranges: Vec<ParamRange>,
    formula_residuals: Vec<PairedSkewCylinderBranchResidualCertificate>,
    formula_branch_endpoints: Vec<[PersistentSkewCylinderFoldedSupportEndpoint; 2]>,
    formula_endpoint_parameters: [[[f64; 2]; 2]; 2],
    formula_endpoint_points: [Vec3; 2],
    formula_seam_parameters: Option<[[[f64; 2]; 2]; 2]>,
    formula_seam_points: Option<[Vec3; 2]>,
    formula_chart_join_parameters: Option<FormulaFoldedEndpointParameters>,
    formula_chart_join_points: Option<[Vec3; 2]>,
    chart_join_longitude: Option<f64>,
    required_edge_tolerance: f64,
    tolerance: f64,
}

/// Persistent exact-topology-owned carrier proof for two regular support
/// sheets meeting at one repeated discriminant root.
#[derive(Debug, Clone, PartialEq)]
pub struct PersistentSkewCylinderTouchingSupportCertificate {
    topology: SkewCylinderTouchingSupportTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    formula_root_longitudes: Vec<Interval>,
    chart_join_longitudes: Vec<f64>,
    guarded_ranges: Vec<ParamRange>,
    formula_residuals: Vec<PairedSkewCylinderBranchResidualCertificate>,
    formula_branch_endpoints: Vec<[PersistentSkewCylinderTouchingSupportEndpoint; 2]>,
    formula_root_parameters: [[[f64; 2]; 2]; 2],
    formula_root_points: [Vec3; 2],
    formula_seam_parameters: [[[f64; 2]; 2]; 2],
    formula_seam_points: [Vec3; 2],
    formula_chart_join_parameters: [FormulaFoldedEndpointParameters; 2],
    formula_chart_join_points: [[Vec3; 2]; 2],
    required_edge_tolerance: f64,
    tolerance: f64,
}

impl PersistentSkewCylinderTouchingSupportCertificate {
    /// Complete exact repeated-root/strict-positive-cell topology.
    pub const fn topology(&self) -> &SkewCylinderTouchingSupportTopologyCertificate {
        &self.topology
    }

    /// Exact-source repeated-root longitude enclosure.
    pub fn formula_root_longitude(&self) -> Interval {
        self.formula_root_longitudes[0]
    }

    /// Complete exact-source repeated-root longitude enclosures.
    pub fn formula_root_longitudes(&self) -> &[Interval] {
        &self.formula_root_longitudes
    }

    /// Exact regular longitudes where tangent/cotangent guarded charts meet.
    pub fn chart_join_longitudes(&self) -> &[f64] {
        &self.chart_join_longitudes
    }

    /// First exact regular tangent/cotangent transition longitude.
    ///
    /// Retained for source compatibility with the original one-join touching
    /// layout; use [`Self::chart_join_longitude_for`] when binding endpoint
    /// identity.
    pub fn chart_join_longitude(&self) -> f64 {
        self.chart_join_longitudes[0]
    }

    /// Exact regular longitude for one proof-owned chart-join ordinal.
    pub fn chart_join_longitude_for(
        &self,
        join: PersistentSkewCylinderTouchingSupportChartJoin,
    ) -> Option<f64> {
        self.chart_join_longitudes.get(join.ordinal()).copied()
    }

    /// Strict-positive guarded carrier ranges in branch publication order.
    pub fn guarded_ranges(&self) -> &[ParamRange] {
        &self.guarded_ranges
    }

    /// Guarded paired residual certificates in branch publication order.
    pub fn formula_residuals(&self) -> &[PairedSkewCylinderBranchResidualCertificate] {
        &self.formula_residuals
    }

    /// Exact endpoint identities aligned with the guarded residual members.
    pub fn formula_branch_endpoints(
        &self,
    ) -> &[[PersistentSkewCylinderTouchingSupportEndpoint; 2]] {
        &self.formula_branch_endpoints
    }

    /// Caller/source-order finite windows.
    pub fn source_windows(&self) -> [[ParamRange; 2]; 2] {
        permute_formula_to_source(self.formula_windows, self.formula_to_source)
    }

    /// Deterministic endpoint representative for one guarded branch end.
    pub fn endpoint_point(&self, endpoint: PersistentSkewCylinderTouchingSupportEndpoint) -> Vec3 {
        match endpoint {
            PersistentSkewCylinderTouchingSupportEndpoint::Root { root, .. } => {
                self.formula_root_points[root.index()]
            }
            PersistentSkewCylinderTouchingSupportEndpoint::Seam(sheet) => {
                self.formula_seam_points[sheet_ordinal(sheet)]
            }
            PersistentSkewCylinderTouchingSupportEndpoint::ChartJoin { sheet, join } => {
                self.formula_chart_join_points[join.ordinal()][sheet_ordinal(sheet)]
            }
        }
    }

    /// Caller-order source parameters for one guarded branch end.
    pub fn source_parameters(
        &self,
        endpoint: PersistentSkewCylinderTouchingSupportEndpoint,
    ) -> [[f64; 2]; 2] {
        let formula = match endpoint {
            PersistentSkewCylinderTouchingSupportEndpoint::Root { root, .. } => {
                self.formula_root_parameters[root.index()]
            }
            PersistentSkewCylinderTouchingSupportEndpoint::Seam(sheet) => {
                self.formula_seam_parameters[sheet_ordinal(sheet)]
            }
            PersistentSkewCylinderTouchingSupportEndpoint::ChartJoin { sheet, join } => {
                self.formula_chart_join_parameters[join.ordinal()][sheet_ordinal(sheet)]
            }
        };
        permute_formula_to_source(formula, self.formula_to_source)
    }

    /// Complete metric envelope joining guarded evaluators to their exact
    /// repeated-root, authored-seam, or chart-join endpoint points.
    pub const fn required_edge_tolerance(&self) -> f64 {
        self.required_edge_tolerance
    }

    /// Requested residual tolerance.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Fixed logical work represented by all guarded branches.
    pub fn work(&self) -> u64 {
        self.formula_residuals.len() as u64
            * (PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
                + SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK)
    }
}

impl PersistentSkewCylinderFoldedSupportCertificate {
    /// Complete exact two-root/one-positive-cell source topology.
    pub const fn topology(&self) -> &SkewCylinderFoldedSupportTopologyCertificate {
        &self.topology
    }

    /// Exact-source root longitude enclosures in increasing formula order.
    pub const fn formula_root_longitudes(&self) -> [Interval; 2] {
        self.formula_root_longitudes
    }

    /// Strict-positive guarded carrier ranges in branch publication order.
    pub fn guarded_ranges(&self) -> &[ParamRange] {
        &self.guarded_ranges
    }

    /// Guarded paired residual certificates in branch publication order.
    pub fn formula_residuals(&self) -> &[PairedSkewCylinderBranchResidualCertificate] {
        &self.formula_residuals
    }

    /// Exact endpoint identities aligned with the guarded residual members.
    pub fn formula_branch_endpoints(&self) -> &[[PersistentSkewCylinderFoldedSupportEndpoint; 2]] {
        &self.formula_branch_endpoints
    }

    /// Caller/source-order finite windows.
    pub fn source_windows(&self) -> [[ParamRange; 2]; 2] {
        permute_formula_to_source(self.formula_windows, self.formula_to_source)
    }

    /// Caller/source-order parameters at the two exact support joins.
    pub fn source_endpoint_parameters(&self) -> [[[f64; 2]; 2]; 2] {
        self.formula_endpoint_parameters
            .map(|parameters| permute_formula_to_source(parameters, self.formula_to_source))
    }

    /// Deterministic model-space representatives of the two exact joins.
    pub const fn endpoint_points(&self) -> [Vec3; 2] {
        self.formula_endpoint_points
    }

    /// Caller-order parameters at the sheet-specific authored seam joins.
    pub fn source_seam_parameters(&self) -> Option<[[[f64; 2]; 2]; 2]> {
        self.formula_seam_parameters.map(|parameters| {
            parameters
                .map(|parameters| permute_formula_to_source(parameters, self.formula_to_source))
        })
    }

    /// Deterministic model-space representatives of the sheet-specific seam joins.
    pub const fn seam_points(&self) -> Option<[Vec3; 2]> {
        self.formula_seam_points
    }

    /// Exact regular longitude where tangent/cotangent guarded charts meet.
    pub const fn chart_join_longitude(&self) -> Option<f64> {
        self.chart_join_longitude
    }

    /// Deterministic endpoint representative for one guarded branch end.
    pub fn endpoint_point(&self, endpoint: PersistentSkewCylinderFoldedSupportEndpoint) -> Vec3 {
        match endpoint {
            PersistentSkewCylinderFoldedSupportEndpoint::Root(ordinal) => {
                self.formula_endpoint_points[ordinal]
            }
            PersistentSkewCylinderFoldedSupportEndpoint::Seam(sheet) => self
                .formula_seam_points
                .expect("sealed seam endpoint retains its point")[sheet_ordinal(sheet)],
            PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(sheet) => self
                .formula_chart_join_points
                .expect("sealed chart-join endpoint retains its point")[sheet_ordinal(sheet)],
        }
    }

    /// Caller-order source parameters for one guarded branch end.
    pub fn source_parameters(
        &self,
        endpoint: PersistentSkewCylinderFoldedSupportEndpoint,
    ) -> [[f64; 2]; 2] {
        match endpoint {
            PersistentSkewCylinderFoldedSupportEndpoint::Root(ordinal) => {
                self.source_endpoint_parameters()[ordinal]
            }
            PersistentSkewCylinderFoldedSupportEndpoint::Seam(sheet) => self
                .source_seam_parameters()
                .expect("sealed seam endpoint retains its source parameters")[sheet_ordinal(sheet)],
            PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(sheet) => {
                permute_formula_to_source(
                    self.formula_chart_join_parameters
                        .expect("sealed chart-join endpoint retains its source parameters")
                        [sheet_ordinal(sheet)],
                    self.formula_to_source,
                )
            }
        }
    }

    /// Complete metric envelope joining guarded sheet evaluators to their
    /// exact support-root or authored-seam endpoint points.
    pub const fn required_edge_tolerance(&self) -> f64 {
        self.required_edge_tolerance
    }

    /// Requested residual tolerance.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Fixed logical work represented by all guarded branches.
    pub fn work(&self) -> u64 {
        self.formula_residuals.len() as u64 * PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
    }
}

const fn sheet_ordinal(sheet: SkewCylinderSheet) -> usize {
    match sheet {
        SkewCylinderSheet::Lower => 0,
        SkewCylinderSheet::Upper => 1,
    }
}

/// Exact axial location of one source coordinate at an isolated support
/// contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PersistentSkewCylinderSupportContactAxialLocation {
    /// Strictly inside the authored axial interval.
    Interior,
    /// Exactly on the authored lower axial bound.
    Lower,
    /// Exactly on the authored upper axial bound.
    Upper,
}

/// Exact root-relation work needed to distinguish support incidence from an
/// interval-only near-boundary overlap.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentSkewCylinderSupportContactBoundaryPlan {
    bits: u8,
}

impl PersistentSkewCylinderSupportContactBoundaryPlan {
    /// One bit per formula-slot lower/upper bound.
    pub const fn bits(self) -> u8 {
        self.bits
    }

    /// Number of exact discriminant/axial-root identity queries.
    pub const fn query_count(self) -> usize {
        self.bits.count_ones() as usize
    }

    /// Existing root-cluster logical work reused by this plan.
    pub const fn work(self) -> u64 {
        self.query_count() as u64 * SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
    }
}

/// Persistent exact-root-owned point carrier for one isolated
/// infinite-support skew-cylinder tangency inside or on the exact authored
/// boundary of two finite source windows.
#[derive(Debug, Clone, PartialEq)]
pub struct PersistentSkewCylinderSupportContactCertificate {
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    formula_axial_locations: [PersistentSkewCylinderSupportContactAxialLocation; 2],
    formula_longitude_enclosures: [Interval; 2],
    boundary_plan: PersistentSkewCylinderSupportContactBoundaryPlan,
    carrier_parameter: f64,
    tolerance: f64,
}

impl PersistentSkewCylinderSupportContactCertificate {
    /// Complete exact discriminant root-and-cell proof.
    pub const fn topology(&self) -> &SkewCylinderDiscriminantContactTopologyCertificate {
        &self.topology
    }

    /// Exact projective root owning this zero-dimensional carrier.
    pub fn root(&self) -> SkewCylinderDiscriminantRoot {
        self.topology
            .isolated_support_root()
            .expect("sealed support-contact certificate retains one isolated root")
    }

    /// Formula-slot to caller/source-slot permutation.
    pub const fn formula_to_source(&self) -> [usize; 2] {
        self.formula_to_source
    }

    /// Source cylinders in caller/live-dependency order.
    pub fn source_cylinders(&self) -> [Cylinder; 2] {
        permute_formula_to_source(self.topology.formula_cylinders(), self.formula_to_source)
    }

    /// Certified finite source windows in caller/live-dependency order.
    pub fn source_windows(&self) -> [[ParamRange; 2]; 2] {
        permute_formula_to_source(self.formula_windows, self.formula_to_source)
    }

    /// Exact finite-window location on each caller/source cylinder.
    pub fn source_axial_locations(&self) -> [PersistentSkewCylinderSupportContactAxialLocation; 2] {
        permute_formula_to_source(self.formula_axial_locations, self.formula_to_source)
    }

    /// Exact authored boundary, when one owns this support point.
    pub fn source_axial_boundaries(&self) -> [Option<SkewCylinderAxialBoundary>; 2] {
        self.source_axial_locations()
            .map(|location| match location {
                PersistentSkewCylinderSupportContactAxialLocation::Interior => None,
                PersistentSkewCylinderSupportContactAxialLocation::Lower => {
                    Some(SkewCylinderAxialBoundary::Lower)
                }
                PersistentSkewCylinderSupportContactAxialLocation::Upper => {
                    Some(SkewCylinderAxialBoundary::Upper)
                }
            })
    }

    /// Exact-source longitude enclosure on each caller/source cylinder.
    pub fn source_longitude_enclosures(&self) -> [Interval; 2] {
        permute_formula_to_source(self.formula_longitude_enclosures, self.formula_to_source)
    }

    /// Exact relation work used to prove boundary ownership.
    pub const fn boundary_plan(&self) -> PersistentSkewCylinderSupportContactBoundaryPlan {
        self.boundary_plan
    }

    /// Deterministic authored-chart representative of the exact root.
    pub const fn carrier_parameter(&self) -> f64 {
        self.carrier_parameter
    }

    /// Operation tolerance used only for representative residual validation.
    pub const fn tolerance(&self) -> f64 {
        self.tolerance
    }

    /// Deterministic model-space representative of the exact projective
    /// contact carrier.
    pub fn point(&self) -> Vec3 {
        self.algebra()
            .authored_carrier_derivs(self.carrier_parameter, 0)
            .d[0]
    }

    /// Parameters on both source cylinders in caller/live-dependency order.
    pub fn source_surface_parameters(&self) -> [[f64; 2]; 2] {
        let algebra = self.algebra();
        let formula = [0, 1].map(|operand| {
            let uv = algebra
                .authored_pcurve_derivs(operand, self.carrier_parameter, 0)
                .d[0];
            [uv.x, uv.y]
        });
        permute_formula_to_source(formula, self.formula_to_source)
    }

    /// Independent source-surface evaluations at the retained parameters.
    pub fn source_surface_points(&self) -> [Vec3; 2] {
        let cylinders = self.source_cylinders();
        let parameters = self.source_surface_parameters();
        [
            cylinders[0].eval(parameters[0]),
            cylinders[1].eval(parameters[1]),
        ]
    }

    fn algebra(&self) -> BranchAlgebra {
        let mut algebra = build_algebra(
            self.topology.formula_cylinders(),
            self.formula_windows[0][0],
            SkewCylinderSheet::Lower,
        )
        .expect("sealed support-contact certificate rebuilds finite algebra");
        let raw_longitude = algebra
            .authored_pcurve_derivs(1, self.carrier_parameter, 0)
            .d[0]
            .x;
        let lifted = fit_full_period_parameter(raw_longitude, self.formula_windows[1][0])
            .expect("sealed support contact retains one opposite-chart lift");
        algebra.longitude_offset = lifted - raw_longitude;
        algebra
    }
}

fn is_seam_root_folded_chart_layout(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
) -> bool {
    if !has_canonical_seam_root(topology) {
        return false;
    }
    let angular = topology
        .roots()
        .map(SkewCylinderDiscriminantRoot::angular_bracket);
    angular[1].lo.to_bits() == core::f64::consts::PI.to_bits()
        && angular[1].hi.to_bits() == core::f64::consts::PI.to_bits()
}

fn has_canonical_seam_root(topology: &SkewCylinderFoldedSupportTopologyCertificate) -> bool {
    let first = topology.roots()[0].angular_bracket();
    first.lo.to_bits() == 0.0_f64.to_bits() && first.hi.to_bits() == 0.0_f64.to_bits()
}

fn is_short_between_seam_root_layout(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
) -> bool {
    let second = topology.roots()[1].angular_bracket();
    topology.positive_cell() == SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
        && has_canonical_seam_root(topology)
        && second.hi < core::f64::consts::PI
}

fn is_short_across_seam_root_layout(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
) -> bool {
    let second = topology.roots()[1].angular_bracket();
    topology.positive_cell() == SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        && has_canonical_seam_root(topology)
        && second.lo > core::f64::consts::PI
        && second.hi < TAU
}

/// Certify one folded support curve whose two simple roots bound a
/// strict-positive cell wholly inside both finite source windows.
pub fn certify_persistent_skew_cylinder_folded_support(
    topology: SkewCylinderFoldedSupportTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
    work_limit: u64,
) -> Result<PersistentSkewCylinderFoldedSupportCertificate, IntersectionCertificateError> {
    validate_inputs(formula_windows, formula_to_source, tolerance)?;
    let roots = topology.roots();
    let canonical_seam_root = has_canonical_seam_root(&topology);
    let seam_root_chart_layout = is_seam_root_folded_chart_layout(&topology);
    let short_between_seam_root_layout = is_short_between_seam_root_layout(&topology);
    let short_across_seam_root_layout = is_short_across_seam_root_layout(&topology);
    let required_work = persistent_skew_cylinder_folded_support_exact_work(&topology);
    if work_limit < required_work {
        return Err(unsupported());
    }
    let evidence = roots.map(|root| {
        let angular = root.angular_bracket();
        let on_seam = canonical_seam_root
            && angular.lo.to_bits() == 0.0_f64.to_bits()
            && angular.hi.to_bits() == 0.0_f64.to_bits();
        root_evidence(topology.topology(), root, formula_windows, on_seam, true)
    });
    let [first, second] = evidence;
    let [first, second] = [first?, second?];
    let formula_root_longitudes = [
        first.formula_longitude_enclosures[0],
        second.formula_longitude_enclosures[0],
    ];
    if formula_root_longitudes[0].hi() >= formula_root_longitudes[1].lo() {
        return Err(unsupported());
    }
    for endpoint in [first, second] {
        if endpoint
            .exact_heights
            .into_iter()
            .zip(formula_windows)
            .any(|(height, window)| !strictly_inside(height, window[1]))
        {
            return Err(unsupported());
        }
    }
    use PersistentSkewCylinderFoldedSupportEndpoint::{ChartJoin, Root, Seam};
    let branch_specs = match topology.positive_cell() {
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots if seam_root_chart_layout => {
            let authored = formula_windows[0][0];
            if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits()
            {
                return Err(unsupported());
            }
            // A one-ULP angular inset at zero underflows the rounded sheet
            // separation even though the exact cell is positive. One
            // sixteenth of machine epsilon in projective space stays within
            // the public endpoint envelope; the exact-zero factor theorem
            // retains a quantitative evaluator radicand without subdivision.
            let root_inset = f64::EPSILON / 16.0;
            let tangent_guard = crate::exact::bounded_polynomial::RootBracket {
                lo: root_inset,
                hi: 1.0_f64.next_down(),
            };
            let cotangent_guard = tangent_guard;
            let tangent_range = ParamRange::new(
                (2.0 * kcore::math::atan2(root_inset, 1.0)).next_up(),
                core::f64::consts::FRAC_PI_2.next_down(),
            );
            let cotangent_range = ParamRange::new(
                core::f64::consts::FRAC_PI_2.next_up(),
                (2.0 * kcore::math::atan2(1.0, root_inset)).next_down(),
            );
            if !strict_guarded_range(tangent_range, authored)
                || !strict_guarded_range(cotangent_range, authored)
            {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    tangent_guard,
                    tangent_range,
                    [Root(0), ChartJoin(SkewCylinderSheet::Lower)],
                ),
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Cotangent,
                    cotangent_guard,
                    cotangent_range,
                    [ChartJoin(SkewCylinderSheet::Lower), Root(1)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    tangent_guard,
                    tangent_range,
                    [Root(0), ChartJoin(SkewCylinderSheet::Upper)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Cotangent,
                    cotangent_guard,
                    cotangent_range,
                    [ChartJoin(SkewCylinderSheet::Upper), Root(1)],
                ),
            ]
        }
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            if short_between_seam_root_layout =>
        {
            let authored = formula_windows[0][0];
            if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits()
            {
                return Err(unsupported());
            }
            let Some(second) = tangent_projective_interval(roots[1].bracket()) else {
                return Err(unsupported());
            };
            // The exact root at zero needs a quantitative source and stored
            // radicand margin. This radius-scaled half-epsilon inset stays
            // inside its public root envelope while avoiding a subnormal
            // guard; the companion root remains one ULP outside the guarded
            // member.
            let root_inset = f64::EPSILON / (2.0 * topology.formula_cylinders()[0].radius());
            let guard = crate::exact::bounded_polynomial::RootBracket {
                lo: root_inset,
                hi: second.lo().next_down(),
            };
            let range = ParamRange::new(
                (2.0 * kcore::math::atan2(guard.lo, 1.0)).next_up(),
                (2.0 * kcore::math::atan2(guard.hi, 1.0)).next_down(),
            );
            if guard.lo >= guard.hi || !strict_guarded_range(range, authored) {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    range,
                    [Root(0), Root(1)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    range,
                    [Root(0), Root(1)],
                ),
            ]
        }
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => {
            let tangent_roots = roots.map(|root| tangent_projective_interval(root.bracket()));
            let (chart, guard, range) = match tangent_roots {
                [Some(first), Some(second)] if first.hi() < second.lo() => {
                    let guard = crate::exact::bounded_polynomial::RootBracket {
                        lo: first.hi().next_up(),
                        hi: second.lo().next_down(),
                    };
                    let angles = [guard.lo, guard.hi]
                        .map(|parameter| 2.0 * kcore::math::atan2(parameter, 1.0));
                    (
                        SkewCylinderHalfAngleChart::Tangent,
                        guard,
                        ParamRange::new(angles[0].next_up(), angles[1].next_down()),
                    )
                }
                _ => {
                    let [Some(first), Some(second)] =
                        roots.map(|root| cotangent_projective_interval(root.bracket()))
                    else {
                        return Err(unsupported());
                    };
                    let guard = inward_projective_guard(second.hi(), first.lo(), 4096.0)
                        .ok_or_else(unsupported)?;
                    let cot_angle = |parameter| 2.0 * kcore::math::atan2(1.0, parameter);
                    (
                        SkewCylinderHalfAngleChart::Cotangent,
                        guard,
                        ParamRange::new(
                            cot_angle(guard.hi).next_up(),
                            cot_angle(guard.lo).next_down(),
                        ),
                    )
                }
            };
            if !strict_guarded_range(range, formula_windows[0][0]) {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    chart,
                    guard,
                    range,
                    [Root(0), Root(1)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    chart,
                    guard,
                    range,
                    [Root(0), Root(1)],
                ),
            ]
        }
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam if seam_root_chart_layout => {
            let authored = formula_windows[0][0];
            if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits()
            {
                return Err(unsupported());
            }
            // This is the exact mirror of the seam-root layout above.  The
            // positive cell starts at the cotangent pole `u = pi`, crosses
            // the regular chart join at `3*pi/2`, and terminates at the
            // authored seam/root.  Keep the same exact-zero-factor inset so
            // the rounded sheet separation stays quantitative at both ends.
            let root_inset = f64::EPSILON / 16.0;
            let projective_guard = crate::exact::bounded_polynomial::RootBracket {
                lo: (-1.0_f64).next_up(),
                hi: -root_inset,
            };
            let cot_angle = |parameter| 2.0 * kcore::math::atan2(1.0, parameter);
            let cotangent_range = ParamRange::new(
                cot_angle(projective_guard.hi).next_up(),
                cot_angle(projective_guard.lo).next_down(),
            );
            let tangent_range = ParamRange::new(
                (2.0 * kcore::math::atan2(projective_guard.lo, 1.0) + TAU).next_up(),
                (2.0 * kcore::math::atan2(projective_guard.hi, 1.0) + TAU).next_down(),
            );
            if !strict_guarded_range(cotangent_range, authored)
                || !strict_guarded_range(tangent_range, authored)
            {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Cotangent,
                    projective_guard,
                    cotangent_range,
                    [Root(1), ChartJoin(SkewCylinderSheet::Lower)],
                ),
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    projective_guard,
                    tangent_range,
                    [ChartJoin(SkewCylinderSheet::Lower), Root(0)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Cotangent,
                    projective_guard,
                    cotangent_range,
                    [Root(1), ChartJoin(SkewCylinderSheet::Upper)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    projective_guard,
                    tangent_range,
                    [ChartJoin(SkewCylinderSheet::Upper), Root(0)],
                ),
            ]
        }
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            if short_across_seam_root_layout =>
        {
            let authored = formula_windows[0][0];
            if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits()
            {
                return Err(unsupported());
            }
            let Some(second) = tangent_projective_interval(roots[1].bracket()) else {
                return Err(unsupported());
            };
            // This is the negative-projective mirror of the short cell above.
            // Keep the seam/root represented only by the shared exact root;
            // a radius-scaled half-epsilon inset supplies the quantitative
            // radicand margin without inventing a zero-length seam member.
            let root_inset = f64::EPSILON / (2.0 * topology.formula_cylinders()[0].radius());
            let guard = crate::exact::bounded_polynomial::RootBracket {
                lo: second.hi().next_up(),
                hi: -root_inset,
            };
            let range = ParamRange::new(
                (2.0 * kcore::math::atan2(guard.lo, 1.0) + TAU).next_up(),
                (2.0 * kcore::math::atan2(guard.hi, 1.0) + TAU).next_down(),
            );
            if guard.lo >= guard.hi || !strict_guarded_range(range, authored) {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    range,
                    [Root(1), Root(0)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    range,
                    [Root(1), Root(0)],
                ),
            ]
        }
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => {
            let [Some(first_projective), Some(second_projective)] =
                roots.map(|root| tangent_projective_interval(root.bracket()))
            else {
                return Err(unsupported());
            };
            let authored = formula_windows[0][0];
            if authored.lo.to_bits() != 0.0_f64.to_bits()
                || authored.hi.to_bits() != TAU.to_bits()
                || second_projective.hi() >= first_projective.lo()
            {
                return Err(unsupported());
            }
            // The seam-centered perpendicular frame amplifies the opposite
            // trace's rounded position residual. Retain a fixed 1/4096 of
            // the exact positive projective cell at each root: this remains
            // inside the exact topology, separates both evaluator tubes, and
            // stays within the public edge tolerance of the physical joins.
            let guard =
                inward_projective_guard(second_projective.hi(), first_projective.lo(), 4096.0)
                    .ok_or_else(unsupported)?;
            let negative_angle = 2.0 * kcore::math::atan2(guard.lo, 1.0);
            let positive_angle = 2.0 * kcore::math::atan2(guard.hi, 1.0);
            let low = ParamRange::new(authored.lo.next_up(), positive_angle.next_down());
            let high = ParamRange::new((negative_angle + TAU).next_up(), authored.hi.next_down());
            if !strict_guarded_range(low, authored) || !strict_guarded_range(high, authored) {
                return Err(unsupported());
            }
            vec![
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    low,
                    [Seam(SkewCylinderSheet::Lower), Root(0)],
                ),
                (
                    SkewCylinderSheet::Lower,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    high,
                    [Root(1), Seam(SkewCylinderSheet::Lower)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    low,
                    [Seam(SkewCylinderSheet::Upper), Root(0)],
                ),
                (
                    SkewCylinderSheet::Upper,
                    SkewCylinderHalfAngleChart::Tangent,
                    guard,
                    high,
                    [Root(1), Seam(SkewCylinderSheet::Upper)],
                ),
            ]
        }
    };
    let mut formula_residuals = Vec::with_capacity(branch_specs.len());
    for (sheet, chart, projective_guard, guarded_range, _) in &branch_specs {
        let exact_radicand_lower = topology
            .positive_radicand_lower_bound(*chart, *projective_guard)
            .map_err(|_| unsupported())?;
        formula_residuals.push(
            super::subrange::certify_paired_skew_cylinder_folded_guarded_residuals(
                topology.formula_cylinders(),
                formula_windows,
                *guarded_range,
                *sheet,
                tolerance,
                super::subrange::FoldedRadicandGuard {
                    chart: *chart,
                    projective: *projective_guard,
                    source_lower: exact_radicand_lower,
                    stored_subdivision_budget: 0,
                    permit_sheet_tube_overlap: false,
                },
            )?,
        );
    }
    let formula_endpoint_parameters = [first, second].map(|endpoint| {
        let mut algebra = endpoint.algebra;
        let raw_longitude = algebra
            .authored_pcurve_derivs(1, endpoint.carrier_parameter, 0)
            .d[0]
            .x;
        let lifted = fit_full_period_parameter(raw_longitude, formula_windows[1][0])
            .expect("root evidence already retained an opposite longitude lift");
        algebra.longitude_offset = lifted - raw_longitude;
        [0, 1].map(|operand| {
            let uv = algebra
                .authored_pcurve_derivs(operand, endpoint.carrier_parameter, 0)
                .d[0];
            [uv.x, uv.y]
        })
    });
    let formula_endpoint_points = formula_endpoint_parameters.map(|parameters| {
        let points = [
            topology.formula_cylinders()[0].eval(parameters[0]),
            topology.formula_cylinders()[1].eval(parameters[1]),
        ];
        (points[0] + points[1]) * 0.5
    });
    for (parameters, point) in formula_endpoint_parameters
        .into_iter()
        .zip(formula_endpoint_points)
    {
        if parameters
            .into_iter()
            .flatten()
            .any(|value| !value.is_finite())
            || !point.to_array().into_iter().all(f64::is_finite)
        {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        let points = [
            topology.formula_cylinders()[0].eval(parameters[0]),
            topology.formula_cylinders()[1].eval(parameters[1]),
        ];
        if points[0].dist(points[1]) > tolerance {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
    }
    let (formula_seam_parameters, formula_seam_points) = if topology.positive_cell()
        == SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        && !seam_root_chart_layout
        && !short_across_seam_root_layout
    {
        let (parameters, points) =
            folded_seam_evidence(topology.formula_cylinders(), formula_windows, tolerance)?;
        (Some(parameters), Some(points))
    } else {
        (None, None)
    };
    let (formula_chart_join_parameters, formula_chart_join_points, chart_join_longitude) =
        if seam_root_chart_layout {
            let longitude = match topology.positive_cell() {
                SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => {
                    core::f64::consts::FRAC_PI_2
                }
                SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => {
                    3.0 * core::f64::consts::FRAC_PI_2
                }
            };
            let (parameters, points) = folded_regular_join_evidence(
                topology.formula_cylinders(),
                formula_windows,
                longitude,
                tolerance,
            )?;
            (Some(parameters), Some(points), Some(longitude))
        } else {
            (None, None, None)
        };
    let mut required_edge_tolerance = formula_residuals
        .iter()
        .flat_map(|residual| residual.residual_bounds())
        .fold(0.0, f64::max);
    for (residual, (_, _, _, _, endpoints)) in formula_residuals.iter().zip(&branch_specs) {
        let carrier = residual.carrier();
        let traces = residual.traces();
        for (parameter, point) in [residual.carrier_range().lo, residual.carrier_range().hi]
            .into_iter()
            .zip(endpoints.map(|endpoint| {
                formula_folded_endpoint_point(
                    endpoint,
                    formula_endpoint_points,
                    formula_seam_points,
                    formula_chart_join_points,
                )
            }))
        {
            required_edge_tolerance =
                required_edge_tolerance.max(carrier.eval(parameter).dist(point));
            for trace in traces {
                let uv = trace.pcurve().eval(parameter);
                required_edge_tolerance =
                    required_edge_tolerance.max(trace.surface().eval([uv.x, uv.y]).dist(point));
            }
        }
    }
    required_edge_tolerance = required_edge_tolerance.next_up();
    if !required_edge_tolerance.is_finite() || required_edge_tolerance > tolerance {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(PersistentSkewCylinderFoldedSupportCertificate {
        topology,
        formula_windows,
        formula_to_source,
        formula_root_longitudes,
        guarded_ranges: branch_specs
            .iter()
            .map(|(_, _, _, range, _)| *range)
            .collect(),
        formula_residuals,
        formula_branch_endpoints: branch_specs
            .into_iter()
            .map(|(_, _, _, _, endpoints)| endpoints)
            .collect(),
        formula_endpoint_parameters,
        formula_endpoint_points,
        formula_seam_parameters,
        formula_seam_points,
        formula_chart_join_parameters,
        formula_chart_join_points,
        chart_join_longitude,
        required_edge_tolerance,
        tolerance,
    })
}

/// Certify one one- or two-repeated-root touching-support family.
///
/// A single repeated root may lie strictly inside one tangent half chart,
/// exactly on the authored periodic seam, or exactly at the opposite
/// half-angle pole. An interior root uses the authored seam and one regular
/// chart join; seam- and opposite-pole roots use both regular chart joins.
/// The exact equal-radius orthogonal layout instead owns two repeated roots at
/// the two half-chart transitions and uses the authored seam. In every layout,
/// three guarded pieces per sheet cover the ordinary strict-positive cyclic
/// cells. The opposite-pole layout uses four per sheet so each side of the
/// root stays in a finite cotangent chart. Every smooth root continuation
/// retains its proof-owned root and port.
pub fn certify_persistent_skew_cylinder_touching_support(
    topology: SkewCylinderTouchingSupportTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
    work_limit: u64,
) -> Result<PersistentSkewCylinderTouchingSupportCertificate, IntersectionCertificateError> {
    validate_inputs(formula_windows, formula_to_source, tolerance)?;
    let required_work = persistent_skew_cylinder_touching_support_exact_work(&topology);
    if work_limit < required_work {
        return Err(unsupported());
    }
    let authored = formula_windows[0][0];
    if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits() {
        return Err(unsupported());
    }
    let roots = topology.roots();
    if !matches!(roots.len(), 1 | 2) || roots.iter().any(|root| !root.repeated()) {
        return Err(unsupported());
    }
    let root_on_seam = |root: SkewCylinderDiscriminantRoot| {
        let angular = root.angular_bracket();
        angular.lo.to_bits() == 0.0_f64.to_bits() && angular.hi.to_bits() == 0.0_f64.to_bits()
    };
    let root_evidence = roots
        .iter()
        .copied()
        .map(|root| {
            root_evidence(
                topology.topology(),
                root,
                formula_windows,
                root_on_seam(root),
                true,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    if root_evidence.iter().any(|evidence| {
        evidence
            .exact_heights
            .into_iter()
            .zip(formula_windows)
            .any(|(height, window)| !strictly_inside(height, window[1]))
    }) {
        return Err(unsupported());
    }
    let radius_scale = topology
        .formula_cylinders()
        .into_iter()
        .map(|cylinder| cylinder.radius())
        .fold(0.0_f64, f64::max);
    let root_inset = tolerance / (64.0 * radius_scale);
    if !root_inset.is_finite() || root_inset <= 0.0 {
        return Err(unsupported());
    }
    let seam_inset = 2.0_f64.powi(-40);
    let seam_after = authored.lo + seam_inset;
    let seam_before = authored.hi - seam_inset;

    use PersistentSkewCylinderTouchingSupportChartJoin::{First, Second};
    use PersistentSkewCylinderTouchingSupportEndpoint::{ChartJoin, Root, Seam};
    use PersistentSkewCylinderTouchingSupportRoot::{First as FirstRoot, Second as SecondRoot};
    let guarded_root = |root: SkewCylinderDiscriminantRoot| {
        let angular = root.angular_bracket();
        (
            (angular.strict_before_side() - root_inset).next_down(),
            (angular.strict_after_side() + root_inset).next_up(),
        )
    };
    let (branch_specs, chart_join_longitudes, seam_is_root) = if let [root] = roots {
        let angular = root.angular_bracket();
        let on_seam = root_on_seam(*root);
        let at_opposite_pole = angular.lo.to_bits() == core::f64::consts::PI.to_bits()
            && angular.hi.to_bits() == core::f64::consts::PI.to_bits();
        let root_in_first_half = angular.lo > 0.0 && angular.hi < core::f64::consts::PI;
        let root_in_second_half = angular.lo > core::f64::consts::PI && angular.hi < TAU;
        if !(on_seam || at_opposite_pole || root_in_first_half || root_in_second_half) {
            return Err(unsupported());
        }
        let (root_before, root_after) = guarded_root(*root);
        let chart_join_longitudes = if on_seam || at_opposite_pole {
            vec![
                core::f64::consts::FRAC_PI_2,
                3.0 * core::f64::consts::FRAC_PI_2,
            ]
        } else if root_in_first_half {
            vec![3.0 * core::f64::consts::FRAC_PI_2]
        } else {
            vec![core::f64::consts::FRAC_PI_2]
        };
        let lower = if on_seam {
            [
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(root_after, chart_join_longitudes[0].next_down()),
                    [
                        Root {
                            root: FirstRoot,
                            continuation: 0,
                        },
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Cotangent,
                    ParamRange::new(
                        chart_join_longitudes[0].next_up(),
                        chart_join_longitudes[1].next_down(),
                    ),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: Second,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(chart_join_longitudes[1].next_up(), root_before),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: Second,
                        },
                        Root {
                            root: FirstRoot,
                            continuation: 1,
                        },
                    ],
                ),
            ]
            .to_vec()
        } else if at_opposite_pole {
            [
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(seam_after, chart_join_longitudes[0].next_down()),
                    [
                        Seam(SkewCylinderSheet::Lower),
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Cotangent,
                    ParamRange::new(chart_join_longitudes[0].next_up(), root_before),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                        Root {
                            root: FirstRoot,
                            continuation: 0,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Cotangent,
                    ParamRange::new(root_after, chart_join_longitudes[1].next_down()),
                    [
                        Root {
                            root: FirstRoot,
                            continuation: 1,
                        },
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: Second,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(chart_join_longitudes[1].next_up(), seam_before),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: Second,
                        },
                        Seam(SkewCylinderSheet::Lower),
                    ],
                ),
            ]
            .to_vec()
        } else if root_in_first_half {
            [
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(seam_after, root_before),
                    [
                        Seam(SkewCylinderSheet::Lower),
                        Root {
                            root: FirstRoot,
                            continuation: 0,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Cotangent,
                    ParamRange::new(root_after, chart_join_longitudes[0].next_down()),
                    [
                        Root {
                            root: FirstRoot,
                            continuation: 1,
                        },
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(chart_join_longitudes[0].next_up(), seam_before),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                        Seam(SkewCylinderSheet::Lower),
                    ],
                ),
            ]
            .to_vec()
        } else {
            [
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(seam_after, chart_join_longitudes[0].next_down()),
                    [
                        Seam(SkewCylinderSheet::Lower),
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Cotangent,
                    ParamRange::new(chart_join_longitudes[0].next_up(), root_before),
                    [
                        ChartJoin {
                            sheet: SkewCylinderSheet::Lower,
                            join: First,
                        },
                        Root {
                            root: FirstRoot,
                            continuation: 0,
                        },
                    ],
                ),
                (
                    SkewCylinderHalfAngleChart::Tangent,
                    ParamRange::new(root_after, seam_before),
                    [
                        Root {
                            root: FirstRoot,
                            continuation: 1,
                        },
                        Seam(SkewCylinderSheet::Lower),
                    ],
                ),
            ]
            .to_vec()
        };
        (
            touching_support_sheet_specs(lower),
            chart_join_longitudes,
            on_seam,
        )
    } else if let [first, second] = roots {
        let first_angular = first.angular_bracket();
        let second_angular = second.angular_bracket();
        if first_angular.lo.to_bits() != core::f64::consts::FRAC_PI_2.to_bits()
            || first_angular.hi.to_bits() != core::f64::consts::FRAC_PI_2.to_bits()
            || second_angular.lo.to_bits() != (3.0 * core::f64::consts::FRAC_PI_2).to_bits()
            || second_angular.hi.to_bits() != (3.0 * core::f64::consts::FRAC_PI_2).to_bits()
        {
            return Err(unsupported_reason(
                "double touching support is not the exact orthogonal chart-root layout",
            ));
        }
        let (first_before, first_after) = guarded_root(*first);
        let (second_before, second_after) = guarded_root(*second);
        let ranges = vec![
            (
                SkewCylinderHalfAngleChart::Tangent,
                ParamRange::new(seam_after, first_before),
                [
                    Seam(SkewCylinderSheet::Lower),
                    Root {
                        root: FirstRoot,
                        continuation: 0,
                    },
                ],
            ),
            (
                SkewCylinderHalfAngleChart::Cotangent,
                ParamRange::new(first_after, second_before),
                [
                    Root {
                        root: FirstRoot,
                        continuation: 1,
                    },
                    Root {
                        root: SecondRoot,
                        continuation: 0,
                    },
                ],
            ),
            (
                SkewCylinderHalfAngleChart::Tangent,
                ParamRange::new(second_after, seam_before),
                [
                    Root {
                        root: SecondRoot,
                        continuation: 1,
                    },
                    Seam(SkewCylinderSheet::Lower),
                ],
            ),
        ];
        (touching_support_sheet_specs(ranges), Vec::new(), false)
    } else {
        return Err(unsupported());
    };
    if branch_specs.len() as u64
        * (PERSISTENT_SKEW_CYLINDER_OPEN_SPAN_WORK
            + SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BOUND_WORK)
        != required_work
        || branch_specs
            .iter()
            .any(|(_, _, range, _)| !strict_guarded_range(*range, authored))
    {
        return Err(unsupported());
    }

    let mut formula_residuals = Vec::with_capacity(branch_specs.len());
    for (sheet, chart, guarded_range, _) in &branch_specs {
        let projective = projective_guard(*chart, *guarded_range).ok_or_else(|| {
            unsupported_reason("touching support projective guard is not finite and ordered")
        })?;
        let source_lower = topology
            .positive_radicand_lower_bound(*chart, projective)
            .map_err(|_error| {
                unsupported_reason("touching support guard lacks an exact positive radicand bound")
            })?;
        formula_residuals.push(
            super::subrange::certify_paired_skew_cylinder_folded_guarded_residuals(
                topology.formula_cylinders(),
                formula_windows,
                *guarded_range,
                *sheet,
                tolerance,
                super::subrange::FoldedRadicandGuard {
                    chart: *chart,
                    projective,
                    source_lower,
                    stored_subdivision_budget:
                        super::SKEW_CYLINDER_TOUCHING_SUPPORT_RADICAND_BERNSTEIN_CELLS,
                    permit_sheet_tube_overlap: true,
                },
            )?,
        );
    }

    let mut formula_root_parameters = Vec::with_capacity(root_evidence.len());
    let mut formula_root_points = Vec::with_capacity(root_evidence.len());
    for evidence in &root_evidence {
        let mut root_algebra = evidence.algebra;
        let raw_root_longitude = root_algebra
            .authored_pcurve_derivs(1, evidence.carrier_parameter, 0)
            .d[0]
            .x;
        let lifted_root = fit_full_period_parameter(raw_root_longitude, formula_windows[1][0])
            .expect("root evidence already retained an opposite longitude lift");
        root_algebra.longitude_offset = lifted_root - raw_root_longitude;
        let parameters = [0, 1].map(|operand| {
            let uv = root_algebra
                .authored_pcurve_derivs(operand, evidence.carrier_parameter, 0)
                .d[0];
            [uv.x, uv.y]
        });
        let root_points = [
            topology.formula_cylinders()[0].eval(parameters[0]),
            topology.formula_cylinders()[1].eval(parameters[1]),
        ];
        if root_points[0].dist(root_points[1]) > tolerance {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        formula_root_parameters.push(parameters);
        formula_root_points.push((root_points[0] + root_points[1]) * 0.5);
    }
    // Root endpoint lookup is public and must remain total for the sealed enum
    // even in a one-root topology. The second slot is therefore a harmless
    // duplicate unless the topology actually admits `Second`.
    let first_root_parameters = formula_root_parameters[0];
    let first_root_point = formula_root_points[0];
    let formula_root_parameters = [
        first_root_parameters,
        formula_root_parameters
            .get(1)
            .copied()
            .unwrap_or(first_root_parameters),
    ];
    let formula_root_points = [
        first_root_point,
        formula_root_points
            .get(1)
            .copied()
            .unwrap_or(first_root_point),
    ];
    let (formula_seam_parameters, formula_seam_points) = if seam_is_root {
        ([formula_root_parameters[0]; 2], [formula_root_points[0]; 2])
    } else {
        folded_regular_join_evidence(
            topology.formula_cylinders(),
            formula_windows,
            authored.lo,
            tolerance,
        )
        .map_err(|_| unsupported_reason("touching support lacks authored-seam join evidence"))?
    };
    let chart_join_evidence = chart_join_longitudes
        .iter()
        .map(|longitude| {
            folded_regular_join_evidence(
                topology.formula_cylinders(),
                formula_windows,
                *longitude,
                tolerance,
            )
            .map_err(|_| unsupported_reason("touching support lacks chart-join evidence"))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let fallback_chart_join = (formula_seam_parameters, formula_seam_points);
    let first_chart_join = chart_join_evidence
        .first()
        .copied()
        .unwrap_or(fallback_chart_join);
    // Keep public endpoint lookup total even when the partition needs only
    // one chart join. The longitude lookup remains the proof-owned admission
    // gate and rejects `Second` for that layout.
    let second_chart_join = chart_join_evidence
        .get(1)
        .copied()
        .unwrap_or(first_chart_join);
    let formula_chart_join_parameters = [first_chart_join.0, second_chart_join.0];
    let formula_chart_join_points = [first_chart_join.1, second_chart_join.1];

    let point_for_endpoint = |endpoint| match endpoint {
        Root { root, .. } => formula_root_points[root.index()],
        Seam(sheet) => formula_seam_points[sheet_ordinal(sheet)],
        ChartJoin { sheet, join } => {
            formula_chart_join_points[join.ordinal()][sheet_ordinal(sheet)]
        }
    };
    let mut required_edge_tolerance = formula_residuals
        .iter()
        .flat_map(|residual| residual.residual_bounds())
        .fold(0.0, f64::max);
    for (residual, (_, _, _, endpoints)) in formula_residuals.iter().zip(&branch_specs) {
        let carrier = residual.carrier();
        let traces = residual.traces();
        for (parameter, point) in [residual.carrier_range().lo, residual.carrier_range().hi]
            .into_iter()
            .zip(endpoints.map(point_for_endpoint))
        {
            required_edge_tolerance =
                required_edge_tolerance.max(carrier.eval(parameter).dist(point));
            for trace in traces {
                let uv = trace.pcurve().eval(parameter);
                required_edge_tolerance =
                    required_edge_tolerance.max(trace.surface().eval([uv.x, uv.y]).dist(point));
            }
        }
    }
    required_edge_tolerance = required_edge_tolerance.next_up();
    if !required_edge_tolerance.is_finite() || required_edge_tolerance > tolerance {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }

    Ok(PersistentSkewCylinderTouchingSupportCertificate {
        topology,
        formula_windows,
        formula_to_source,
        formula_root_longitudes: root_evidence
            .iter()
            .map(|evidence| evidence.formula_longitude_enclosures[0])
            .collect(),
        chart_join_longitudes,
        guarded_ranges: branch_specs.iter().map(|(_, _, range, _)| *range).collect(),
        formula_residuals,
        formula_branch_endpoints: branch_specs
            .into_iter()
            .map(|(_, _, _, endpoints)| endpoints)
            .collect(),
        formula_root_parameters,
        formula_root_points,
        formula_seam_parameters,
        formula_seam_points,
        formula_chart_join_parameters,
        formula_chart_join_points,
        required_edge_tolerance,
        tolerance,
    })
}

type TouchingSupportBaseSpec = (
    SkewCylinderHalfAngleChart,
    ParamRange,
    [PersistentSkewCylinderTouchingSupportEndpoint; 2],
);

type TouchingSupportBranchSpec = (
    SkewCylinderSheet,
    SkewCylinderHalfAngleChart,
    ParamRange,
    [PersistentSkewCylinderTouchingSupportEndpoint; 2],
);

fn touching_support_sheet_specs(
    lower: Vec<TouchingSupportBaseSpec>,
) -> Vec<TouchingSupportBranchSpec> {
    let upper = lower
        .iter()
        .copied()
        .map(|(chart, range, endpoints)| {
            let endpoints = endpoints.map(|endpoint| match endpoint {
                PersistentSkewCylinderTouchingSupportEndpoint::Root { root, continuation } => {
                    PersistentSkewCylinderTouchingSupportEndpoint::Root { root, continuation }
                }
                PersistentSkewCylinderTouchingSupportEndpoint::Seam(_) => {
                    PersistentSkewCylinderTouchingSupportEndpoint::Seam(SkewCylinderSheet::Upper)
                }
                PersistentSkewCylinderTouchingSupportEndpoint::ChartJoin { join, .. } => {
                    PersistentSkewCylinderTouchingSupportEndpoint::ChartJoin {
                        sheet: SkewCylinderSheet::Upper,
                        join,
                    }
                }
            });
            (chart, range, endpoints)
        })
        .collect::<Vec<_>>();
    lower
        .into_iter()
        .map(|(chart, range, endpoints)| (SkewCylinderSheet::Lower, chart, range, endpoints))
        .chain(
            upper.into_iter().map(|(chart, range, endpoints)| {
                (SkewCylinderSheet::Upper, chart, range, endpoints)
            }),
        )
        .collect()
}

fn projective_guard(
    chart: SkewCylinderHalfAngleChart,
    range: ParamRange,
) -> Option<crate::exact::bounded_polynomial::RootBracket> {
    let projective = [range.lo, range.hi].map(|parameter| {
        let (sin, cos) = kcore::math::sincos(parameter);
        match chart {
            SkewCylinderHalfAngleChart::Tangent => sin / (1.0 + cos),
            SkewCylinderHalfAngleChart::Cotangent => sin / (1.0 - cos),
        }
    });
    if projective.into_iter().any(|value| !value.is_finite()) {
        return None;
    }
    let lo = projective[0].min(projective[1]);
    let hi = projective[0].max(projective[1]);
    (lo < hi).then_some(crate::exact::bounded_polynomial::RootBracket { lo, hi })
}

fn strict_guarded_range(range: ParamRange, authored: ParamRange) -> bool {
    range.is_finite() && range.width() > 0.0 && range.lo > authored.lo && range.hi < authored.hi
}

fn inward_projective_guard(
    left: f64,
    right: f64,
    divisor: f64,
) -> Option<crate::exact::bounded_polynomial::RootBracket> {
    if !left.is_finite()
        || !right.is_finite()
        || !divisor.is_finite()
        || divisor <= 2.0
        || left >= right
    {
        return None;
    }
    let inset = (right - left) / divisor;
    let lo = (left + inset).max(left.next_up());
    let hi = (right - inset).min(right.next_down());
    (lo < hi).then_some(crate::exact::bounded_polynomial::RootBracket { lo, hi })
}

fn folded_seam_evidence(
    cylinders: [Cylinder; 2],
    formula_windows: [[ParamRange; 2]; 2],
    tolerance: f64,
) -> Result<FormulaFoldedSeamEvidence, IntersectionCertificateError> {
    folded_regular_join_evidence(
        cylinders,
        formula_windows,
        formula_windows[0][0].lo,
        tolerance,
    )
}

fn folded_regular_join_evidence(
    cylinders: [Cylinder; 2],
    formula_windows: [[ParamRange; 2]; 2],
    carrier_parameter: f64,
    tolerance: f64,
) -> Result<FormulaFoldedSeamEvidence, IntersectionCertificateError> {
    if !formula_windows[0][0].contains(carrier_parameter) {
        return Err(unsupported());
    }
    let mut parameters = [[[0.0; 2]; 2]; 2];
    let mut points = [Vec3::default(); 2];
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        let ordinal = sheet_ordinal(sheet);
        let mut algebra = build_algebra(cylinders, formula_windows[0][0], sheet)
            .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
        let raw_longitude = algebra.authored_pcurve_derivs(1, carrier_parameter, 0).d[0].x;
        let lifted = fit_full_period_parameter(raw_longitude, formula_windows[1][0])
            .ok_or_else(unsupported)?;
        algebra.longitude_offset = lifted - raw_longitude;
        parameters[ordinal] = [0, 1].map(|operand| {
            let uv = algebra
                .authored_pcurve_derivs(operand, carrier_parameter, 0)
                .d[0];
            [uv.x, uv.y]
        });
        if parameters[ordinal]
            .into_iter()
            .zip(formula_windows)
            .any(|(uv, window)| !window[0].contains(uv[0]) || !window[1].contains(uv[1]))
        {
            return Err(unsupported());
        }
        let source_points = [
            cylinders[0].eval(parameters[ordinal][0]),
            cylinders[1].eval(parameters[ordinal][1]),
        ];
        if source_points[0].dist(source_points[1]) > tolerance {
            return Err(IntersectionCertificateError::InvalidTraceFamily);
        }
        points[ordinal] = (source_points[0] + source_points[1]) * 0.5;
    }
    Ok((parameters, points))
}

fn formula_folded_endpoint_point(
    endpoint: PersistentSkewCylinderFoldedSupportEndpoint,
    roots: [Vec3; 2],
    seams: Option<[Vec3; 2]>,
    chart_joins: Option<[Vec3; 2]>,
) -> Vec3 {
    match endpoint {
        PersistentSkewCylinderFoldedSupportEndpoint::Root(ordinal) => roots[ordinal],
        PersistentSkewCylinderFoldedSupportEndpoint::Seam(sheet) => {
            seams.expect("seam branch retains seam points")[sheet_ordinal(sheet)]
        }
        PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(sheet) => {
            chart_joins.expect("chart-split branch retains chart-join points")[sheet_ordinal(sheet)]
        }
    }
}

/// Plan exact axial-bound root relations for one isolated support tangency in
/// two full-period finite cylinder windows.
pub fn plan_persistent_skew_cylinder_support_contact_boundaries(
    topology: &SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
) -> Result<PersistentSkewCylinderSupportContactBoundaryPlan, IntersectionCertificateError> {
    validate_inputs(formula_windows, formula_to_source, tolerance)?;
    let evidence = support_root_evidence(topology, formula_windows)?;
    let (_, bits) = planned_axial_locations(evidence.exact_heights, formula_windows)?;
    Ok(PersistentSkewCylinderSupportContactBoundaryPlan { bits })
}

/// Certify one isolated support point, requiring the exact work advertised by
/// [`plan_persistent_skew_cylinder_support_contact_boundaries`] whenever an
/// authored axial bound owns the point.
pub fn certify_persistent_skew_cylinder_support_contact(
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
    root_relation_work_limit: u64,
) -> Result<PersistentSkewCylinderSupportContactCertificate, IntersectionCertificateError> {
    validate_inputs(formula_windows, formula_to_source, tolerance)?;
    let evidence = support_root_evidence(&topology, formula_windows)?;
    let (formula_axial_locations, bits) =
        planned_axial_locations(evidence.exact_heights, formula_windows)?;
    let boundary_plan = PersistentSkewCylinderSupportContactBoundaryPlan { bits };
    if root_relation_work_limit < boundary_plan.work() {
        return Err(unsupported());
    }
    for formula_operand in 0..2 {
        let boundary = match formula_axial_locations[formula_operand] {
            PersistentSkewCylinderSupportContactAxialLocation::Interior => continue,
            PersistentSkewCylinderSupportContactAxialLocation::Lower => {
                SkewCylinderAxialBoundary::Lower
            }
            PersistentSkewCylinderSupportContactAxialLocation::Upper => {
                SkewCylinderAxialBoundary::Upper
            }
        };
        let bound = match boundary {
            SkewCylinderAxialBoundary::Lower => formula_windows[formula_operand][1].lo,
            SkewCylinderAxialBoundary::Upper => formula_windows[formula_operand][1].hi,
        };
        let provenance = SkewCylinderAxialBoundProvenance {
            source_operand: formula_to_source[formula_operand],
            boundary,
            value: bound,
        };
        if !topology
            .support_root_matches_axial_bound(evidence.root, formula_to_source, provenance)
            .map_err(|_| unsupported())?
        {
            return Err(unsupported());
        }
    }

    let carrier_parameter = evidence.carrier_parameter;
    let mut algebra = evidence.algebra;
    let raw_longitude = algebra.authored_pcurve_derivs(1, carrier_parameter, 0).d[0].x;
    let lifted =
        fit_full_period_parameter(raw_longitude, formula_windows[1][0]).ok_or_else(unsupported)?;
    algebra.longitude_offset = lifted - raw_longitude;
    let formula_parameters = [0, 1].map(|operand| {
        let uv = algebra
            .authored_pcurve_derivs(operand, carrier_parameter, 0)
            .d[0];
        [uv.x, uv.y]
    });
    if formula_parameters
        .into_iter()
        .flatten()
        .any(|value| !value.is_finite())
    {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    let formula_points = [
        topology.formula_cylinders()[0].eval(formula_parameters[0]),
        topology.formula_cylinders()[1].eval(formula_parameters[1]),
    ];
    let residual = formula_points[0].dist(formula_points[1]);
    if !residual.is_finite() || residual > tolerance {
        return Err(IntersectionCertificateError::InvalidTraceFamily);
    }
    Ok(PersistentSkewCylinderSupportContactCertificate {
        topology,
        formula_windows,
        formula_to_source,
        formula_axial_locations,
        formula_longitude_enclosures: evidence.formula_longitude_enclosures,
        boundary_plan,
        carrier_parameter,
        tolerance,
    })
}

#[derive(Debug, Clone, Copy)]
struct SupportRootEvidence {
    root: SkewCylinderDiscriminantRoot,
    algebra: BranchAlgebra,
    carrier_parameter: f64,
    exact_heights: [Interval; 2],
    formula_longitude_enclosures: [Interval; 2],
}

fn validate_inputs(
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
) -> Result<(), IntersectionCertificateError> {
    if !matches!(formula_to_source, [0, 1] | [1, 0])
        || !tolerance.is_finite()
        || tolerance < 0.0
        || formula_windows
            .iter()
            .flatten()
            .any(|range| !range.is_finite() || range.lo > range.hi)
        || formula_windows
            .iter()
            .any(|window| window[0].width() != TAU)
    {
        return Err(unsupported());
    }
    Ok(())
}

fn support_root_evidence(
    topology: &SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
) -> Result<SupportRootEvidence, IntersectionCertificateError> {
    let root = topology.isolated_support_root().ok_or_else(unsupported)?;
    root_evidence(topology, root, formula_windows, false, false)
}

fn root_evidence(
    topology: &SkewCylinderDiscriminantContactTopologyCertificate,
    root: SkewCylinderDiscriminantRoot,
    formula_windows: [[ParamRange; 2]; 2],
    allow_canonical_periodic_seam: bool,
    allow_opposite_periodic_seam: bool,
) -> Result<SupportRootEvidence, IntersectionCertificateError> {
    if !topology.roots().contains(&root) {
        return Err(unsupported_reason(
            "folded support root is foreign to its exact topology",
        ));
    }
    let angular = root.angular_bracket();
    if !angular.lo.is_finite() || !angular.hi.is_finite() || angular.lo > angular.hi {
        return Err(unsupported_reason(
            "folded support angular root enclosure is invalid",
        ));
    }
    let canonical_parameter = if angular.lo == angular.hi {
        angular.lo
    } else {
        angular.lo / 2.0 + angular.hi / 2.0
    };
    let carrier_parameter =
        fit_full_period_parameter(canonical_parameter, formula_windows[0][0])
            .ok_or_else(|| unsupported_reason("folded support root has no canonical chart lift"))?;
    let algebra = build_algebra(
        topology.formula_cylinders(),
        formula_windows[0][0],
        SkewCylinderSheet::Lower,
    )
    .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    let proof = coefficient_proof(algebra)
        .ok_or_else(|| unsupported_reason("folded support root lacks coefficient evidence"))?;
    let [cosine, sine] = projective_root_trig_intervals(root)?;
    let exact_m = proof
        .m_true
        .interval(cosine, sine)
        .ok_or_else(|| unsupported_reason("folded support root M enclosure is invalid"))?;
    let exact_v = finite_interval(
        (Interval::point(-1.0) * exact_m)
            .checked_div(proof.a_true)
            .ok_or_else(|| unsupported_reason("folded support root axial division is singular"))?,
    )
    .ok_or_else(|| unsupported_reason("folded support root axial enclosure is invalid"))?;
    let exact_coordinates = [0, 1, 2].map(|coordinate| {
        proof.harmonics_true[coordinate]
            .interval(cosine, sine)
            .and_then(|value| finite_interval(value + exact_v * proof.directions_true[coordinate]))
            .ok_or_else(|| unsupported_reason("folded support root coordinate is invalid"))
    });
    let [exact_x, exact_y, exact_z] = exact_coordinates;
    let [exact_x, exact_y, exact_z] = [exact_x?, exact_y?, exact_z?];
    let normalized_x = exact_x
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(|| unsupported_reason("folded support root radial x is invalid"))?;
    let normalized_y = exact_y
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(|| unsupported_reason("folded support root radial y is invalid"))?;
    if normalized_x.contains_zero() && normalized_y.contains_zero() {
        return Err(unsupported_reason(
            "folded support root radial direction is singular",
        ));
    }
    let exact_second_height = exact_z
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(|| unsupported_reason("folded support opposite height is invalid"))?;
    let first_longitude = lift_interval_near(
        Interval::new(angular.lo, angular.hi),
        carrier_parameter,
        formula_windows[0][0],
    )
    .or_else(|| {
        let range = formula_windows[0][0];
        (allow_canonical_periodic_seam
            && range.lo.to_bits() == 0.0_f64.to_bits()
            && range.hi.to_bits() == TAU.to_bits()
            && angular.lo.to_bits() == 0.0_f64.to_bits()
            && angular.hi.to_bits() == 0.0_f64.to_bits())
        .then_some(Interval::point(range.lo))
    })
    .ok_or_else(|| {
        unsupported_reason("folded support canonical root enclosure crosses its authored seam")
    })?;
    let second_representative = algebra.authored_pcurve_derivs(1, carrier_parameter, 0).d[0].x;
    let raw_second_longitude = longitude_interval(normalized_x, normalized_y);
    let second_longitude = lift_interval_near(
        raw_second_longitude,
        second_representative,
        formula_windows[1][0],
    )
    .or_else(|| {
        let range = formula_windows[1][0];
        (allow_opposite_periodic_seam
            && range.lo.to_bits() == 0.0_f64.to_bits()
            && range.hi.to_bits() == TAU.to_bits()
            && raw_second_longitude.contains_zero())
        .then_some(Interval::new(range.lo, range.hi))
    })
    .ok_or_else(|| {
        unsupported_reason("folded support opposite root enclosure crosses its authored seam")
    })?;
    Ok(SupportRootEvidence {
        root,
        algebra,
        carrier_parameter,
        exact_heights: [exact_v, exact_second_height],
        formula_longitude_enclosures: [first_longitude, second_longitude],
    })
}

fn projective_root_trig_intervals(
    root: SkewCylinderDiscriminantRoot,
) -> Result<[Interval; 2], IntersectionCertificateError> {
    let bracket = root.bracket();
    let projective = Interval::new(bracket.lo, bracket.hi);
    let square = projective.square();
    let denominator = Interval::point(1.0) + square;
    let cosine_numerator = match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent => Interval::point(1.0) - square,
        SkewCylinderHalfAngleChart::Cotangent => square - Interval::point(1.0),
    };
    let cosine = cosine_numerator
        .checked_div(denominator)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    let sine = (Interval::point(2.0) * projective)
        .checked_div(denominator)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    Ok([cosine, sine])
}

fn planned_axial_locations(
    exact_heights: [Interval; 2],
    formula_windows: [[ParamRange; 2]; 2],
) -> Result<
    ([PersistentSkewCylinderSupportContactAxialLocation; 2], u8),
    IntersectionCertificateError,
> {
    let mut bits = 0_u8;
    let mut locations = [PersistentSkewCylinderSupportContactAxialLocation::Interior; 2];
    for operand in 0..2 {
        let height = exact_heights[operand];
        let range = formula_windows[operand][1];
        locations[operand] = if strictly_inside(height, range) {
            PersistentSkewCylinderSupportContactAxialLocation::Interior
        } else if height.contains(range.lo) && height.hi() < range.hi {
            bits |= 1 << (2 * operand);
            PersistentSkewCylinderSupportContactAxialLocation::Lower
        } else if height.contains(range.hi) && height.lo() > range.lo {
            bits |= 1 << (2 * operand + 1);
            PersistentSkewCylinderSupportContactAxialLocation::Upper
        } else {
            return Err(unsupported());
        };
    }
    Ok((locations, bits))
}

fn lift_interval_near(
    interval: Interval,
    representative: f64,
    range: ParamRange,
) -> Option<Interval> {
    if !interval.lo().is_finite()
        || !interval.hi().is_finite()
        || interval.lo() > interval.hi()
        || !representative.is_finite()
        || !range.is_finite()
        || range.width() != TAU
    {
        return None;
    }
    let lift = |value: f64| value + ((representative - value) / TAU).round() * TAU;
    let first = lift(interval.lo());
    let second = lift(interval.hi());
    let lifted = Interval::new(first.min(second), first.max(second));
    (lifted.lo() > range.lo && lifted.hi() < range.hi).then_some(lifted)
}

fn strictly_inside(value: Interval, range: ParamRange) -> bool {
    value.lo() > range.lo && value.hi() < range.hi
}

fn fit_full_period_parameter(candidate: f64, range: ParamRange) -> Option<f64> {
    if !candidate.is_finite() || !range.is_finite() || range.width() != TAU {
        return None;
    }
    let turns = ((range.lo - candidate) / TAU).ceil();
    let lifted = candidate + turns * TAU;
    (lifted.is_finite() && range.contains(lifted)).then_some(lifted)
}

fn permute_formula_to_source<T: Copy>(formula: [T; 2], permutation: [usize; 2]) -> [T; 2] {
    let mut source = formula;
    for formula_slot in 0..2 {
        source[permutation[formula_slot]] = formula[formula_slot];
    }
    source
}

fn unsupported() -> IntersectionCertificateError {
    unsupported_reason(UNSUPPORTED_REASON)
}

fn unsupported_reason(reason: &'static str) -> IntersectionCertificateError {
    IntersectionCertificateError::UnsupportedCarrierParameterization { reason }
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    use super::*;
    use crate::{
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, SkewCylinderExactDiscriminantTopology,
        SkewCylinderFoldedSupportCellLocation, certify_persistent_skew_cylinder_folded_support,
        certify_persistent_skew_cylinder_touching_support,
        certify_skew_cylinder_folded_support_topology,
        certify_skew_cylinder_touching_support_topology, classify_skew_cylinder_exact_discriminant,
    };

    fn cylinders(offset: f64) -> [Cylinder; 2] {
        let first = Cylinder::new(Frame::world(), 1.0).unwrap();
        let second = Cylinder::new(
            Frame::new(
                Point3::new(0.0, offset, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            )
            .unwrap(),
            2.0,
        )
        .unwrap();
        [first, second]
    }

    fn seam_cylinders(offset: f64) -> [Cylinder; 2] {
        let first = Cylinder::new(Frame::world(), 1.0).unwrap();
        let second = Cylinder::new(
            Frame::new(
                Point3::new(offset, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            2.0,
        )
        .unwrap();
        [first, second]
    }

    fn bounded_seam_cylinders(frame: Frame, offset: f64) -> [Cylinder; 2] {
        [
            Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 2.25), 1.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    frame.origin() + frame.x() * offset - frame.y() * 1.25,
                    frame.y(),
                    frame.x(),
                )
                .unwrap(),
                2.0,
            )
            .unwrap(),
        ]
    }

    fn touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame, 1.0).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() + frame.y() * 0.5, frame.x(), frame.y()).unwrap(),
                1.5,
            )
            .unwrap(),
        ]
    }

    fn seam_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame, 1.0).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() - frame.x() * 0.5, frame.y(), frame.x()).unwrap(),
                1.5,
            )
            .unwrap(),
        ]
    }

    fn bounded_opposite_pole_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap(),
            Cylinder::new(
                Frame::new(
                    frame.origin() + frame.x() * 0.125 - frame.y() * 0.5,
                    frame.y(),
                    frame.x(),
                )
                .unwrap(),
                0.375,
            )
            .unwrap(),
        ]
    }

    fn double_touching_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame, 1.0).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin(), frame.x(), frame.y()).unwrap(),
                1.0,
            )
            .unwrap(),
        ]
    }

    fn seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame, 1.0).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() + frame.y() * 2.0, frame.x(), frame.y()).unwrap(),
                2.0,
            )
            .unwrap(),
        ]
    }

    fn seam_root_across_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        [
            Cylinder::new(frame, 1.0).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() - frame.y() * 2.0, frame.x(), frame.y()).unwrap(),
                2.0,
            )
            .unwrap(),
        ]
    }

    fn short_seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
        let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
        let first_radius = 1.0;
        let second_radius = 2.0;
        let offset = second_radial * second_radius - frame.x() * first_radius;
        [
            Cylinder::new(frame, first_radius).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
                second_radius,
            )
            .unwrap(),
        ]
    }

    fn bounded_short_seam_root_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
        let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
        let first_radius = 0.0625;
        let second_radius = 0.125;
        let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
            - frame.z() * 0.125;
        [
            Cylinder::new(frame, first_radius).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
                second_radius,
            )
            .unwrap(),
        ]
    }

    fn bounded_short_seam_root_across_folded_support_cylinders(frame: Frame) -> [Cylinder; 2] {
        let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
        let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
        let first_radius = 0.0625;
        let second_radius = 0.125;
        let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
            - frame.z() * 0.125;
        let reversed_first = Frame::new(frame.origin() + frame.z(), -frame.z(), frame.x()).unwrap();
        [
            Cylinder::new(reversed_first, first_radius).unwrap(),
            Cylinder::new(
                Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
                second_radius,
            )
            .unwrap(),
        ]
    }

    fn touching_support_windows() -> [[ParamRange; 2]; 2] {
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-2.0, 2.0)],
        ]
    }

    fn bounded_touching_support_windows() -> [[ParamRange; 2]; 2] {
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 1.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 1.0)],
        ]
    }

    fn bounded_windows() -> [[ParamRange; 2]; 2] {
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 4.5)],
            [ParamRange::new(0.0, TAU), ParamRange::new(0.0, 2.5)],
        ]
    }

    fn windows() -> [[ParamRange; 2]; 2] {
        [
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
            [ParamRange::new(0.0, TAU), ParamRange::new(-1.0, 1.0)],
        ]
    }

    #[test]
    fn exact_isolated_support_root_mints_a_persistent_point_but_rooted_arc_does_not() {
        let exact = match classify_skew_cylinder_exact_discriminant(
            cylinders(3.0),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected contact topology, got {other:?}"),
        };
        assert_eq!(exact.roots().len(), 1);
        assert!(exact.roots()[0].repeated());
        let certified =
            certify_persistent_skew_cylinder_support_contact(exact, windows(), [0, 1], 1.0e-9, 0)
                .unwrap();
        assert!(certified.point().dist(Point3::new(0.0, 1.0, 0.0)) <= 1.0e-12);
        assert_eq!(certified.source_surface_parameters()[0][1], 0.0);

        let mut boundary_windows = windows();
        boundary_windows[0][1] = ParamRange::new(0.0, 1.0);
        let plan = plan_persistent_skew_cylinder_support_contact_boundaries(
            certified.topology(),
            boundary_windows,
            [0, 1],
            1.0e-9,
        );
        let plan = plan.unwrap();
        assert_eq!(plan.bits(), 1);
        assert_eq!(
            plan.work(),
            SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
        );
        assert!(
            certify_persistent_skew_cylinder_support_contact(
                certified.topology().clone(),
                boundary_windows,
                [0, 1],
                1.0e-9,
                plan.work() - 1,
            )
            .is_err()
        );
        let boundary = certify_persistent_skew_cylinder_support_contact(
            certified.topology().clone(),
            boundary_windows,
            [0, 1],
            1.0e-9,
            plan.work(),
        )
        .unwrap();
        assert_eq!(
            boundary.source_axial_locations(),
            [
                PersistentSkewCylinderSupportContactAxialLocation::Lower,
                PersistentSkewCylinderSupportContactAxialLocation::Interior,
            ]
        );

        let rooted = match classify_skew_cylinder_exact_discriminant(
            cylinders(3.0_f64.next_down()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected rooted contact topology, got {other:?}"),
        };
        assert!(rooted.roots().len() >= 2);
        let folded = certify_skew_cylinder_folded_support_topology(rooted.clone()).unwrap();
        assert!(folded.roots().iter().all(|root| !root.repeated()));
        assert_eq!(folded.roots(), rooted.roots());
        assert_eq!(
            folded.positive_cell(),
            SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                folded.clone(),
                windows(),
                [0, 1],
                1.0e-9,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            folded,
            windows(),
            [0, 1],
            1.0e-9,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
        assert!(folded.guarded_ranges()[0].width() > 0.0);
        assert_eq!(
            folded
                .formula_residuals()
                .iter()
                .map(|residual| residual.sheet())
                .collect::<Vec<_>>(),
            vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
        );
        assert!(folded.endpoint_points().iter().all(|point| point.y > 0.0));
        assert!(
            certify_persistent_skew_cylinder_support_contact(rooted, windows(), [0, 1], 1.0e-9, 0,)
                .is_err()
        );
    }

    #[test]
    fn across_seam_folded_support_splits_into_four_exactly_joined_members() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            seam_cylinders(3.0_f64.next_down()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected rooted contact topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        assert_eq!(
            topology.positive_cell(),
            SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                windows(),
                [0, 1],
                1.0e-9,
                SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            windows(),
            [0, 1],
            1.0e-9,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.formula_residuals().len(), 4);
        assert_eq!(folded.formula_branch_endpoints().len(), 4);
        assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
        assert!(folded.seam_points().is_some());
        assert!(folded.source_seam_parameters().is_some());
        assert!(
            folded
                .guarded_ranges()
                .iter()
                .all(|range| range.width() > 0.0)
        );
    }

    #[test]
    fn bounded_seam_folded_support_is_exact_rigid_frame_stable() {
        let rotated = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 0.0),
        )
        .unwrap();
        for frame in [Frame::world(), rotated] {
            let direct = bounded_seam_cylinders(frame, 3.0_f64.next_down());
            for cylinders in [direct, [direct[1], direct[0]]] {
                let contact = match classify_skew_cylinder_exact_discriminant(
                    cylinders,
                    SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
                )
                .unwrap()
                {
                    SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
                    other => panic!("expected rooted contact topology, got {other:?}"),
                };
                let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
                let location = topology.positive_cell();
                let windows = if cylinders == direct {
                    bounded_windows()
                } else {
                    let windows = bounded_windows();
                    [windows[1], windows[0]]
                };
                let roots = topology.roots();
                let root_evidence: [SupportRootEvidence; 2] = core::array::from_fn(|ordinal| {
                    let root = roots[ordinal];
                    root_evidence(topology.topology(), root, windows, false, true)
                        .unwrap_or_else(|error| {
                        panic!(
                            "reversed={} location={location:?} root {ordinal} {:?} evidence: {error:?}",
                            cylinders != direct,
                            root.angular_bracket(),
                        )
                        })
                });
                assert!(root_evidence.iter().all(|evidence| {
                    evidence
                        .exact_heights
                        .into_iter()
                        .zip(windows)
                        .all(|(height, window)| strictly_inside(height, window[1]))
                }));
                let folded = certify_persistent_skew_cylinder_folded_support(
                    topology,
                    windows,
                    [0, 1],
                    1.0e-7,
                    SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
                )
                .unwrap_or_else(|error| {
                    panic!(
                        "reversed={} location={location:?}: {error:?}",
                        cylinders != direct
                    )
                });
                assert_eq!(
                    folded.formula_residuals().len(),
                    match location {
                        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => 2,
                        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => 4,
                    }
                );
            }
        }
    }

    #[test]
    fn repeated_positive_support_touch_mints_six_cross_sheet_members_atomically() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            touching_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected rooted contact topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
        assert!(topology.root().repeated());
        assert!(
            certify_persistent_skew_cylinder_touching_support(
                topology.clone(),
                touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let touching = certify_persistent_skew_cylinder_touching_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(touching.work(), SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK);
        assert_eq!(touching.formula_residuals().len(), 6);
        assert_eq!(touching.formula_branch_endpoints().len(), 6);
        assert!(
            touching
                .chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::First)
                .is_some()
        );
        assert_eq!(
            touching
                .chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::Second),
            None
        );
        let mut root_ports = touching
            .formula_branch_endpoints()
            .iter()
            .flatten()
            .filter_map(|endpoint| match endpoint {
                PersistentSkewCylinderTouchingSupportEndpoint::Root { continuation, .. } => {
                    Some(*continuation)
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        root_ports.sort_unstable();
        assert_eq!(root_ports, vec![0, 0, 1, 1]);
        let first_root = PersistentSkewCylinderTouchingSupportEndpoint::Root {
            root: PersistentSkewCylinderTouchingSupportRoot::First,
            continuation: 0,
        };
        let unused_second_root = PersistentSkewCylinderTouchingSupportEndpoint::Root {
            root: PersistentSkewCylinderTouchingSupportRoot::Second,
            continuation: 0,
        };
        assert_eq!(
            touching.endpoint_point(unused_second_root),
            touching.endpoint_point(first_root)
        );
        assert_eq!(
            touching.source_parameters(unused_second_root),
            touching.source_parameters(first_root)
        );
        assert!(touching.required_edge_tolerance() <= touching.tolerance());
    }

    #[test]
    fn repeated_positive_seam_touch_uses_two_chart_joins_per_sheet() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            seam_touching_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected seam-root contact topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
        assert_eq!(
            topology.root().angular_bracket(),
            crate::SkewCylinderAngularRootBracket { lo: 0.0, hi: 0.0 }
        );
        let touching = certify_persistent_skew_cylinder_touching_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(touching.formula_residuals().len(), 6);
        assert_eq!(
            touching.chart_join_longitudes(),
            &[
                core::f64::consts::FRAC_PI_2,
                3.0 * core::f64::consts::FRAC_PI_2,
            ]
        );
        assert_eq!(
            touching
                .chart_join_longitude_for(PersistentSkewCylinderTouchingSupportChartJoin::Second),
            Some(3.0 * core::f64::consts::FRAC_PI_2)
        );
        assert!(
            touching
                .formula_branch_endpoints()
                .iter()
                .flatten()
                .all(|endpoint| !matches!(
                    endpoint,
                    PersistentSkewCylinderTouchingSupportEndpoint::Seam(_)
                ))
        );
        assert!(touching.required_edge_tolerance() <= touching.tolerance());
    }

    #[test]
    fn repeated_positive_opposite_pole_touch_uses_two_chart_joins_per_sheet() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            bounded_opposite_pole_touching_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected opposite-pole contact topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
        assert_eq!(
            topology.root().angular_bracket(),
            crate::SkewCylinderAngularRootBracket {
                lo: core::f64::consts::PI,
                hi: core::f64::consts::PI,
            }
        );
        assert!(
            certify_persistent_skew_cylinder_touching_support(
                topology.clone(),
                bounded_touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let touching = certify_persistent_skew_cylinder_touching_support(
            topology,
            bounded_touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(
            touching.work(),
            SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK
        );
        assert_eq!(touching.formula_residuals().len(), 8);
        assert_eq!(touching.formula_branch_endpoints().len(), 8);
        assert_eq!(
            touching.chart_join_longitudes(),
            &[
                core::f64::consts::FRAC_PI_2,
                3.0 * core::f64::consts::FRAC_PI_2,
            ]
        );
        let seam_endpoints = touching
            .formula_branch_endpoints()
            .iter()
            .flatten()
            .filter(|endpoint| {
                matches!(
                    endpoint,
                    PersistentSkewCylinderTouchingSupportEndpoint::Seam(_)
                )
            })
            .count();
        assert_eq!(seam_endpoints, 4);
        assert!(touching.required_edge_tolerance() <= touching.tolerance());
    }

    #[test]
    fn two_repeated_positive_roots_mint_two_closed_crossing_curves() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            double_touching_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected double-touching contact topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_touching_support_topology(contact).unwrap();
        assert_eq!(topology.roots().len(), 2);
        assert!(topology.roots().iter().all(|root| root.repeated()));
        let touching = certify_persistent_skew_cylinder_touching_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(touching.formula_residuals().len(), 6);
        assert_eq!(touching.formula_root_longitudes().len(), 2);
        assert!(touching.chart_join_longitudes().is_empty());
        let mut root_ports = touching
            .formula_branch_endpoints()
            .iter()
            .flatten()
            .filter_map(|endpoint| match endpoint {
                PersistentSkewCylinderTouchingSupportEndpoint::Root { root, continuation } => {
                    Some((root.ordinal(), *continuation))
                }
                _ => None,
            })
            .collect::<Vec<_>>();
        root_ports.sort_unstable();
        assert_eq!(
            root_ports,
            vec![
                (0, 0),
                (0, 0),
                (0, 1),
                (0, 1),
                (1, 0),
                (1, 0),
                (1, 1),
                (1, 1),
            ]
        );
        assert!(touching.required_edge_tolerance() <= touching.tolerance());
    }

    #[test]
    fn simple_root_on_authored_seam_mints_four_chart_split_members() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            seam_root_folded_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected seam-root folded topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        assert_eq!(
            persistent_skew_cylinder_folded_support_exact_work(&topology),
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(folded.formula_residuals().len(), 4);
        assert_eq!(
            folded.formula_branch_endpoints(),
            [
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Lower,
                    ),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Lower,
                    ),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Upper,
                    ),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Upper,
                    ),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                ],
            ]
        );
        assert_eq!(
            folded.chart_join_longitude(),
            Some(core::f64::consts::FRAC_PI_2)
        );
        assert!(folded.required_edge_tolerance() <= folded.tolerance());
    }

    #[test]
    fn seam_root_across_positive_cell_mints_four_chart_split_members() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            seam_root_across_folded_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected across-seam pole-pair topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        assert_eq!(
            topology.positive_cell(),
            SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        );
        assert_eq!(
            persistent_skew_cylinder_folded_support_exact_work(&topology),
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.work(), SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(folded.formula_residuals().len(), 4);
        assert_eq!(
            folded.formula_branch_endpoints(),
            [
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Lower,
                    ),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Lower,
                    ),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Upper,
                    ),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::ChartJoin(
                        SkewCylinderSheet::Upper,
                    ),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                ],
            ]
        );
        assert_eq!(
            folded.chart_join_longitude(),
            Some(3.0 * core::f64::consts::FRAC_PI_2)
        );
        assert!(folded.seam_points().is_none());
        assert!(folded.required_edge_tolerance() <= folded.tolerance());
    }

    #[test]
    fn short_positive_cell_from_simple_seam_root_mints_two_members() {
        let contact = match classify_skew_cylinder_exact_discriminant(
            short_seam_root_folded_support_cylinders(Frame::world()),
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected short seam-root folded topology, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        let angular = topology
            .roots()
            .map(SkewCylinderDiscriminantRoot::angular_bracket);
        assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
        assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
        assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
        assert_eq!(
            topology.positive_cell(),
            SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
        );
        assert_eq!(
            persistent_skew_cylinder_folded_support_exact_work(&topology),
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(folded.formula_residuals().len(), 2);
        assert_eq!(
            folded.formula_branch_endpoints(),
            [
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                ],
            ]
        );
        assert_eq!(folded.chart_join_longitude(), None);
        assert!(folded.seam_points().is_none());
        assert!(folded.required_edge_tolerance() <= folded.tolerance());
    }

    #[test]
    fn perpendicular_semantic_discriminant_preserves_bounded_short_seam_root() {
        let cylinders = bounded_short_seam_root_folded_support_cylinders(Frame::world());
        let contact = match classify_skew_cylinder_exact_discriminant(
            cylinders,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected bounded short seam-root contact, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        let angular = topology
            .roots()
            .map(SkewCylinderDiscriminantRoot::angular_bracket);
        assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
        assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
        assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            bounded_touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.formula_residuals().len(), 2);
        assert!(
            folded
                .source_endpoint_parameters()
                .into_iter()
                .flatten()
                .all(|parameter| parameter[1] > 0.0 && parameter[1] < 1.0)
        );
    }

    #[test]
    fn short_across_seam_non_pole_layout_mints_two_members() {
        let cylinders = bounded_short_seam_root_across_folded_support_cylinders(Frame::world());
        let contact = match classify_skew_cylinder_exact_discriminant(
            cylinders,
            SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
        )
        .unwrap()
        {
            SkewCylinderExactDiscriminantTopology::Contact(topology) => *topology,
            other => panic!("expected bounded short across-seam contact, got {other:?}"),
        };
        let topology = certify_skew_cylinder_folded_support_topology(contact).unwrap();
        let angular = topology
            .roots()
            .map(SkewCylinderDiscriminantRoot::angular_bracket);
        assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
        assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
        assert!(angular[1].lo > core::f64::consts::PI && angular[1].hi < TAU);
        assert_eq!(
            topology.positive_cell(),
            SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        );
        assert_eq!(
            persistent_skew_cylinder_folded_support_exact_work(&topology),
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(
            certify_persistent_skew_cylinder_folded_support(
                topology.clone(),
                bounded_touching_support_windows(),
                [0, 1],
                1.0e-7,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
            )
            .is_err()
        );
        let folded = certify_persistent_skew_cylinder_folded_support(
            topology,
            bounded_touching_support_windows(),
            [0, 1],
            1.0e-7,
            SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        )
        .unwrap();
        assert_eq!(folded.work(), SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(folded.formula_residuals().len(), 2);
        assert_eq!(
            folded.formula_branch_endpoints(),
            [
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                ],
                [
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(1),
                    PersistentSkewCylinderFoldedSupportEndpoint::Root(0),
                ],
            ]
        );
        assert_eq!(folded.chart_join_longitude(), None);
        assert!(folded.seam_points().is_none());
        assert!(folded.required_edge_tolerance() <= folded.tolerance());
        assert!(
            folded
                .source_endpoint_parameters()
                .into_iter()
                .flatten()
                .all(|parameter| parameter[1] > 0.0 && parameter[1] < 1.0)
        );
    }
}
