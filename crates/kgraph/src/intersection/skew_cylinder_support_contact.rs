//! Persistent exact-root carrier for one isolated skew-cylinder support
//! tangency.
//!
//! The existing cyclic second-harmonic theorem proves that exactly one
//! repeated discriminant root exists and that every complementary longitude
//! is a strict miss. This module binds that exact projective root to a finite
//! pair of full-period cylinder windows. Outward interval evaluation proves
//! the common support point is inside both axial windows or exactly incident
//! to an authored axial bound; rounded analytic evaluation supplies only its
//! deterministic representative.

use kcore::interval::Interval;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::Vec3;

use super::{
    BranchAlgebra, SkewCylinderDiscriminantContactTopologyCertificate,
    SkewCylinderDiscriminantRoot, SkewCylinderHalfAngleChart, SkewCylinderSheet, build_algebra,
    coefficient_proof, finite_interval, longitude_interval,
};
use super::{
    SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK, SkewCylinderAxialBoundProvenance,
    SkewCylinderAxialBoundary,
};
use crate::IntersectionCertificateError;

const TAU: f64 = core::f64::consts::TAU;
const UNSUPPORTED_REASON: &str = "skew Cylinder/Cylinder support contact is not one isolated repeated root inside or exactly on the boundary of two full-period finite windows";

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
    let angular = root.angular_bracket();
    if !angular.lo.is_finite() || !angular.hi.is_finite() || angular.lo > angular.hi {
        return Err(unsupported());
    }
    let canonical_parameter = if angular.lo == angular.hi {
        angular.lo
    } else {
        angular.lo / 2.0 + angular.hi / 2.0
    };
    let carrier_parameter = fit_full_period_parameter(canonical_parameter, formula_windows[0][0])
        .ok_or_else(unsupported)?;
    let algebra = build_algebra(
        topology.formula_cylinders(),
        formula_windows[0][0],
        SkewCylinderSheet::Lower,
    )
    .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    let proof = coefficient_proof(algebra).ok_or_else(unsupported)?;
    let [cosine, sine] = projective_root_trig_intervals(root)?;
    let exact_m = proof
        .m_true
        .interval(cosine, sine)
        .ok_or_else(unsupported)?;
    let exact_v = finite_interval(
        (Interval::point(-1.0) * exact_m)
            .checked_div(proof.a_true)
            .ok_or_else(unsupported)?,
    )
    .ok_or_else(unsupported)?;
    let exact_coordinates = [0, 1, 2].map(|coordinate| {
        proof.harmonics_true[coordinate]
            .interval(cosine, sine)
            .and_then(|value| finite_interval(value + exact_v * proof.directions_true[coordinate]))
            .ok_or_else(unsupported)
    });
    let [exact_x, exact_y, exact_z] = exact_coordinates;
    let [exact_x, exact_y, exact_z] = [exact_x?, exact_y?, exact_z?];
    let normalized_x = exact_x
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    let normalized_y = exact_y
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    if normalized_x.contains_zero() && normalized_y.contains_zero() {
        return Err(unsupported());
    }
    let exact_second_height = exact_z
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    let first_longitude = lift_interval_near(
        Interval::new(angular.lo, angular.hi),
        carrier_parameter,
        formula_windows[0][0],
    )
    .ok_or_else(unsupported)?;
    let second_representative = algebra.authored_pcurve_derivs(1, carrier_parameter, 0).d[0].x;
    let second_longitude = lift_interval_near(
        longitude_interval(normalized_x, normalized_y),
        second_representative,
        formula_windows[1][0],
    )
    .ok_or_else(unsupported)?;
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
    IntersectionCertificateError::UnsupportedCarrierParameterization {
        reason: UNSUPPORTED_REASON,
    }
}

#[cfg(test)]
mod tests {
    use kgeom::frame::Frame;
    use kgeom::vec::Point3;

    use super::*;
    use crate::{
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK, SkewCylinderExactDiscriminantTopology,
        classify_skew_cylinder_exact_discriminant,
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
        assert!(
            certify_persistent_skew_cylinder_support_contact(rooted, windows(), [0, 1], 1.0e-9, 0,)
                .is_err()
        );
    }
}
