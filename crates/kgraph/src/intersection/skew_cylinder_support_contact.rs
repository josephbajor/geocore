//! Persistent exact-root carrier for one isolated skew-cylinder support
//! tangency.
//!
//! The existing cyclic second-harmonic theorem proves that exactly one
//! repeated discriminant root exists and that every complementary longitude
//! is a strict miss. This module binds that exact projective root to a finite
//! pair of full-period cylinder windows. Outward interval evaluation proves
//! the common support point is strictly inside both axial windows; rounded
//! analytic evaluation supplies only its deterministic representative.

use kcore::interval::Interval;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::Vec3;

use super::{
    BranchAlgebra, SkewCylinderDiscriminantContactTopologyCertificate,
    SkewCylinderDiscriminantRoot, SkewCylinderSheet, build_algebra, coefficient_proof,
    finite_interval, trig_interval,
};
use crate::IntersectionCertificateError;

const TAU: f64 = core::f64::consts::TAU;
const UNSUPPORTED_REASON: &str = "skew Cylinder/Cylinder support contact is not one isolated repeated root strictly inside two full-period finite windows";

/// Persistent exact-root-owned point carrier for one isolated
/// infinite-support skew-cylinder tangency inside two finite source windows.
#[derive(Debug, Clone, PartialEq)]
pub struct PersistentSkewCylinderSupportContactCertificate {
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
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

/// Certify one exact isolated support tangency inside two full-period finite
/// cylinder windows.
pub fn certify_persistent_skew_cylinder_support_contact(
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
    formula_windows: [[ParamRange; 2]; 2],
    formula_to_source: [usize; 2],
    tolerance: f64,
) -> Result<PersistentSkewCylinderSupportContactCertificate, IntersectionCertificateError> {
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
    let mut algebra = build_algebra(
        topology.formula_cylinders(),
        formula_windows[0][0],
        SkewCylinderSheet::Lower,
    )
    .ok_or(IntersectionCertificateError::InvalidTraceFamily)?;
    let proof = coefficient_proof(algebra).ok_or_else(unsupported)?;
    let cosine = trig_interval(angular.lo, angular.hi, false);
    let sine = trig_interval(angular.lo, angular.hi, true);
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
    let exact_z = proof.harmonics_true[2]
        .interval(cosine, sine)
        .and_then(|z| finite_interval(z + exact_v * proof.directions_true[2]))
        .ok_or_else(unsupported)?;
    let exact_second_height = exact_z
        .checked_div(proof.e_true)
        .and_then(finite_interval)
        .ok_or_else(unsupported)?;
    if !strictly_inside(exact_v, formula_windows[0][1])
        || !strictly_inside(exact_second_height, formula_windows[1][1])
    {
        return Err(unsupported());
    }

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
        carrier_parameter,
        tolerance,
    })
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
            certify_persistent_skew_cylinder_support_contact(exact, windows(), [0, 1], 1.0e-9)
                .unwrap();
        assert!(certified.point().dist(Point3::new(0.0, 1.0, 0.0)) <= 1.0e-12);
        assert_eq!(certified.source_surface_parameters()[0][1], 0.0);

        let mut boundary_windows = windows();
        boundary_windows[0][1] = ParamRange::new(0.0, 1.0);
        assert!(
            certify_persistent_skew_cylinder_support_contact(
                certified.topology().clone(),
                boundary_windows,
                [0, 1],
                1.0e-9,
            )
            .is_err(),
            "a support root on an authored axial boundary remains refused"
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
            certify_persistent_skew_cylinder_support_contact(rooted, windows(), [0, 1], 1.0e-9,)
                .is_err()
        );
    }
}
