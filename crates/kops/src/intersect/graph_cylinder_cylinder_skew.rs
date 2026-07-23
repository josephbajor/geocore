//! Exact discriminant admission for nonparallel Cylinder/Cylinder supports.
//!
//! A ruling of the first canonical cylinder is substituted into a
//! division-free dual chart of the second cylinder's stored frame. The
//! resulting exact quadratic in ruling height has an exact cyclic
//! second-harmonic discriminant. A strictly negative discriminant proves a
//! complete miss. A strictly positive discriminant proves the existence of two
//! infinite-support sheets. Full-cycle publication then requires paired
//! residual certificates for the procedural carrier and both pcurves. Contact
//! roots and failed exact classification remain typed indeterminate; no
//! sampled marcher is allowed to claim completion.

use kcore::error::CapabilityId;
use kcore::operation::{DiagnosticCode, DiagnosticKind, OperationScope, StageId};
use kcore::predicates::{Orientation, orient3d};
use kcore::proof::{IncompleteCause, IncompleteEvidence};
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;
use kgraph::{
    IntersectionCertificateError, PairedSkewCylinderBranchResidualCertificate,
    SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK, SkewCylinderSheet,
    certify_paired_skew_cylinder_branch_residuals,
};

use super::bounded_trigonometric::{
    CYCLIC_SECOND_HARMONIC_EXACT_WORK, CyclicSecondHarmonicFailure, ExactTrigScalar,
    SecondHarmonicCoefficients, StrictSign, classify_cyclic_second_harmonic,
};
use super::cylinder_cylinder::{compare_cylinder_windows, validate_ranges};
use super::error::IntersectionError;
use super::graph_surface::{GraphSurfaceIntersectionError, GraphSurfaceIntersectionResult};
use super::result::{
    ContactKind, SurfaceIntersectionCurve, SurfaceSurfaceCurve, SurfaceSurfaceIntersections,
};

const TWO_SHEET_REASON: &str = "strict-positive skew Cylinder/Cylinder discriminant requires a certified full-cycle contained two-sheet branch carrier";
const CONTACT_TOPOLOGY_REASON: &str =
    "skew Cylinder/Cylinder discriminant contact roots require certified branch topology";
const NUMERIC_RESOLUTION_REASON: &str =
    "exact skew Cylinder/Cylinder classification or branch proof did not finish";
const NONPARALLEL_REASON: &str =
    "skew Cylinder/Cylinder discriminant admission requires exact nonparallel axes";

/// Stable work stage for one exact full-cycle skew-cylinder discriminant proof.
pub const SKEW_CYLINDER_DISCRIMINANT_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-discriminant-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder discriminant stage"),
    };

/// Exact atomic work charged by one admitted skew-cylinder classification.
pub const SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK: u64 = 2 * CYCLIC_SECOND_HARMONIC_EXACT_WORK;

/// Stable work stage for one atomic pair of certified procedural branches.
pub const SKEW_CYLINDER_TWO_SHEET_WORK: StageId =
    match StageId::new("kops.intersect.skew-cylinder-two-sheet-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid skew-cylinder two-sheet stage"),
    };

/// Atomic work charged before certifying both procedural skew branches.
pub const SKEW_CYLINDER_TWO_SHEET_EXACT_WORK: u64 = 2 * SKEW_CYLINDER_BRANCH_CERTIFICATE_WORK;

/// Missing carrier for the two sheets proved by a strict-positive discriminant.
pub const SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER: CapabilityId =
    match CapabilityId::new("kops.intersect.skew-cylinder-two-sheet-branch-carrier") {
        Ok(capability) => capability,
        Err(_) => panic!("valid skew-cylinder two-sheet capability"),
    };

/// Missing topology for zeroes of the exact cyclic discriminant.
pub const SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY: CapabilityId =
    match CapabilityId::new("kops.intersect.skew-cylinder-contact-root-topology") {
        Ok(capability) => capability,
        Err(_) => panic!("valid skew-cylinder contact-root capability"),
    };

/// Strict-positive discriminant was proved, but its branch carrier is pending.
pub const SKEW_CYLINDER_TWO_SHEET_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-two-sheet-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder two-sheet diagnostic"),
    };

/// The discriminant has contact/root topology outside this initial admission.
pub const SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-contact-topology-incomplete") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder contact-topology diagnostic"),
    };

/// Exact construction or cyclic classification failed inside its safe envelope.
pub const SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION: DiagnosticCode =
    match DiagnosticCode::new("kops.intersect.skew-cylinder-discriminant-numeric-resolution") {
        Ok(code) => code,
        Err(_) => panic!("valid skew-cylinder numeric-resolution diagnostic"),
    };

/// Non-forgeable proof that an exact nonparallel Cylinder/Cylinder pair has a
/// strictly negative ruling discriminant over the complete canonical cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SkewCylinderStrictDiscriminantMiss {
    _private: (),
}

/// Complete graph inputs produced by the exact skew-cylinder admission.
pub(super) struct CertifiedSkewCylinderIntersections {
    pub(super) raw: SurfaceSurfaceIntersections,
    pub(super) strict_miss: Option<SkewCylinderStrictDiscriminantMiss>,
    pub(super) two_sheet_certificates: Option<[PairedSkewCylinderBranchResidualCertificate; 2]>,
}

impl SkewCylinderStrictDiscriminantMiss {
    const fn certified() -> Self {
        Self { _private: () }
    }
}

#[derive(Debug, Clone)]
struct FirstHarmonic {
    constant: ExactTrigScalar,
    cosine: ExactTrigScalar,
    sine: ExactTrigScalar,
}

#[derive(Debug, Clone)]
struct SecondHarmonic {
    constant: ExactTrigScalar,
    cosine: ExactTrigScalar,
    sine: ExactTrigScalar,
    cosine2: ExactTrigScalar,
    sine2: ExactTrigScalar,
}

/// Classify one validated exact-nonparallel pair from a canonical source order.
pub(super) fn intersect_certified_skew_cylinders(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    validate_ranges(ranges[0], ranges[1])
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    let (cylinders, ranges, reversed) = canonical_pair(cylinders, ranges);
    if !axes_are_exactly_nonparallel(cylinders) {
        return Err(GraphSurfaceIntersectionError::BranchCertificate(
            kgraph::IntersectionCertificateError::UnsupportedCarrierParameterization {
                reason: NONPARALLEL_REASON,
            },
        ));
    }

    // The two deterministic parameterization attempts form one atomic proof
    // unit. A failed charge records the attempted N/N-1 crossing without
    // partially consuming this stage.
    scope.ledger_mut().charge(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    )?;

    let mut admission = classify_one_parameterization(cylinders);
    let mut parameterization_reversed = false;
    if admission == DiscriminantAdmission::NumericResolution {
        admission = classify_one_parameterization([cylinders[1], cylinders[0]]);
        parameterization_reversed = true;
    }

    match admission {
        DiscriminantAdmission::Strict(StrictSign::Negative) => {
            Ok(CertifiedSkewCylinderIntersections {
                raw: SurfaceSurfaceIntersections::complete_empty(),
                strict_miss: Some(SkewCylinderStrictDiscriminantMiss::certified()),
                two_sheet_certificates: None,
            })
        }
        DiscriminantAdmission::Strict(StrictSign::Positive) => {
            let (proof_cylinders, proof_ranges) = if parameterization_reversed {
                ([cylinders[1], cylinders[0]], [ranges[1], ranges[0]])
            } else {
                (cylinders, ranges)
            };
            intersect_strict_positive_two_sheet(
                proof_cylinders,
                proof_ranges,
                reversed ^ parameterization_reversed,
                tolerance,
                scope,
            )
        }
        DiscriminantAdmission::Contact => Ok(CertifiedSkewCylinderIntersections {
            raw: contact_topology_incomplete(scope),
            strict_miss: None,
            two_sheet_certificates: None,
        }),
        DiscriminantAdmission::NumericResolution => Ok(CertifiedSkewCylinderIntersections {
            raw: numeric_resolution(scope, SKEW_CYLINDER_DISCRIMINANT_WORK),
            strict_miss: None,
            two_sheet_certificates: None,
        }),
    }
}

fn intersect_strict_positive_two_sheet(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
    reversed: bool,
    tolerance: f64,
    scope: &mut OperationScope<'_, '_>,
) -> GraphSurfaceIntersectionResult<CertifiedSkewCylinderIntersections> {
    scope.ledger_mut().charge(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK,
    )?;
    let certified = [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper].map(|sheet| {
        certify_paired_skew_cylinder_branch_residuals(cylinders, ranges, sheet, tolerance)
    });
    let certificates = match certified {
        [Ok(lower), Ok(upper)] => [lower, upper],
        failures => {
            let unsupported = failures.iter().any(|result| {
                matches!(
                    result,
                    Err(
                        IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
                            | IntersectionCertificateError::InvalidCarrierRange
                    )
                )
            });
            return Ok(CertifiedSkewCylinderIntersections {
                raw: if unsupported {
                    two_sheet_incomplete(scope)
                } else {
                    numeric_resolution(scope, SKEW_CYLINDER_TWO_SHEET_WORK)
                },
                strict_miss: None,
                two_sheet_certificates: None,
            });
        }
    };
    let certificates = if reversed {
        certificates.map(PairedSkewCylinderBranchResidualCertificate::swapped)
    } else {
        certificates
    };
    let curves = certificates
        .iter()
        .map(raw_two_sheet_curve)
        .collect::<Vec<_>>();
    let raw = SurfaceSurfaceIntersections::canonicalized_complete(Vec::new(), curves)
        .map_err(IntersectionError::from)
        .map_err(GraphSurfaceIntersectionError::Intersection)?;
    Ok(CertifiedSkewCylinderIntersections {
        raw,
        strict_miss: None,
        two_sheet_certificates: Some(certificates),
    })
}

fn raw_two_sheet_curve(
    certificate: &PairedSkewCylinderBranchResidualCertificate,
) -> SurfaceSurfaceCurve {
    let carrier = certificate.carrier();
    let range = certificate.carrier_range();
    let traces = certificate.traces();
    let endpoint = |trace: kgraph::SkewCylinderBranchTrace, parameter| {
        let uv = trace.pcurve().eval(parameter);
        [uv.x, uv.y]
    };
    SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::SkewCylinder(carrier),
        curve_range: range,
        uv_a_start: endpoint(traces[0], range.lo),
        uv_a_end: endpoint(traces[0], range.hi),
        uv_b_start: endpoint(traces[1], range.lo),
        uv_b_end: endpoint(traces[1], range.hi),
        kind: ContactKind::Transverse,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum DiscriminantAdmission {
    Strict(StrictSign),
    Contact,
    NumericResolution,
}

fn classify_one_parameterization(cylinders: [Cylinder; 2]) -> DiscriminantAdmission {
    let Some(classification) = exact_ruling_discriminant(cylinders) else {
        return DiscriminantAdmission::NumericResolution;
    };
    let (discriminant, cardinal_contact) = match classification {
        ExactRulingDiscriminant::Admission(admission) => return admission,
        ExactRulingDiscriminant::Harmonic {
            coefficients,
            cardinal_contact,
        } => (coefficients, cardinal_contact),
    };
    if cardinal_contact {
        return DiscriminantAdmission::Contact;
    }
    match classify_cyclic_second_harmonic(&discriminant, CYCLIC_SECOND_HARMONIC_EXACT_WORK) {
        Ok(topology) => topology.strict_full_cycle_sign().map_or(
            DiscriminantAdmission::Contact,
            DiscriminantAdmission::Strict,
        ),
        Err(
            CyclicSecondHarmonicFailure::ExactArithmetic(_)
            | CyclicSecondHarmonicFailure::InconsistentChartTopology
            | CyclicSecondHarmonicFailure::WorkLimit { .. },
        ) => DiscriminantAdmission::NumericResolution,
    }
}

enum ExactRulingDiscriminant {
    Admission(DiscriminantAdmission),
    Harmonic {
        coefficients: SecondHarmonicCoefficients,
        cardinal_contact: bool,
    },
}

fn canonical_pair(
    cylinders: [Cylinder; 2],
    ranges: [[ParamRange; 2]; 2],
) -> ([Cylinder; 2], [[ParamRange; 2]; 2], bool) {
    if compare_cylinder_windows(&cylinders[0], ranges[0], &cylinders[1], ranges[1]).is_gt() {
        ([cylinders[1], cylinders[0]], [ranges[1], ranges[0]], true)
    } else {
        (cylinders, ranges, false)
    }
}

fn axes_are_exactly_nonparallel(cylinders: [Cylinder; 2]) -> bool {
    let first = cylinders[0].frame().z().to_array();
    let second = cylinders[1].frame().z().to_array();
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .any(|axis| orient3d(first, second, axis, [0.0; 3]) != Orientation::Zero)
}

/// Form `A v² + B(u) v + C(u)` in a division-free dual chart of the second
/// cylinder, then return the exact second-harmonic coefficients of `B²-4AC`.
///
/// For the second stored frame `(x,y,z)`, let `E=det(x,y,z)`,
/// `X=det(p-O,y,z)`, and `Y=det(x,p-O,z)`. Its actual parametric descriptor
/// satisfies `X²+Y²-r²E²=0` without assuming the rounded stored frame is
/// exactly orthonormal.
fn exact_ruling_discriminant(cylinders: [Cylinder; 2]) -> Option<ExactRulingDiscriminant> {
    let first = cylinders[0];
    let second = cylinders[1];
    let second_axes = [
        second.frame().x().to_array(),
        second.frame().y().to_array(),
        second.frame().z().to_array(),
    ];
    let ruling = first.frame().z().to_array();
    let direction = [
        exact_determinant(ruling, second_axes[1], second_axes[2])?,
        exact_determinant(second_axes[0], ruling, second_axes[2])?,
    ];
    let a = direction[0]
        .mul(&direction[0])
        .ok()?
        .add(&direction[1].mul(&direction[1]).ok()?)
        .ok()?;
    if a.sign() <= 0 {
        return None;
    }

    let radial = [
        radial_coordinate_harmonic(first, second, DualCoordinate::First)?,
        radial_coordinate_harmonic(first, second, DualCoordinate::Second)?,
    ];
    let radius = ExactTrigScalar::from_f64(second.radius()).ok()?;
    let determinant = exact_determinant(second_axes[0], second_axes[1], second_axes[2])?;
    let radius_determinant = radius.mul(&determinant).ok()?;
    let radius_determinant_squared = radius_determinant.mul(&radius_determinant).ok()?;

    // B=2(dx X+dy Y) and C=X²+Y²-r²E². Applying the exact
    // two-dimensional Lagrange identity before expansion avoids manufacturing
    // large terms that must later cancel:
    //
    // B²-4AC = 4(A r²E² - (dx Y-dy X)²).
    let cross = cross_first_harmonic(&radial, &direction)?;
    let k = a.mul(&radius_determinant_squared).ok()?;
    if let Some(admission) = factor_extrema_admission(&k, &cross) {
        return Some(ExactRulingDiscriminant::Admission(admission));
    }
    let discriminant = SecondHarmonic::constant(k)
        .sub(&cross.square()?)?
        .scale(&ExactTrigScalar::from_f64(4.0).ok()?)?;
    let cardinal_contact = discriminant.cardinal_contact().unwrap_or(false);
    Some(ExactRulingDiscriminant::Harmonic {
        coefficients: SecondHarmonicCoefficients::new(
            discriminant.constant,
            discriminant.cosine,
            discriminant.sine,
            discriminant.cosine2,
            discriminant.sine2,
        ),
        cardinal_contact,
    })
}

/// Classify the exact extrema of `4(K-L(u)²)` without square roots.
///
/// For `L=c+p cos(u)+q sin(u)`, write `C²=c²`, `R²=p²+q²`, and
/// `P=4 C² R²`. The two squared comparisons below are exactly equivalent to
/// comparing `sqrt(K)` with `|c|±sqrt(R²)`. If neither strict inequality
/// holds, continuity proves that the discriminant has contact/root topology.
fn factor_extrema_admission(
    k: &ExactTrigScalar,
    harmonic: &FirstHarmonic,
) -> Option<DiscriminantAdmission> {
    if k.sign() <= 0 {
        return None;
    }
    let c2 = harmonic.constant.mul(&harmonic.constant).ok()?;
    let r2 = harmonic
        .cosine
        .mul(&harmonic.cosine)
        .ok()?
        .add(&harmonic.sine.mul(&harmonic.sine).ok()?)
        .ok()?;
    let p = c2.mul(&r2).ok()?.scale(4.0).ok()?;

    let t = k.sub(&c2).ok()?.sub(&r2).ok()?;
    if t.sign() > 0 && t.mul(&t).ok()?.sub(&p).ok()?.sign() > 0 {
        return Some(DiscriminantAdmission::Strict(StrictSign::Positive));
    }

    let u = c2.add(&r2).ok()?.sub(k).ok()?;
    if c2.sub(&r2).ok()?.sign() > 0 && u.sign() > 0 && u.mul(&u).ok()?.sub(&p).ok()?.sign() > 0 {
        return Some(DiscriminantAdmission::Strict(StrictSign::Negative));
    }
    Some(DiscriminantAdmission::Contact)
}

#[derive(Debug, Clone, Copy)]
enum DualCoordinate {
    First,
    Second,
}

fn radial_coordinate_harmonic(
    first: Cylinder,
    second: Cylinder,
    coordinate: DualCoordinate,
) -> Option<FirstHarmonic> {
    let radius = ExactTrigScalar::from_f64(first.radius()).ok()?;
    let first_axes = [first.frame().x().to_array(), first.frame().y().to_array()];
    let second_axes = [
        second.frame().x().to_array(),
        second.frame().y().to_array(),
        second.frame().z().to_array(),
    ];
    let offset = exact_vector_difference(
        first.frame().origin().to_array(),
        second.frame().origin().to_array(),
    )?;
    let determinant = |vector: [ExactTrigScalar; 3]| match coordinate {
        DualCoordinate::First => exact_determinant_expansion(
            vector,
            exact_vector(second_axes[1])?,
            exact_vector(second_axes[2])?,
        ),
        DualCoordinate::Second => exact_determinant_expansion(
            exact_vector(second_axes[0])?,
            vector,
            exact_vector(second_axes[2])?,
        ),
    };
    Some(FirstHarmonic {
        constant: determinant(offset)?,
        cosine: determinant(exact_vector(first_axes[0])?)
            .and_then(|value| value.mul(&radius).ok())?,
        sine: determinant(exact_vector(first_axes[1])?)
            .and_then(|value| value.mul(&radius).ok())?,
    })
}

fn cross_first_harmonic(
    values: &[FirstHarmonic; 2],
    weights: &[ExactTrigScalar; 2],
) -> Option<FirstHarmonic> {
    let component = |select: fn(&FirstHarmonic) -> &ExactTrigScalar| {
        weights[0]
            .mul(select(&values[1]))
            .ok()?
            .sub(&weights[1].mul(select(&values[0])).ok()?)
            .ok()
    };
    Some(FirstHarmonic {
        constant: component(|value| &value.constant)?,
        cosine: component(|value| &value.cosine)?,
        sine: component(|value| &value.sine)?,
    })
}

impl FirstHarmonic {
    fn square(&self) -> Option<SecondHarmonic> {
        let cosine_squared = self.cosine.mul(&self.cosine).ok()?;
        let sine_squared = self.sine.mul(&self.sine).ok()?;
        let first_square_sum = cosine_squared.add(&sine_squared).ok()?;
        let first_square_difference = cosine_squared.sub(&sine_squared).ok()?;
        Some(SecondHarmonic {
            constant: self
                .constant
                .mul(&self.constant)
                .ok()?
                .add(&first_square_sum.scale(0.5).ok()?)
                .ok()?,
            cosine: self.constant.mul(&self.cosine).ok()?.scale(2.0).ok()?,
            sine: self.constant.mul(&self.sine).ok()?.scale(2.0).ok()?,
            cosine2: first_square_difference.scale(0.5).ok()?,
            sine2: self.cosine.mul(&self.sine).ok()?,
        })
    }
}

impl SecondHarmonic {
    fn constant(value: ExactTrigScalar) -> Self {
        Self {
            constant: value,
            cosine: ExactTrigScalar::zero(),
            sine: ExactTrigScalar::zero(),
            cosine2: ExactTrigScalar::zero(),
            sine2: ExactTrigScalar::zero(),
        }
    }

    fn sub(&self, rhs: &Self) -> Option<Self> {
        Some(Self {
            constant: self.constant.sub(&rhs.constant).ok()?,
            cosine: self.cosine.sub(&rhs.cosine).ok()?,
            sine: self.sine.sub(&rhs.sine).ok()?,
            cosine2: self.cosine2.sub(&rhs.cosine2).ok()?,
            sine2: self.sine2.sub(&rhs.sine2).ok()?,
        })
    }

    fn scale(&self, factor: &ExactTrigScalar) -> Option<Self> {
        Some(Self {
            constant: self.constant.mul(factor).ok()?,
            cosine: self.cosine.mul(factor).ok()?,
            sine: self.sine.mul(factor).ok()?,
            cosine2: self.cosine2.mul(factor).ok()?,
            sine2: self.sine2.mul(factor).ok()?,
        })
    }

    /// A cardinal zero or opposite exact cardinal signs proves that the cycle
    /// contains contact/root topology. Same-sign values make no claim and
    /// continue to the complete cyclic authority.
    fn cardinal_contact(&self) -> Option<bool> {
        let values = [
            self.constant
                .add(&self.cosine)
                .ok()?
                .add(&self.cosine2)
                .ok()?,
            self.constant
                .add(&self.sine)
                .ok()?
                .sub(&self.cosine2)
                .ok()?,
            self.constant
                .sub(&self.cosine)
                .ok()?
                .add(&self.cosine2)
                .ok()?,
            self.constant
                .sub(&self.sine)
                .ok()?
                .sub(&self.cosine2)
                .ok()?,
        ];
        let signs = values.each_ref().map(ExactTrigScalar::sign);
        Some(signs.contains(&0) || (signs.contains(&-1) && signs.contains(&1)))
    }
}

fn exact_vector(vector: [f64; 3]) -> Option<[ExactTrigScalar; 3]> {
    Some([
        ExactTrigScalar::from_f64(vector[0]).ok()?,
        ExactTrigScalar::from_f64(vector[1]).ok()?,
        ExactTrigScalar::from_f64(vector[2]).ok()?,
    ])
}

fn exact_vector_difference(point: [f64; 3], origin: [f64; 3]) -> Option<[ExactTrigScalar; 3]> {
    let mut difference = [
        ExactTrigScalar::zero(),
        ExactTrigScalar::zero(),
        ExactTrigScalar::zero(),
    ];
    for coordinate in 0..3 {
        let point = ExactTrigScalar::from_f64(point[coordinate]).ok()?;
        let origin = ExactTrigScalar::from_f64(origin[coordinate]).ok()?;
        difference[coordinate] = point.sub(&origin).ok()?;
    }
    Some(difference)
}

fn exact_determinant(
    first: [f64; 3],
    second: [f64; 3],
    third: [f64; 3],
) -> Option<ExactTrigScalar> {
    exact_determinant_expansion(
        exact_vector(first)?,
        exact_vector(second)?,
        exact_vector(third)?,
    )
}

fn exact_determinant_expansion(
    first: [ExactTrigScalar; 3],
    second: [ExactTrigScalar; 3],
    third: [ExactTrigScalar; 3],
) -> Option<ExactTrigScalar> {
    let minor = |a: usize, b: usize, c: usize, d: usize| {
        second[a]
            .mul(&third[b])
            .ok()?
            .sub(&second[c].mul(&third[d]).ok()?)
            .ok()
    };
    first[0]
        .mul(&minor(1, 2, 2, 1)?)
        .ok()?
        .sub(&first[1].mul(&minor(0, 2, 2, 0)?).ok()?)
        .ok()?
        .add(&first[2].mul(&minor(0, 1, 1, 0)?).ok()?)
        .ok()
}

fn two_sheet_incomplete(scope: &mut OperationScope<'_, '_>) -> SurfaceSurfaceIntersections {
    scope.diagnose(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
        DiagnosticKind::ProofIncomplete,
        TWO_SHEET_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        TWO_SHEET_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
            stage: SKEW_CYLINDER_TWO_SHEET_WORK,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
            },
            message: TWO_SHEET_REASON,
        }],
    )
}

fn contact_topology_incomplete(scope: &mut OperationScope<'_, '_>) -> SurfaceSurfaceIntersections {
    scope.diagnose(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
        DiagnosticKind::ProofIncomplete,
        CONTACT_TOPOLOGY_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        CONTACT_TOPOLOGY_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_CONTACT_TOPOLOGY_INCOMPLETE,
            stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
            cause: IncompleteCause::ProofMethodUnavailable {
                capability: SKEW_CYLINDER_CONTACT_ROOT_TOPOLOGY,
            },
            message: CONTACT_TOPOLOGY_REASON,
        }],
    )
}

fn numeric_resolution(
    scope: &mut OperationScope<'_, '_>,
    stage: StageId,
) -> SurfaceSurfaceIntersections {
    scope.record_numeric_resolution(stage);
    scope.diagnose(
        stage,
        SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION,
        DiagnosticKind::NumericResolution,
        NUMERIC_RESOLUTION_REASON,
    );
    SurfaceSurfaceIntersections::indeterminate_empty_with_evidence(
        NUMERIC_RESOLUTION_REASON,
        vec![IncompleteEvidence {
            code: SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION,
            stage,
            cause: IncompleteCause::NumericResolution,
            message: NUMERIC_RESOLUTION_REASON,
        }],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    #[test]
    fn reversed_parameterization_recovers_from_one_sided_exact_envelope_refusal() {
        let first = Cylinder::new(Frame::world(), 2.0).unwrap();
        let second = Cylinder::new(
            Frame::new(
                Point3::new(0.0, 8.0, 0.0),
                Vec3::new(1.0, 1.0, 2.0_f64.powi(-500)),
                Vec3::new(1.0, -1.0, 0.0),
            )
            .unwrap(),
            1.0,
        )
        .unwrap();

        assert_eq!(
            classify_one_parameterization([first, second]),
            DiscriminantAdmission::NumericResolution
        );
        assert_eq!(
            classify_one_parameterization([second, first]),
            DiscriminantAdmission::Strict(StrictSign::Negative)
        );
    }
}
