//! Exact cyclic cut topology for finite skew-cylinder axial bounds.
//!
//! The canonical carrier parameter is longitude `u` on the first cylinder.
//! In the second cylinder's division-free dual frame, the two strictly
//! positive sheets are
//!
//! `v_s = (-M(u) + s sqrt(K - L(u)^2)) / A`, `s in {-1, +1}`.
//!
//! Substituting one axial bound and eliminating the square root produces one
//! exact cyclic second harmonic whose roots are the union of both sheets'
//! crossings. An unsquared first-harmonic selector assigns each root to its
//! sheet and, together with the eliminated sign, proves both sheets' strict
//! Above/Below relation on every open cyclic cell. No sampled evaluation is
//! used for a topological decision.

use super::SkewCylinderSheet;
use kcore::math;
use kgeom::param::ParamRange;
use kgeom::surface::Cylinder;

use crate::exact::bounded_polynomial::{
    ExactPolynomial, ExactScalar, RootBracket, RootIsolation, RootIsolationFailure,
};
use crate::exact::bounded_root_relation::{ExactRootRelation, classify_exact_root_relation};
use crate::exact::bounded_trigonometric::{
    CYCLIC_SECOND_HARMONIC_EXACT_WORK, CyclicRoot, CyclicRootBracket, CyclicSecondHarmonicFailure,
    CyclicSecondHarmonicTopology, HalfAngleChart, SecondHarmonicCoefficients, StrictSign,
    classify_cyclic_second_harmonic,
};

const TAU: f64 = core::f64::consts::TAU;

/// Fixed reservation for one complete axial-bound topology query.
///
/// The caller that combines all four finite-window bounds owns the atomic
/// `4 * 64` operation-ledger charge. This pure query checks its complete
/// allowance before constructing exact coefficients and never returns a
/// partial root list.
pub const SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK: u64 = CYCLIC_SECOND_HARMONIC_EXACT_WORK;

/// Caller-authored side of one finite axial window.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderAxialBoundary {
    /// Low end of the caller-authored axial interval.
    Lower,
    /// High end of the caller-authored axial interval.
    Upper,
}

/// Caller-order provenance retained by every source root.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderAxialBoundProvenance {
    /// Caller/source operand whose axial chart supplies the bound.
    pub source_operand: usize,
    /// Authored side of that operand's axial interval.
    pub boundary: SkewCylinderAxialBoundary,
    /// Exact caller-authored axial coordinate.
    pub value: f64,
}

/// Strict relation of one sheet height to the queried bound on an open cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderAxialRelation {
    /// The sheet height is strictly less than the bound.
    Below,
    /// The sheet height is strictly greater than the bound.
    Above,
}

/// Half-angle chart retaining the exact source-root identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderHalfAngleChart {
    /// Tangent half-angle coordinate `t = tan(u / 2)`.
    Tangent,
    /// Cotangent half-angle coordinate `q = cot(u / 2)`.
    Cotangent,
}

impl From<HalfAngleChart> for SkewCylinderHalfAngleChart {
    fn from(chart: HalfAngleChart) -> Self {
        match chart {
            HalfAngleChart::Tangent => Self::Tangent,
            HalfAngleChart::Cotangent => Self::Cotangent,
        }
    }
}

/// One source root bracket in its owning projective half-angle chart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderHalfAngleRootBracket {
    /// Projective chart that owns the bracket without ambiguity.
    pub chart: SkewCylinderHalfAngleChart,
    /// Inclusive lower projective coordinate.
    pub lo: f64,
    /// Inclusive upper projective coordinate.
    pub hi: f64,
}

pub(crate) fn tangent_projective_interval(
    bracket: SkewCylinderHalfAngleRootBracket,
) -> Option<kcore::interval::Interval> {
    let interval = kcore::interval::Interval::new(bracket.lo, bracket.hi);
    match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent => Some(interval),
        SkewCylinderHalfAngleChart::Cotangent => {
            kcore::interval::Interval::point(1.0).checked_div(interval)
        }
    }
}

/// Numeric enclosure of one source root, ordered by increasing canonical
/// longitude in `[0, 2π)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderAngularRootBracket {
    /// Inclusive lower canonical-longitude coordinate.
    pub lo: f64,
    /// Inclusive upper canonical-longitude coordinate.
    pub hi: f64,
}

/// One exact projective root of the infinite-support skew-cylinder
/// discriminant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderDiscriminantRoot {
    bracket: SkewCylinderHalfAngleRootBracket,
    repeated: bool,
}

impl SkewCylinderDiscriminantRoot {
    /// Exact-source isolating bracket in its owning projective chart.
    pub const fn bracket(self) -> SkewCylinderHalfAngleRootBracket {
        self.bracket
    }

    /// Whether the exact discriminant root has even multiplicity.
    pub const fn repeated(self) -> bool {
        self.repeated
    }

    /// Increasing canonical-longitude enclosure derived from the exact
    /// projective root.
    pub fn angular_bracket(self) -> SkewCylinderAngularRootBracket {
        angular_bracket(self.bracket)
    }
}

#[allow(dead_code)] // Legacy lift helpers remain covered by the root-level tests.
impl SkewCylinderAngularRootBracket {
    /// Deterministic numeric representative of this exact-source bracket.
    pub fn representative(self) -> f64 {
        if self.lo == self.hi {
            self.lo
        } else {
            self.lo / 2.0 + self.hi / 2.0
        }
    }

    /// Endpoint certified on the increasing-`u` side before the root.
    ///
    /// For an exactly representable root the bracket collapses and the exact
    /// root itself is returned; otherwise this is the non-root enclosure
    /// endpoint on the requested side.
    pub const fn before_side(self) -> f64 {
        self.lo
    }

    /// Endpoint certified on the increasing-`u` side after the root.
    pub const fn after_side(self) -> f64 {
        self.hi
    }

    /// Strict representable endpoint on the increasing-`u` side before the
    /// root corridor. A collapsed exact-source corridor is opened by one
    /// representable parameter step so a subrange residual proof never uses
    /// the boundary root itself as an interior point.
    pub fn strict_before_side(self) -> f64 {
        if self.lo == 0.0 {
            TAU.next_down()
        } else {
            self.lo.next_down()
        }
    }

    /// Strict representable endpoint on the increasing-`u` side after the
    /// root corridor.
    pub fn strict_after_side(self) -> f64 {
        self.hi.next_up()
    }

    /// Lift one canonical representative into any exact full-period authored
    /// longitude window. The earliest accepted representative is chosen, so
    /// a canonical seam root maps deterministically to the window's low end.
    pub fn lift_representative(self, range: ParamRange) -> Option<f64> {
        if !range.is_finite() || range.width() != TAU {
            return None;
        }
        fit_periodic_parameter(self.representative(), range, TAU, 0.0)
    }

    /// Lift the certified before-side endpoint into a full-period window.
    pub fn lift_before_side(self, range: ParamRange) -> Option<f64> {
        if !range.is_finite() || range.width() != TAU {
            return None;
        }
        fit_periodic_parameter(self.before_side(), range, TAU, 0.0)
    }

    /// Lift the certified after-side endpoint into a full-period window.
    pub fn lift_after_side(self, range: ParamRange) -> Option<f64> {
        if !range.is_finite() || range.width() != TAU {
            return None;
        }
        fit_periodic_parameter(self.after_side(), range, TAU, 0.0)
    }
}

/// One sheet-owned root of an authored axial bound.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderAxialRoot {
    /// Caller-order bound identity.
    pub provenance: SkewCylinderAxialBoundProvenance,
    /// Ordered quadratic sheet that crosses the bound.
    pub sheet: SkewCylinderSheet,
    /// Ordinal of the distinct cyclic cut. Both sheets share this ordinal
    /// only in the exact `dz = 0` dual-height family.
    pub cyclic_ordinal: usize,
    /// Exact-source isolating bracket in its projective chart.
    pub bracket: SkewCylinderHalfAngleRootBracket,
    /// Whether the eliminated equation has a repeated/contact root.
    pub repeated: bool,
    /// Strict sheet relation immediately before the cut.
    pub before: SkewCylinderAxialRelation,
    /// Strict sheet relation immediately after the cut.
    pub after: SkewCylinderAxialRelation,
}

impl SkewCylinderAxialRoot {
    /// Convert the projective source bracket into increasing canonical
    /// longitude without changing its exact half-angle provenance.
    pub fn angular_bracket(self) -> SkewCylinderAngularRootBracket {
        angular_bracket(self.bracket)
    }
}

/// Complete finite root and open-cell topology for one axial bound.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderAxialBoundTopology {
    formula_cylinders: [Cylinder; 2],
    formula_to_source: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
    /// Sheet-owned events in canonical cyclic-cut order. Equal ordinals are
    /// ordered Lower then Upper.
    roots: Vec<SkewCylinderAxialRoot>,
    /// With roots, entry `i` is the two-sheet relation after distinct cut `i`
    /// through the open cell leading to cut `i + 1` modulo the cut count. A
    /// root-free cycle has one entry.
    open_cell_relations: Vec<[SkewCylinderAxialRelation; 2]>,
}

impl SkewCylinderAxialBoundTopology {
    /// Exact cylinders in the ruling formula order used by the classifier.
    pub const fn formula_cylinders(&self) -> [Cylinder; 2] {
        self.formula_cylinders
    }

    /// Formula-slot to caller/source-slot permutation used by the classifier.
    pub const fn formula_to_source(&self) -> [usize; 2] {
        self.formula_to_source
    }

    /// Caller-order identity of the classified axial bound.
    pub const fn provenance(&self) -> SkewCylinderAxialBoundProvenance {
        self.provenance
    }

    /// Complete sheet-owned root events in distinct cyclic-cut order.
    pub fn roots(&self) -> &[SkewCylinderAxialRoot] {
        &self.roots
    }

    /// Complete two-sheet relations on every cyclic open cell.
    pub fn open_cell_relations(&self) -> &[[SkewCylinderAxialRelation; 2]] {
        &self.open_cell_relations
    }

    pub(crate) fn exact_root_polynomial(
        &self,
        chart: SkewCylinderHalfAngleChart,
    ) -> Result<ExactPolynomial, SkewCylinderAxialRootFailure> {
        let canonical_operand =
            canonical_operand(self.formula_to_source, self.provenance.source_operand)?;
        let coefficients = ExactSkewCylinderAlgebra::new(self.formula_cylinders)?
            .axial_equation(canonical_operand, self.provenance.value)?
            .coefficients();
        Ok(coefficients.half_angle_polynomial(match chart {
            SkewCylinderHalfAngleChart::Tangent => HalfAngleChart::Tangent,
            SkewCylinderHalfAngleChart::Cotangent => HalfAngleChart::Cotangent,
        })?)
    }
}

/// Stable fail-closed causes for one exact axial-bound query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderAxialRootFailure {
    /// The complete fixed-work query was not fully reserved.
    WorkLimit {
        /// Minimum logical work required for publication.
        required: u64,
        /// Logical work made available by the caller.
        provided: u64,
    },
    /// Formula slots were not mapped by one of the two valid permutations.
    InvalidSourcePermutation,
    /// Bound provenance named a caller/source slot outside `0..2`.
    InvalidSourceOperand,
    /// The authored bound was NaN or infinite.
    NonFiniteBound,
    /// The exact ruling construction is degenerate for this formula order.
    DegenerateExactSource,
    /// A sheet equation vanishes identically on the queried bound.
    IdenticallyOnBound,
    /// Exact polynomial isolation refused or exhausted its safe envelope.
    ExactArithmetic(RootIsolationFailure),
    /// Root events and open-cell relations did not form one cyclic topology.
    InconsistentTopology,
}

impl From<RootIsolationFailure> for SkewCylinderAxialRootFailure {
    fn from(failure: RootIsolationFailure) -> Self {
        Self::ExactArithmetic(failure)
    }
}

impl From<CyclicSecondHarmonicFailure> for SkewCylinderAxialRootFailure {
    fn from(failure: CyclicSecondHarmonicFailure) -> Self {
        match failure {
            CyclicSecondHarmonicFailure::WorkLimit { required, provided } => {
                Self::WorkLimit { required, provided }
            }
            CyclicSecondHarmonicFailure::ExactArithmetic(failure) => Self::ExactArithmetic(failure),
            CyclicSecondHarmonicFailure::InconsistentChartTopology => Self::InconsistentTopology,
        }
    }
}

/// Exact discriminant shortcut shared with the strict-positive admission.
#[derive(Debug)]
pub enum ExactSkewCylinderDiscriminant {
    /// Root-free discriminant with one strict sign over the full cycle.
    Strict(StrictSign),
    /// The exact discriminant has a zero/contact on the full cycle.
    Contact,
    /// Nonconstant cyclic second harmonic requiring exact classification.
    Harmonic {
        /// Exact-source second-harmonic coefficients.
        coefficients: SecondHarmonicCoefficients,
        /// Whether an exactly evaluated cardinal longitude is a contact.
        cardinal_contact: bool,
    },
}

/// Sealed exact strict-positive admission for two nonparallel formula cylinders.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
    formula_cylinders: [Cylinder; 2],
}

/// Sealed complete root-and-cell topology for a non-strict skew-cylinder
/// discriminant.
///
/// Roots retain exact projective source identity. Open-cell signs remain
/// private so discovery cannot smuggle a downstream topology verdict; the
/// isolated-contact consumer asks this certificate to re-check its complete
/// one-root/negative-cell obligation.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderDiscriminantContactTopologyCertificate {
    formula_cylinders: [Cylinder; 2],
    identically_zero: bool,
    roots: Vec<SkewCylinderDiscriminantRoot>,
    open_cell_signs: Vec<StrictSign>,
}

/// Position of the sole strict-positive cell in a two-simple-root
/// skew-cylinder discriminant cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderFoldedSupportCellLocation {
    /// The positive cell runs from the first canonical root to the second.
    BetweenCanonicalRoots,
    /// The positive cell runs from the second canonical root across the
    /// projective seam to the first.
    AcrossCanonicalSeam,
}

/// Sealed exact topology of one positive-length folded support curve.
///
/// The two simple roots are the only support joins. One complementary cyclic
/// cell is strictly positive and carries two regular sheets; the other is a
/// strict miss. Rounded angular brackets are retained only to construct
/// guarded evaluators after this exact sign topology has been sealed.
#[derive(Debug, Clone, PartialEq)]
pub struct SkewCylinderFoldedSupportTopologyCertificate {
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
    positive_cell: SkewCylinderFoldedSupportCellLocation,
}

impl SkewCylinderFoldedSupportTopologyCertificate {
    /// Exact cylinders in ruling-formula order.
    pub const fn formula_cylinders(&self) -> [Cylinder; 2] {
        self.topology.formula_cylinders
    }

    /// The two exact simple roots in canonical cyclic order.
    pub fn roots(&self) -> [SkewCylinderDiscriminantRoot; 2] {
        self.topology
            .roots
            .as_slice()
            .try_into()
            .expect("sealed folded support topology retains two roots")
    }

    /// Location of the sole strict-positive cell.
    pub const fn positive_cell(&self) -> SkewCylinderFoldedSupportCellLocation {
        self.positive_cell
    }

    /// Original complete exact discriminant topology.
    pub const fn topology(&self) -> &SkewCylinderDiscriminantContactTopologyCertificate {
        &self.topology
    }

    pub(crate) fn positive_radicand_lower_bound(
        &self,
        projective: RootBracket,
    ) -> Result<f64, SkewCylinderAxialRootFailure> {
        let roots = self.roots();
        let projective_roots = roots.map(|root| tangent_projective_interval(root.bracket));
        let [Some(first), Some(second)] = projective_roots else {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        };
        if self.positive_cell != SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            || projective.lo <= first.hi()
            || projective.hi >= second.lo()
            || projective.lo >= projective.hi
        {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        }
        let polynomial = self
            .topology
            .exact_discriminant_root_polynomial(SkewCylinderHalfAngleChart::Tangent)?;
        let numerator = polynomial
            .positive_lower_bound_on_interval(projective.lo, projective.hi)?
            .ok_or(SkewCylinderAxialRootFailure::InconsistentTopology)?;
        let projective_interval = kcore::interval::Interval::new(projective.lo, projective.hi);
        // The exact topology polynomial is `4 * (K - L²)` after the
        // half-angle denominator is cleared. Recover a lower bound for the
        // unscaled radicand consumed by the branch evaluator.
        let denominator = kcore::interval::Interval::point(4.0)
            * (kcore::interval::Interval::point(1.0) + projective_interval.square()).square();
        let lower = kcore::interval::Interval::point(numerator)
            .checked_div(denominator)
            .ok_or(SkewCylinderAxialRootFailure::InconsistentTopology)?
            .lo();
        if lower.is_finite() && lower > 0.0 {
            Ok(lower)
        } else {
            Err(SkewCylinderAxialRootFailure::InconsistentTopology)
        }
    }
}

/// Seal exactly two simple discriminant roots with one strict-positive and
/// one strict-negative complementary cell as a folded support topology.
pub fn certify_skew_cylinder_folded_support_topology(
    topology: SkewCylinderDiscriminantContactTopologyCertificate,
) -> Result<SkewCylinderFoldedSupportTopologyCertificate, SkewCylinderAxialRootFailure> {
    let positive_cell = match (
        topology.identically_zero,
        topology.roots.as_slice(),
        topology.open_cell_signs.as_slice(),
    ) {
        (false, [first, second], [StrictSign::Positive, StrictSign::Negative])
            if !first.repeated() && !second.repeated() =>
        {
            SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
        }
        (false, [first, second], [StrictSign::Negative, StrictSign::Positive])
            if !first.repeated() && !second.repeated() =>
        {
            SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
        }
        _ => return Err(SkewCylinderAxialRootFailure::InconsistentTopology),
    };
    Ok(SkewCylinderFoldedSupportTopologyCertificate {
        topology,
        positive_cell,
    })
}

impl SkewCylinderDiscriminantContactTopologyCertificate {
    /// Exact cylinders in the ruling formula order used by the classifier.
    pub const fn formula_cylinders(&self) -> [Cylinder; 2] {
        self.formula_cylinders
    }

    /// Complete distinct discriminant roots in canonical cyclic order.
    pub fn roots(&self) -> &[SkewCylinderDiscriminantRoot] {
        &self.roots
    }

    /// Whether every discriminant coefficient is exactly zero.
    pub const fn is_identically_zero(&self) -> bool {
        self.identically_zero
    }

    /// Return the sole exact support-contact root only when every other
    /// longitude is a strict miss.
    pub fn isolated_support_root(&self) -> Option<SkewCylinderDiscriminantRoot> {
        match (
            self.identically_zero,
            self.roots.as_slice(),
            self.open_cell_signs.as_slice(),
        ) {
            (false, [root], [StrictSign::Negative]) if root.repeated() => Some(*root),
            _ => None,
        }
    }

    pub(crate) fn support_root_matches_axial_bound(
        &self,
        root: SkewCylinderDiscriminantRoot,
        formula_to_source: [usize; 2],
        provenance: SkewCylinderAxialBoundProvenance,
    ) -> Result<bool, SkewCylinderAxialRootFailure> {
        if self.isolated_support_root() != Some(root) {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        }
        let chart = root.bracket().chart;
        let discriminant = self.exact_discriminant_root_polynomial(chart)?;
        let canonical_operand = canonical_operand(formula_to_source, provenance.source_operand)?;
        let axial = ExactSkewCylinderAlgebra::new(self.formula_cylinders)?
            .axial_equation(canonical_operand, provenance.value)?
            .coefficients()
            .half_angle_polynomial(match chart {
                SkewCylinderHalfAngleChart::Tangent => HalfAngleChart::Tangent,
                SkewCylinderHalfAngleChart::Cotangent => HalfAngleChart::Cotangent,
            })?;
        let bracket = RootBracket {
            lo: root.bracket().lo,
            hi: root.bracket().hi,
        };
        classify_exact_root_relation(&discriminant, bracket, &axial, bracket)
            .map(|relation| relation == ExactRootRelation::Same)
            .map_err(|_| SkewCylinderAxialRootFailure::InconsistentTopology)
    }

    pub(crate) fn exact_discriminant_root_polynomial(
        &self,
        chart: SkewCylinderHalfAngleChart,
    ) -> Result<ExactPolynomial, SkewCylinderAxialRootFailure> {
        let ExactSkewCylinderDiscriminant::Harmonic { coefficients, .. } =
            ExactSkewCylinderAlgebra::new(self.formula_cylinders)?.discriminant()?
        else {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        };
        Ok(coefficients.half_angle_polynomial(match chart {
            SkewCylinderHalfAngleChart::Tangent => HalfAngleChart::Tangent,
            SkewCylinderHalfAngleChart::Cotangent => HalfAngleChart::Cotangent,
        })?)
    }
}

impl SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
    /// Exact cylinders in the admitted ruling formula order.
    pub const fn formula_cylinders(self) -> [Cylinder; 2] {
        self.formula_cylinders
    }

    /// Existing logical work represented by this one parameterization.
    pub const fn work(self) -> u64 {
        SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK
    }
}

/// Complete exact discriminant outcome for one ruling parameterization.
// The strict-positive admission certificate stays inline so the established
// Copy certificate contract survives value handoff without indirection.
#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, PartialEq)]
pub enum SkewCylinderExactDiscriminantTopology {
    /// Both regular infinite-support sheets exist over the complete cycle.
    StrictPositive(SkewCylinderStrictPositiveTwoSheetAdmissionCertificate),
    /// No real infinite-support sheet exists over the complete cycle.
    StrictNegative,
    /// Complete exact roots and open-cell signs for a non-strict
    /// discriminant.
    Contact(Box<SkewCylinderDiscriminantContactTopologyCertificate>),
}

/// Classify the complete skew-cylinder discriminant in one formula order.
pub fn classify_skew_cylinder_exact_discriminant(
    cylinders: [Cylinder; 2],
    work_limit: u64,
) -> Result<SkewCylinderExactDiscriminantTopology, SkewCylinderAxialRootFailure> {
    if work_limit < SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK {
        return Err(SkewCylinderAxialRootFailure::WorkLimit {
            required: SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            provided: work_limit,
        });
    }
    let classification = exact_skew_cylinder_discriminant(cylinders)?;
    let topology = match classification {
        ExactSkewCylinderDiscriminant::Strict(StrictSign::Positive) => {
            return Ok(SkewCylinderExactDiscriminantTopology::StrictPositive(
                SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
                    formula_cylinders: cylinders,
                },
            ));
        }
        ExactSkewCylinderDiscriminant::Strict(StrictSign::Negative) => {
            return Ok(SkewCylinderExactDiscriminantTopology::StrictNegative);
        }
        ExactSkewCylinderDiscriminant::Contact => CyclicSecondHarmonicTopology::IdenticallyZero,
        ExactSkewCylinderDiscriminant::Harmonic { coefficients, .. } => {
            classify_cyclic_second_harmonic(&coefficients, work_limit)?
        }
    };
    if let Some(sign) = topology.strict_full_cycle_sign() {
        return Ok(match sign {
            StrictSign::Positive => SkewCylinderExactDiscriminantTopology::StrictPositive(
                SkewCylinderStrictPositiveTwoSheetAdmissionCertificate {
                    formula_cylinders: cylinders,
                },
            ),
            StrictSign::Negative => SkewCylinderExactDiscriminantTopology::StrictNegative,
        });
    }
    let (identically_zero, roots, open_cell_signs) = match topology.into_parts() {
        Some((roots, open_cell_signs)) => (
            false,
            roots
                .into_iter()
                .map(|root| SkewCylinderDiscriminantRoot {
                    bracket: SkewCylinderHalfAngleRootBracket {
                        chart: root.bracket.chart.into(),
                        lo: root.bracket.lo,
                        hi: root.bracket.hi,
                    },
                    repeated: root.repeated,
                })
                .collect(),
            open_cell_signs,
        ),
        None => (true, Vec::new(), Vec::new()),
    };
    Ok(SkewCylinderExactDiscriminantTopology::Contact(Box::new(
        SkewCylinderDiscriminantContactTopologyCertificate {
            formula_cylinders: cylinders,
            identically_zero,
            roots,
            open_cell_signs,
        },
    )))
}

/// Build the existing exact skew-cylinder discriminant from source values.
pub fn exact_skew_cylinder_discriminant(
    cylinders: [Cylinder; 2],
) -> Result<ExactSkewCylinderDiscriminant, SkewCylinderAxialRootFailure> {
    ExactSkewCylinderAlgebra::new(cylinders)?.discriminant()
}

/// Classify one caller-authored axial bound over a complete canonical cycle.
///
/// `cylinders` must be the strict-positive solver's canonical ruling order.
/// `canonical_to_source` maps those two entries back to caller operand order
/// and must be either `[0, 1]` or `[1, 0]`. This query deliberately does not
/// require axial windows or a paired branch certificate: it creates the cuts
/// that a later finite-window proof consumes.
pub fn classify_skew_cylinder_axial_bound(
    cylinders: [Cylinder; 2],
    canonical_to_source: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
    work_limit: u64,
) -> Result<SkewCylinderAxialBoundTopology, SkewCylinderAxialRootFailure> {
    let canonical_operand = canonical_operand(canonical_to_source, provenance.source_operand)?;
    if !provenance.value.is_finite() {
        return Err(SkewCylinderAxialRootFailure::NonFiniteBound);
    }
    if work_limit < SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK {
        return Err(SkewCylinderAxialRootFailure::WorkLimit {
            required: SKEW_CYLINDER_AXIAL_BOUND_EXACT_WORK,
            provided: work_limit,
        });
    }

    let algebra = ExactSkewCylinderAlgebra::new(cylinders)?;
    let equation = algebra.axial_equation(canonical_operand, provenance.value)?;
    let coefficients = equation.coefficients();
    let topology = classify_cyclic_second_harmonic(&coefficients, work_limit)?;
    topology_from_classification(
        equation,
        cylinders,
        canonical_to_source,
        provenance,
        topology,
    )
}

fn canonical_operand(
    canonical_to_source: [usize; 2],
    source_operand: usize,
) -> Result<usize, SkewCylinderAxialRootFailure> {
    if !matches!(canonical_to_source, [0, 1] | [1, 0]) {
        return Err(SkewCylinderAxialRootFailure::InvalidSourcePermutation);
    }
    if source_operand > 1 {
        return Err(SkewCylinderAxialRootFailure::InvalidSourceOperand);
    }
    canonical_to_source
        .iter()
        .position(|operand| *operand == source_operand)
        .ok_or(SkewCylinderAxialRootFailure::InvalidSourceOperand)
}

#[derive(Debug, Clone)]
enum AxialEquation {
    Canonical {
        eliminated: ExactSecondHarmonic,
        selector: ExactFirstHarmonic,
    },
    Opposite {
        eliminated: ExactSecondHarmonic,
        selector: ExactFirstHarmonic,
        dz_sign: i8,
        e_sign: i8,
    },
    SharedOppositeHeight {
        direct: ExactFirstHarmonic,
        e_sign: i8,
    },
}

impl AxialEquation {
    fn coefficients(&self) -> SecondHarmonicCoefficients {
        match self {
            Self::Canonical { eliminated, .. } | Self::Opposite { eliminated, .. } => {
                eliminated.coefficients()
            }
            Self::SharedOppositeHeight { direct, .. } => {
                ExactSecondHarmonic::from_first(direct.clone()).coefficients()
            }
        }
    }
}

fn topology_from_classification(
    equation: AxialEquation,
    formula_cylinders: [Cylinder; 2],
    formula_to_source: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
    topology: CyclicSecondHarmonicTopology,
) -> Result<SkewCylinderAxialBoundTopology, SkewCylinderAxialRootFailure> {
    let CyclicSecondHarmonicTopology::Nonzero {
        roots,
        open_cell_signs,
    } = topology
    else {
        return Err(SkewCylinderAxialRootFailure::IdenticallyOnBound);
    };
    if roots.is_empty() {
        let sign = *open_cell_signs
            .first()
            .ok_or(SkewCylinderAxialRootFailure::InconsistentTopology)?;
        let relations = root_free_relations(&equation, sign)?;
        return Ok(SkewCylinderAxialBoundTopology {
            formula_cylinders,
            formula_to_source,
            provenance,
            roots: Vec::new(),
            open_cell_relations: vec![relations],
        });
    }
    if roots.len() != open_cell_signs.len() {
        return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
    }

    match &equation {
        AxialEquation::SharedOppositeHeight { e_sign, .. } => shared_height_topology(
            formula_cylinders,
            formula_to_source,
            provenance,
            &roots,
            &open_cell_signs,
            *e_sign,
        ),
        AxialEquation::Canonical { selector, .. } | AxialEquation::Opposite { selector, .. } => {
            let selector_signs = roots
                .iter()
                .map(|root| selector_sign_at_root(selector, *root))
                .collect::<Result<Vec<_>, _>>()?;
            selected_sheet_topology(
                equation,
                formula_cylinders,
                formula_to_source,
                provenance,
                roots,
                open_cell_signs,
                selector_signs,
            )
        }
    }
}

fn root_free_relations(
    equation: &AxialEquation,
    eliminated_sign: StrictSign,
) -> Result<[SkewCylinderAxialRelation; 2], SkewCylinderAxialRootFailure> {
    match equation {
        AxialEquation::SharedOppositeHeight { direct, e_sign } => {
            let direct_sign = direct.sign_at_zero()?;
            if direct_sign != eliminated_sign {
                return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
            }
            let relation = relation_from_product_sign(direct_sign, *e_sign)?;
            Ok([relation; 2])
        }
        AxialEquation::Canonical { selector, .. } | AxialEquation::Opposite { selector, .. } => {
            if eliminated_sign == StrictSign::Negative {
                // The sheet relations are opposite wherever their eliminated
                // product is negative. A selector zero inside this cell is
                // therefore harmless and occurs in symmetric valid families.
                general_cell_relations(equation, eliminated_sign, StrictSign::Positive)
            } else {
                general_cell_relations(equation, eliminated_sign, selector.sign_at_zero()?)
            }
        }
    }
}

fn selected_sheet_topology(
    equation: AxialEquation,
    formula_cylinders: [Cylinder; 2],
    formula_to_source: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
    roots: Vec<CyclicRoot>,
    open_cell_signs: Vec<StrictSign>,
    selector_signs: Vec<StrictSign>,
) -> Result<SkewCylinderAxialBoundTopology, SkewCylinderAxialRootFailure> {
    let open_cell_relations = open_cell_signs
        .iter()
        .copied()
        .zip(selector_signs.iter().copied())
        .map(|(eliminated, selector)| general_cell_relations(&equation, eliminated, selector))
        .collect::<Result<Vec<_>, _>>()?;
    let root_count = roots.len();
    let mut events = Vec::with_capacity(root_count);
    for (ordinal, root) in roots.into_iter().enumerate() {
        let sheet = selected_sheet(&equation, selector_signs[ordinal])?;
        let before =
            open_cell_relations[(ordinal + root_count - 1) % root_count][sheet_index(sheet)];
        let after = open_cell_relations[ordinal][sheet_index(sheet)];
        let other = 1 - sheet_index(sheet);
        if open_cell_relations[(ordinal + root_count - 1) % root_count][other]
            != open_cell_relations[ordinal][other]
        {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        }
        if !root.repeated && before == after {
            return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
        }
        events.push(root_event(provenance, sheet, ordinal, root, before, after));
    }
    Ok(SkewCylinderAxialBoundTopology {
        formula_cylinders,
        formula_to_source,
        provenance,
        roots: events,
        open_cell_relations,
    })
}

fn shared_height_topology(
    formula_cylinders: [Cylinder; 2],
    formula_to_source: [usize; 2],
    provenance: SkewCylinderAxialBoundProvenance,
    roots: &[CyclicRoot],
    open_cell_signs: &[StrictSign],
    e_sign: i8,
) -> Result<SkewCylinderAxialBoundTopology, SkewCylinderAxialRootFailure> {
    let open_cell_relations = open_cell_signs
        .iter()
        .copied()
        .map(|sign| {
            let relation = relation_from_product_sign(sign, e_sign)?;
            Ok([relation; 2])
        })
        .collect::<Result<Vec<_>, SkewCylinderAxialRootFailure>>()?;
    let root_count = roots.len();
    let mut events = Vec::with_capacity(2 * root_count);
    for (ordinal, root) in roots.iter().copied().enumerate() {
        for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
            let index = sheet_index(sheet);
            let before = open_cell_relations[(ordinal + root_count - 1) % root_count][index];
            let after = open_cell_relations[ordinal][index];
            if !root.repeated && before == after {
                return Err(SkewCylinderAxialRootFailure::InconsistentTopology);
            }
            events.push(root_event(provenance, sheet, ordinal, root, before, after));
        }
    }
    Ok(SkewCylinderAxialBoundTopology {
        formula_cylinders,
        formula_to_source,
        provenance,
        roots: events,
        open_cell_relations,
    })
}

fn root_event(
    provenance: SkewCylinderAxialBoundProvenance,
    sheet: SkewCylinderSheet,
    cyclic_ordinal: usize,
    root: CyclicRoot,
    before: SkewCylinderAxialRelation,
    after: SkewCylinderAxialRelation,
) -> SkewCylinderAxialRoot {
    SkewCylinderAxialRoot {
        provenance,
        sheet,
        cyclic_ordinal,
        bracket: SkewCylinderHalfAngleRootBracket {
            chart: root.bracket.chart.into(),
            lo: root.bracket.lo,
            hi: root.bracket.hi,
        },
        repeated: root.repeated,
        before,
        after,
    }
}

fn selected_sheet(
    equation: &AxialEquation,
    selector_sign: StrictSign,
) -> Result<SkewCylinderSheet, SkewCylinderAxialRootFailure> {
    match equation {
        AxialEquation::Canonical { .. } => Ok(match selector_sign {
            StrictSign::Negative => SkewCylinderSheet::Lower,
            StrictSign::Positive => SkewCylinderSheet::Upper,
        }),
        AxialEquation::Opposite { dz_sign, .. } => {
            let product = sign_product(strict_sign_i8(selector_sign), *dz_sign)?;
            Ok(if product > 0 {
                SkewCylinderSheet::Lower
            } else {
                SkewCylinderSheet::Upper
            })
        }
        AxialEquation::SharedOppositeHeight { .. } => {
            Err(SkewCylinderAxialRootFailure::InconsistentTopology)
        }
    }
}

fn general_cell_relations(
    equation: &AxialEquation,
    eliminated_sign: StrictSign,
    selector_sign: StrictSign,
) -> Result<[SkewCylinderAxialRelation; 2], SkewCylinderAxialRootFailure> {
    match (equation, eliminated_sign) {
        (AxialEquation::Canonical { .. }, StrictSign::Negative) => Ok([
            SkewCylinderAxialRelation::Below,
            SkewCylinderAxialRelation::Above,
        ]),
        (AxialEquation::Canonical { .. }, StrictSign::Positive) => {
            let relation = match selector_sign {
                StrictSign::Negative => SkewCylinderAxialRelation::Above,
                StrictSign::Positive => SkewCylinderAxialRelation::Below,
            };
            Ok([relation; 2])
        }
        (
            AxialEquation::Opposite {
                dz_sign, e_sign, ..
            },
            StrictSign::Negative,
        ) => {
            let direction = sign_product(*dz_sign, *e_sign)?;
            Ok(if direction > 0 {
                [
                    SkewCylinderAxialRelation::Below,
                    SkewCylinderAxialRelation::Above,
                ]
            } else {
                [
                    SkewCylinderAxialRelation::Above,
                    SkewCylinderAxialRelation::Below,
                ]
            })
        }
        (AxialEquation::Opposite { e_sign, .. }, StrictSign::Positive) => {
            let relation = relation_from_product_sign(selector_sign, *e_sign)?;
            Ok([relation; 2])
        }
        (AxialEquation::SharedOppositeHeight { .. }, _) => {
            Err(SkewCylinderAxialRootFailure::InconsistentTopology)
        }
    }
}

fn relation_from_product_sign(
    sign: StrictSign,
    factor_sign: i8,
) -> Result<SkewCylinderAxialRelation, SkewCylinderAxialRootFailure> {
    let product = sign_product(strict_sign_i8(sign), factor_sign)?;
    Ok(if product < 0 {
        SkewCylinderAxialRelation::Below
    } else {
        SkewCylinderAxialRelation::Above
    })
}

fn strict_sign_i8(sign: StrictSign) -> i8 {
    match sign {
        StrictSign::Negative => -1,
        StrictSign::Positive => 1,
    }
}

fn sign_product(lhs: i8, rhs: i8) -> Result<i8, SkewCylinderAxialRootFailure> {
    match lhs * rhs {
        -1 => Ok(-1),
        1 => Ok(1),
        _ => Err(SkewCylinderAxialRootFailure::DegenerateExactSource),
    }
}

fn sheet_index(sheet: SkewCylinderSheet) -> usize {
    match sheet {
        SkewCylinderSheet::Lower => 0,
        SkewCylinderSheet::Upper => 1,
    }
}

fn selector_sign_at_root(
    selector: &ExactFirstHarmonic,
    root: CyclicRoot,
) -> Result<StrictSign, SkewCylinderAxialRootFailure> {
    if let Some(sign) = selector.strict_full_cycle_sign()? {
        return Ok(sign);
    }
    if let Some(sign) = selector.strict_sign_on_chart_bracket(root.bracket)? {
        return Ok(sign);
    }
    let polynomial = selector
        .chart_polynomial(root.bracket.chart)?
        .ok_or(SkewCylinderAxialRootFailure::InconsistentTopology)?;
    match polynomial.isolate(root.bracket.lo, root.bracket.hi) {
        RootIsolation::Complete(roots) if roots.is_empty() => {
            strict_sign(polynomial.evaluate(root.bracket.lo)?.sign())
        }
        RootIsolation::Complete(_) => Err(SkewCylinderAxialRootFailure::InconsistentTopology),
        RootIsolation::Ambiguous(failure) => Err(failure.into()),
    }
}

fn strict_sign(sign: i8) -> Result<StrictSign, SkewCylinderAxialRootFailure> {
    match sign {
        -1 => Ok(StrictSign::Negative),
        1 => Ok(StrictSign::Positive),
        _ => Err(SkewCylinderAxialRootFailure::InconsistentTopology),
    }
}

fn angular_bracket(bracket: SkewCylinderHalfAngleRootBracket) -> SkewCylinderAngularRootBracket {
    let first = canonical_angle(half_angle_to_parameter(bracket.chart, bracket.lo));
    let second = canonical_angle(half_angle_to_parameter(bracket.chart, bracket.hi));
    match bracket.chart {
        SkewCylinderHalfAngleChart::Tangent => SkewCylinderAngularRootBracket {
            lo: first,
            hi: second,
        },
        SkewCylinderHalfAngleChart::Cotangent => SkewCylinderAngularRootBracket {
            lo: second,
            hi: first,
        },
    }
}

fn half_angle_to_parameter(chart: SkewCylinderHalfAngleChart, parameter: f64) -> f64 {
    match chart {
        SkewCylinderHalfAngleChart::Tangent => 2.0 * math::atan2(parameter, 1.0),
        SkewCylinderHalfAngleChart::Cotangent => 2.0 * math::atan2(1.0, parameter),
    }
}

fn canonical_angle(parameter: f64) -> f64 {
    let mut parameter = parameter % TAU;
    if parameter < 0.0 {
        parameter += TAU;
    }
    if parameter == TAU || parameter == -0.0 {
        0.0
    } else {
        parameter
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

#[derive(Debug, Clone)]
struct ExactFirstHarmonic {
    constant: ExactScalar,
    cosine: ExactScalar,
    sine: ExactScalar,
}

impl ExactFirstHarmonic {
    fn constant(value: ExactScalar) -> Self {
        Self {
            constant: value,
            cosine: ExactScalar::zero(),
            sine: ExactScalar::zero(),
        }
    }

    fn add(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.add(&rhs.constant)?,
            cosine: self.cosine.add(&rhs.cosine)?,
            sine: self.sine.add(&rhs.sine)?,
        })
    }

    fn sub(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.sub(&rhs.constant)?,
            cosine: self.cosine.sub(&rhs.cosine)?,
            sine: self.sine.sub(&rhs.sine)?,
        })
    }

    fn scale(&self, factor: &ExactScalar) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.mul(factor)?,
            cosine: self.cosine.mul(factor)?,
            sine: self.sine.mul(factor)?,
        })
    }

    fn square(&self) -> Result<ExactSecondHarmonic, RootIsolationFailure> {
        self.product(self)
    }

    fn product(&self, rhs: &Self) -> Result<ExactSecondHarmonic, RootIsolationFailure> {
        let cosine_product = self.cosine.mul(&rhs.cosine)?;
        let sine_product = self.sine.mul(&rhs.sine)?;
        Ok(ExactSecondHarmonic {
            constant: self
                .constant
                .mul(&rhs.constant)?
                .add(&cosine_product.add(&sine_product)?.scale(0.5)?)?,
            cosine: self
                .constant
                .mul(&rhs.cosine)?
                .add(&rhs.constant.mul(&self.cosine)?)?,
            sine: self
                .constant
                .mul(&rhs.sine)?
                .add(&rhs.constant.mul(&self.sine)?)?,
            cosine2: cosine_product.sub(&sine_product)?.scale(0.5)?,
            sine2: self
                .cosine
                .mul(&rhs.sine)?
                .add(&self.sine.mul(&rhs.cosine)?)?
                .scale(0.5)?,
        })
    }

    fn sign_at_zero(&self) -> Result<StrictSign, SkewCylinderAxialRootFailure> {
        strict_sign(self.constant.add(&self.cosine)?.sign())
    }

    fn strict_full_cycle_sign(&self) -> Result<Option<StrictSign>, SkewCylinderAxialRootFailure> {
        let constant_squared = self.constant.mul(&self.constant)?;
        let radial_squared = self
            .cosine
            .mul(&self.cosine)?
            .add(&self.sine.mul(&self.sine)?)?;
        if constant_squared.sub(&radial_squared)?.sign() > 0 {
            Ok(Some(strict_sign(self.constant.sign())?))
        } else {
            Ok(None)
        }
    }

    /// Prove a selector sign on one already-isolated root corridor through
    /// the exact quadratic Bernstein control values in its owning chart.
    /// This avoids a second Sturm chain on adjacent-float endpoints while
    /// retaining an exact no-zero proof for the complete corridor.
    fn strict_sign_on_chart_bracket(
        &self,
        bracket: CyclicRootBracket,
    ) -> Result<Option<StrictSign>, SkewCylinderAxialRootFailure> {
        let first = self.constant.add(&self.cosine)?;
        let middle = self.sine.scale(2.0)?;
        let last = self.constant.sub(&self.cosine)?;
        let [constant, linear, quadratic] = match bracket.chart {
            HalfAngleChart::Tangent => [first, middle, last],
            HalfAngleChart::Cotangent => [last, middle, first],
        };
        let evaluate = |parameter: f64| -> Result<ExactScalar, RootIsolationFailure> {
            quadratic
                .scale(parameter)?
                .add(&linear)?
                .scale(parameter)?
                .add(&constant)
        };
        let at_lo = evaluate(bracket.lo)?;
        if bracket.lo == bracket.hi {
            return Ok(match at_lo.sign() {
                -1 => Some(StrictSign::Negative),
                1 => Some(StrictSign::Positive),
                _ => None,
            });
        }
        let at_hi = evaluate(bracket.hi)?;
        let width = ExactScalar::from_f64(bracket.hi)?.sub(&ExactScalar::from_f64(bracket.lo)?)?;
        let derivative_at_lo = quadratic.scale(2.0)?.scale(bracket.lo)?.add(&linear)?;
        let middle_control = at_lo.add(&derivative_at_lo.mul(&width)?.scale(0.5)?)?;
        let signs = [at_lo.sign(), middle_control.sign(), at_hi.sign()];
        Ok(if signs.iter().all(|sign| *sign < 0) {
            Some(StrictSign::Negative)
        } else if signs.iter().all(|sign| *sign > 0) {
            Some(StrictSign::Positive)
        } else {
            None
        })
    }

    fn chart_polynomial(
        &self,
        chart: HalfAngleChart,
    ) -> Result<Option<ExactPolynomial>, RootIsolationFailure> {
        let first = self.constant.add(&self.cosine)?;
        let middle = self.sine.scale(2.0)?;
        let last = self.constant.sub(&self.cosine)?;
        let coefficients = match chart {
            HalfAngleChart::Tangent => vec![first, middle, last],
            HalfAngleChart::Cotangent => vec![last, middle, first],
        };
        if coefficients.iter().all(ExactScalar::is_zero) {
            Ok(None)
        } else {
            ExactPolynomial::new(coefficients).map(Some)
        }
    }
}

#[derive(Debug, Clone)]
struct ExactSecondHarmonic {
    constant: ExactScalar,
    cosine: ExactScalar,
    sine: ExactScalar,
    cosine2: ExactScalar,
    sine2: ExactScalar,
}

impl ExactSecondHarmonic {
    fn constant(value: ExactScalar) -> Self {
        Self {
            constant: value,
            cosine: ExactScalar::zero(),
            sine: ExactScalar::zero(),
            cosine2: ExactScalar::zero(),
            sine2: ExactScalar::zero(),
        }
    }

    fn from_first(value: ExactFirstHarmonic) -> Self {
        Self {
            constant: value.constant,
            cosine: value.cosine,
            sine: value.sine,
            cosine2: ExactScalar::zero(),
            sine2: ExactScalar::zero(),
        }
    }

    fn add(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.add(&rhs.constant)?,
            cosine: self.cosine.add(&rhs.cosine)?,
            sine: self.sine.add(&rhs.sine)?,
            cosine2: self.cosine2.add(&rhs.cosine2)?,
            sine2: self.sine2.add(&rhs.sine2)?,
        })
    }

    fn sub(&self, rhs: &Self) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.sub(&rhs.constant)?,
            cosine: self.cosine.sub(&rhs.cosine)?,
            sine: self.sine.sub(&rhs.sine)?,
            cosine2: self.cosine2.sub(&rhs.cosine2)?,
            sine2: self.sine2.sub(&rhs.sine2)?,
        })
    }

    fn scale(&self, factor: &ExactScalar) -> Result<Self, RootIsolationFailure> {
        Ok(Self {
            constant: self.constant.mul(factor)?,
            cosine: self.cosine.mul(factor)?,
            sine: self.sine.mul(factor)?,
            cosine2: self.cosine2.mul(factor)?,
            sine2: self.sine2.mul(factor)?,
        })
    }

    fn coefficients(&self) -> SecondHarmonicCoefficients {
        SecondHarmonicCoefficients::new(
            self.constant.clone(),
            self.cosine.clone(),
            self.sine.clone(),
            self.cosine2.clone(),
            self.sine2.clone(),
        )
    }

    fn cardinal_contact(&self) -> Result<bool, RootIsolationFailure> {
        let values = [
            self.constant.add(&self.cosine)?.add(&self.cosine2)?,
            self.constant.add(&self.sine)?.sub(&self.cosine2)?,
            self.constant.sub(&self.cosine)?.add(&self.cosine2)?,
            self.constant.sub(&self.sine)?.sub(&self.cosine2)?,
        ];
        let signs = values.each_ref().map(ExactScalar::sign);
        Ok(signs.contains(&0) || (signs.contains(&-1) && signs.contains(&1)))
    }
}

#[derive(Debug, Clone)]
struct ExactSkewCylinderAlgebra {
    cylinders: [Cylinder; 2],
    e: ExactScalar,
    directions: [ExactScalar; 2],
    a: ExactScalar,
    k: ExactScalar,
    l: ExactFirstHarmonic,
}

impl ExactSkewCylinderAlgebra {
    fn new(cylinders: [Cylinder; 2]) -> Result<Self, SkewCylinderAxialRootFailure> {
        let first = cylinders[0];
        let second = cylinders[1];
        let second_axes = [
            exact_vector(second.frame().x().to_array())?,
            exact_vector(second.frame().y().to_array())?,
            exact_vector(second.frame().z().to_array())?,
        ];
        let ruling = exact_vector(first.frame().z().to_array())?;
        let directions = [
            exact_determinant_expansion(
                ruling.clone(),
                second_axes[1].clone(),
                second_axes[2].clone(),
            )?,
            exact_determinant_expansion(
                second_axes[0].clone(),
                ruling.clone(),
                second_axes[2].clone(),
            )?,
        ];
        let a = directions[0]
            .mul(&directions[0])?
            .add(&directions[1].mul(&directions[1])?)?;
        let e = exact_determinant_expansion(
            second_axes[0].clone(),
            second_axes[1].clone(),
            second_axes[2].clone(),
        )?;
        if a.sign() <= 0 || e.sign() == 0 {
            return Err(SkewCylinderAxialRootFailure::DegenerateExactSource);
        }

        let coordinates = [
            radial_coordinate_harmonic(first, second, DualCoordinate::First)?,
            radial_coordinate_harmonic(first, second, DualCoordinate::Second)?,
        ];
        let l = coordinates[1]
            .scale(&directions[0])?
            .sub(&coordinates[0].scale(&directions[1])?)?;
        let radius = ExactScalar::from_f64(second.radius())?;
        let radius_determinant = radius.mul(&e)?;
        let radius_determinant_squared = radius_determinant.mul(&radius_determinant)?;
        let k = a.mul(&radius_determinant_squared)?;
        Ok(Self {
            cylinders,
            e,
            directions,
            a,
            k,
            l,
        })
    }

    fn discriminant(&self) -> Result<ExactSkewCylinderDiscriminant, SkewCylinderAxialRootFailure> {
        // The extrema shortcut is optional: exact expansion growth may leave
        // its narrow product envelope even when direct harmonic construction
        // remains safe, so preserve the latter as the authoritative fallback.
        if let Ok(Some(admission @ ExactSkewCylinderDiscriminant::Strict(_))) =
            factor_extrema_admission(&self.k, &self.l)
        {
            return Ok(admission);
        }
        let discriminant = ExactSecondHarmonic::constant(self.k.clone())
            .sub(&self.l.square()?)?
            .scale(&ExactScalar::from_f64(4.0)?)?;
        let cardinal_contact = discriminant.cardinal_contact().unwrap_or(false);
        Ok(ExactSkewCylinderDiscriminant::Harmonic {
            coefficients: discriminant.coefficients(),
            cardinal_contact,
        })
    }

    fn axial_equation(
        &self,
        canonical_operand: usize,
        bound: f64,
    ) -> Result<AxialEquation, SkewCylinderAxialRootFailure> {
        let authored_bound = bound;
        let bound = ExactScalar::from_f64(bound)?;
        if canonical_operand == 0
            && let Some(equation) =
                self.semantic_perpendicular_canonical_equation(authored_bound)?
        {
            return Ok(equation);
        }
        let (m, radial_equation) = self.radial_terms()?;
        if canonical_operand == 0 {
            let selector = m.add(&ExactFirstHarmonic::constant(self.a.mul(&bound)?))?;
            let eliminated = radial_equation
                .add(&ExactSecondHarmonic::from_first(
                    m.scale(&bound.scale(2.0)?)?,
                ))?
                .add(&ExactSecondHarmonic::constant(
                    self.a.mul(&bound.mul(&bound)?)?,
                ))?;
            return Ok(AxialEquation::Canonical {
                eliminated,
                selector,
            });
        }
        if canonical_operand != 1 {
            return Err(SkewCylinderAxialRootFailure::InvalidSourceOperand);
        }

        if let Some(height) = self.shared_semantic_opposite_height()? {
            let direct = height.sub(&ExactFirstHarmonic::constant(bound))?;
            return Ok(AxialEquation::SharedOppositeHeight { direct, e_sign: 1 });
        }
        let (dz, z0) = self.opposite_height_terms()?;
        let direct = z0.sub(&ExactFirstHarmonic::constant(self.e.mul(&bound)?))?;
        if dz.is_zero() {
            return Ok(AxialEquation::SharedOppositeHeight {
                direct,
                e_sign: self.e.sign(),
            });
        }
        let selector = direct.scale(&self.a)?.sub(&m.scale(&dz)?)?;
        let dz_squared = dz.mul(&dz)?;
        let eliminated = direct
            .square()?
            .scale(&self.a)?
            .sub(&m.product(&direct)?.scale(&dz.scale(2.0)?)?)?
            .add(&radial_equation.scale(&dz_squared)?)?;
        Ok(AxialEquation::Opposite {
            eliminated,
            selector,
            dz_sign: dz.sign(),
            e_sign: self.e.sign(),
        })
    }

    fn opposite_height_terms(
        &self,
    ) -> Result<(ExactScalar, ExactFirstHarmonic), SkewCylinderAxialRootFailure> {
        let [first, second] = self.cylinders;
        let second_axes = [
            exact_vector(second.frame().x().to_array())?,
            exact_vector(second.frame().y().to_array())?,
            exact_vector(second.frame().z().to_array())?,
        ];
        let dz = exact_determinant_expansion(
            second_axes[0].clone(),
            second_axes[1].clone(),
            exact_vector(first.frame().z().to_array())?,
        )?;
        let z0 = radial_coordinate_harmonic(first, second, DualCoordinate::Third)?;
        Ok((dz, z0))
    }

    /// Return the common sheet height implied by exactly perpendicular
    /// authored axes under `Frame`'s semantic orthonormal-basis contract.
    ///
    /// A frame's stored companion axis comes from rounded cross products, so
    /// the dual determinant can contain a spurious sub-ulp ruling-height
    /// coefficient even when the two authored axes have an exactly zero dot
    /// product. In that family both sheets share the ordinary axial dot
    /// coordinate and no quadratic elimination is necessary.
    fn shared_semantic_opposite_height(
        &self,
    ) -> Result<Option<ExactFirstHarmonic>, SkewCylinderAxialRootFailure> {
        let [first, second] = self.cylinders;
        let axis = second.frame().z();
        // Frame-relative coordinates throughout kgeom use this deterministic
        // rounded dot operation. An exact zero in that semantic coordinate is
        // the stable authored perpendicular-axis relation; re-expanding the
        // frame components would resurrect cross-product roundoff that the
        // `Frame` abstraction deliberately excludes.
        if first.frame().z().dot(axis) != 0.0 {
            return Ok(None);
        }
        let offset = first.frame().origin() - second.frame().origin();
        Ok(Some(ExactFirstHarmonic {
            constant: ExactScalar::from_f64(offset.dot(axis))?,
            cosine: ExactScalar::from_f64(first.frame().x().dot(axis) * first.radius())?,
            sine: ExactScalar::from_f64(first.frame().y().dot(axis) * first.radius())?,
        }))
    }

    /// Build the first cylinder's bound equation directly in its semantic
    /// orthonormal coordinates when the authored axes are perpendicular.
    /// This is algebraically the same radial-distance theorem as the dual
    /// formula, but it preserves rigid-frame root factors that rounded cross
    /// products in the companion frame would otherwise perturb by one ulp.
    fn semantic_perpendicular_canonical_equation(
        &self,
        bound: f64,
    ) -> Result<Option<AxialEquation>, SkewCylinderAxialRootFailure> {
        let [first, second] = self.cylinders;
        if first.frame().z().dot(second.frame().z()) != 0.0 {
            return Ok(None);
        }
        let offset = first.frame().origin() - second.frame().origin() + first.frame().z() * bound;
        let radius = first.radius();
        let axes = [first.frame().x(), first.frame().y(), first.frame().z()];
        let coordinates: [Result<ExactFirstHarmonic, RootIsolationFailure>; 3] = axes.map(|axis| {
            Ok(ExactFirstHarmonic {
                constant: ExactScalar::from_f64(offset.dot(axis))?,
                cosine: ExactScalar::from_f64(first.frame().x().dot(axis) * radius)?,
                sine: ExactScalar::from_f64(first.frame().y().dot(axis) * radius)?,
            })
        });
        let [x, y, z] = coordinates;
        let [x, y, z] = [x?, y?, z?];
        let second_axis = second.frame().z();
        let second_height = ExactFirstHarmonic {
            constant: ExactScalar::from_f64(offset.dot(second_axis))?,
            cosine: ExactScalar::from_f64(first.frame().x().dot(second_axis) * radius)?,
            sine: ExactScalar::from_f64(first.frame().y().dot(second_axis) * radius)?,
        };
        let radius_squared = ExactScalar::from_f64(second.radius())?
            .mul(&ExactScalar::from_f64(second.radius())?)?;
        let eliminated = x
            .square()?
            .add(&y.square()?)?
            .add(&z.square()?)?
            .sub(&second_height.square()?)?
            .sub(&ExactSecondHarmonic::constant(radius_squared))?;
        Ok(Some(AxialEquation::Canonical {
            eliminated,
            selector: z,
        }))
    }

    fn radial_terms(
        &self,
    ) -> Result<(ExactFirstHarmonic, ExactSecondHarmonic), SkewCylinderAxialRootFailure> {
        let [first, second] = self.cylinders;
        let coordinates = [
            radial_coordinate_harmonic(first, second, DualCoordinate::First)?,
            radial_coordinate_harmonic(first, second, DualCoordinate::Second)?,
        ];
        let m = coordinates[0]
            .scale(&self.directions[0])?
            .add(&coordinates[1].scale(&self.directions[1])?)?;
        let radius_determinant = ExactScalar::from_f64(second.radius())?.mul(&self.e)?;
        let radial_equation = coordinates[0]
            .square()?
            .add(&coordinates[1].square()?)?
            .sub(&ExactSecondHarmonic::constant(
                radius_determinant.mul(&radius_determinant)?,
            ))?;
        Ok((m, radial_equation))
    }
}

fn factor_extrema_admission(
    k: &ExactScalar,
    harmonic: &ExactFirstHarmonic,
) -> Result<Option<ExactSkewCylinderDiscriminant>, RootIsolationFailure> {
    if k.sign() <= 0 {
        return Ok(None);
    }
    let c2 = harmonic.constant.mul(&harmonic.constant)?;
    let r2 = harmonic
        .cosine
        .mul(&harmonic.cosine)?
        .add(&harmonic.sine.mul(&harmonic.sine)?)?;
    let p = c2.mul(&r2)?.scale(4.0)?;

    let t = k.sub(&c2)?.sub(&r2)?;
    if t.sign() > 0 && t.mul(&t)?.sub(&p)?.sign() > 0 {
        return Ok(Some(ExactSkewCylinderDiscriminant::Strict(
            StrictSign::Positive,
        )));
    }

    let u = c2.add(&r2)?.sub(k)?;
    if c2.sub(&r2)?.sign() > 0 && u.sign() > 0 && u.mul(&u)?.sub(&p)?.sign() > 0 {
        return Ok(Some(ExactSkewCylinderDiscriminant::Strict(
            StrictSign::Negative,
        )));
    }
    Ok(Some(ExactSkewCylinderDiscriminant::Contact))
}

#[derive(Debug, Clone, Copy)]
enum DualCoordinate {
    First,
    Second,
    Third,
}

fn radial_coordinate_harmonic(
    first: Cylinder,
    second: Cylinder,
    coordinate: DualCoordinate,
) -> Result<ExactFirstHarmonic, RootIsolationFailure> {
    let radius = ExactScalar::from_f64(first.radius())?;
    let first_axes = [
        exact_vector(first.frame().x().to_array())?,
        exact_vector(first.frame().y().to_array())?,
    ];
    let second_axes = [
        exact_vector(second.frame().x().to_array())?,
        exact_vector(second.frame().y().to_array())?,
        exact_vector(second.frame().z().to_array())?,
    ];
    let offset = exact_vector_difference(
        first.frame().origin().to_array(),
        second.frame().origin().to_array(),
    )?;
    let determinant = |vector: [ExactScalar; 3]| match coordinate {
        DualCoordinate::First => {
            exact_determinant_expansion(vector, second_axes[1].clone(), second_axes[2].clone())
        }
        DualCoordinate::Second => {
            exact_determinant_expansion(second_axes[0].clone(), vector, second_axes[2].clone())
        }
        DualCoordinate::Third => {
            exact_determinant_expansion(second_axes[0].clone(), second_axes[1].clone(), vector)
        }
    };
    Ok(ExactFirstHarmonic {
        constant: determinant(offset)?,
        cosine: determinant(first_axes[0].clone())?.mul(&radius)?,
        sine: determinant(first_axes[1].clone())?.mul(&radius)?,
    })
}

fn exact_vector(vector: [f64; 3]) -> Result<[ExactScalar; 3], RootIsolationFailure> {
    Ok([
        ExactScalar::from_f64(vector[0])?,
        ExactScalar::from_f64(vector[1])?,
        ExactScalar::from_f64(vector[2])?,
    ])
}

fn exact_vector_difference(
    point: [f64; 3],
    origin: [f64; 3],
) -> Result<[ExactScalar; 3], RootIsolationFailure> {
    Ok([
        ExactScalar::from_f64(point[0])?.sub(&ExactScalar::from_f64(origin[0])?)?,
        ExactScalar::from_f64(point[1])?.sub(&ExactScalar::from_f64(origin[1])?)?,
        ExactScalar::from_f64(point[2])?.sub(&ExactScalar::from_f64(origin[2])?)?,
    ])
}

fn exact_determinant_expansion(
    first: [ExactScalar; 3],
    second: [ExactScalar; 3],
    third: [ExactScalar; 3],
) -> Result<ExactScalar, RootIsolationFailure> {
    let minor = |a: usize, b: usize, c: usize, d: usize| {
        second[a].mul(&third[b])?.sub(&second[c].mul(&third[d])?)
    };
    first[0]
        .mul(&minor(1, 2, 2, 1)?)?
        .sub(&first[1].mul(&minor(0, 2, 2, 0)?)?)?
        .add(&first[2].mul(&minor(0, 1, 1, 0)?)?)
}

#[cfg(test)]
#[path = "skew_cylinder_axial_bound_tests.rs"]
mod tests;
