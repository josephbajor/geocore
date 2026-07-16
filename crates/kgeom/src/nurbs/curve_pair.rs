//! Source-provenanced adaptive isolation for NURBS curve pairs.

use super::NurbsCurve;
use crate::aabb::Aabb3;
use crate::curve::Curve;
use crate::param::ParamRange;
use kcore::error::{Error, Result};
use kcore::expansion;
use kcore::interval::Interval;
use kcore::operation::{
    AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationPolicyError, OperationScope,
    ResourceKind, StageId,
};
use kcore::predicates::{Orientation, orient2d, orient3d};
use std::sync::Arc;

const DEFAULT_DEPTH: u32 = 6;
const DEFAULT_CANDIDATES: u64 = 4_096;
const DEFAULT_SUBDIVISIONS: u64 = 6_828;
const MIN_ALGEBRAIC_FORM_COEFFICIENT: u8 = 6;
const DEFAULT_ALGEBRAIC_FORM_COEFFICIENT: u8 = 12;
const MAX_ALGEBRAIC_FORM_COEFFICIENT: u8 = 14;

const fn stage(value: &'static str) -> StageId {
    match StageId::new(value) {
        Ok(stage) => stage,
        Err(_) => panic!("invalid NURBS curve-pair stage"),
    }
}

/// Cumulative curve-pair setup and deterministic subdivision attempts.
pub const NURBS_CURVE_PAIR_SUBDIVISIONS: StageId = stage("kgeom.nurbs.curve-pair-subdivisions");
/// High-water retained conservative curve-pair candidate cells.
pub const NURBS_CURVE_PAIR_CANDIDATES: StageId = stage("kgeom.nurbs.curve-pair-candidates");
/// High-water binary subdivision depth per curve in a pair cell.
pub const NURBS_CURVE_PAIR_DEPTH: StageId = stage("kgeom.nurbs.curve-pair-depth");

/// Version-1 bounded profile for source-provenanced NURBS curve-pair isolation.
#[derive(Debug, Clone, Copy, Default)]
pub struct NurbsCurvePairBudgetProfile;

impl NurbsCurvePairBudgetProfile {
    /// Exact ceilings for one single-span root pair through six four-way
    /// rounds: at most 4,096 retained cells and 6,828 setup, subdivision, and
    /// source-range span-scan charges.
    pub fn v1_defaults() -> BudgetPlan {
        BudgetPlan::new([
            LimitSpec::new(
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                DEFAULT_SUBDIVISIONS,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                DEFAULT_CANDIDATES,
            ),
            LimitSpec::new(
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                u64::from(DEFAULT_DEPTH),
            ),
        ])
        .expect("built-in curve-pair isolation profile is valid")
    }

    /// Exact default subdivision depth.
    pub const fn default_depth() -> u32 {
        DEFAULT_DEPTH
    }

    /// Require all curve-pair stages with their canonical accounting modes.
    pub fn validate(plan: &BudgetPlan) -> core::result::Result<(), OperationPolicyError> {
        plan.require_limit(
            NURBS_CURVE_PAIR_SUBDIVISIONS,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        plan.require_limit(
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
            AccountingMode::HighWater,
        )?;
        plan.require_limit(
            NURBS_CURVE_PAIR_DEPTH,
            ResourceKind::Depth,
            AccountingMode::HighWater,
        )?;
        Ok(())
    }
}

/// One derived subcurve pair whose conservative source-range position
/// enclosures are not certified farther apart than the requested Euclidean
/// tolerance, with shared provenance back to the original source curves.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvePairCandidateCell {
    first: NurbsCurve,
    second: NurbsCurve,
    first_source: Arc<NurbsCurve>,
    second_source: Arc<NurbsCurve>,
    first_bounds: Aabb3,
    second_bounds: Aabb3,
    depth: u32,
}

/// Axis-aligned plane used by an exact unique-root certificate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum CurvePairProjectionPlane {
    /// Root equations use the x/y projection.
    Xy,
    /// Root equations use the x/z projection.
    Xz,
    /// Root equations use the y/z projection.
    Yz,
}

/// Validated finite search contract for exact algebraic curve-pair lifts.
///
/// The compatibility default searches the complete canonical primitive-
/// integer carrier/residual family through coefficient magnitude twelve. A
/// caller may explicitly opt into the magnitude-thirteen or magnitude-fourteen
/// shell. The supported interval is deliberately narrow: it makes each
/// additional exact search a reviewed, deterministic finite limit rather than
/// an unbounded integer-form enumeration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairAlgebraicSearchConfig {
    maximum_primitive_form_coefficient: u8,
}

impl CurvePairAlgebraicSearchConfig {
    /// Construct a validated primitive-integer coefficient ceiling.
    ///
    /// Magnitudes six through fourteen are accepted. The magnitude-six tier
    /// contains the complete smaller-form prefix; larger values add one
    /// canonical exact-magnitude shell at a time.
    pub const fn new(
        maximum_primitive_form_coefficient: u8,
    ) -> core::result::Result<Self, CurvePairAlgebraicSearchConfigError> {
        if maximum_primitive_form_coefficient < MIN_ALGEBRAIC_FORM_COEFFICIENT
            || maximum_primitive_form_coefficient > MAX_ALGEBRAIC_FORM_COEFFICIENT
        {
            return Err(CurvePairAlgebraicSearchConfigError {
                requested: maximum_primitive_form_coefficient,
            });
        }
        Ok(Self {
            maximum_primitive_form_coefficient,
        })
    }

    /// Compatibility ceiling used by [`Default`].
    pub const fn compatibility_maximum_primitive_form_coefficient() -> u8 {
        DEFAULT_ALGEBRAIC_FORM_COEFFICIENT
    }

    /// Largest reviewed coefficient magnitude accepted by [`Self::new`].
    pub const fn supported_maximum_primitive_form_coefficient() -> u8 {
        MAX_ALGEBRAIC_FORM_COEFFICIENT
    }

    /// Configured inclusive primitive-integer coefficient ceiling.
    pub const fn maximum_primitive_form_coefficient(self) -> u8 {
        self.maximum_primitive_form_coefficient
    }
}

impl Default for CurvePairAlgebraicSearchConfig {
    fn default() -> Self {
        Self {
            maximum_primitive_form_coefficient: DEFAULT_ALGEBRAIC_FORM_COEFFICIENT,
        }
    }
}

/// Rejection of an unreviewed algebraic curve-pair search ceiling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CurvePairAlgebraicSearchConfigError {
    requested: u8,
}

impl CurvePairAlgebraicSearchConfigError {
    /// Rejected coefficient ceiling.
    pub const fn requested(self) -> u8 {
        self.requested
    }

    /// Inclusive supported coefficient-ceiling interval.
    pub const fn supported_range(self) -> core::ops::RangeInclusive<u8> {
        MIN_ALGEBRAIC_FORM_COEFFICIENT..=MAX_ALGEBRAIC_FORM_COEFFICIENT
    }
}

impl core::fmt::Display for CurvePairAlgebraicSearchConfigError {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            formatter,
            "algebraic curve-pair coefficient ceiling {} is outside the supported range {}..={}",
            self.requested, MIN_ALGEBRAIC_FORM_COEFFICIENT, MAX_ALGEBRAIC_FORM_COEFFICIENT,
        )
    }
}

impl std::error::Error for CurvePairAlgebraicSearchConfigError {}

/// Proof that one retained NURBS pair cell contains exactly one transverse root.
///
/// The certificate combines an exact existence witness with a strictly
/// positive interval P-matrix Jacobian for global injectivity on the parameter
/// rectangle. Existence comes either from Poincaré–Miranda face signs plus
/// exact coplanarity, directly from an exact shared 3D parameter corner, or
/// from bounded exact rational source samples or source full-multiplicity
/// knots.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurvePairRootCertificate {
    first_range: ParamRange,
    second_range: ParamRange,
    projection_plane: CurvePairProjectionPlane,
    determinant_lower_bound: f64,
}

impl CurvePairRootCertificate {
    /// Certified first-curve parameter interval.
    pub const fn first_range(self) -> ParamRange {
        self.first_range
    }

    /// Certified second-curve parameter interval.
    pub const fn second_range(self) -> ParamRange {
        self.second_range
    }

    /// Coordinate plane whose difference map is certified injective.
    pub const fn projection_plane(self) -> CurvePairProjectionPlane {
        self.projection_plane
    }

    /// Strict lower bound for the oriented projected Jacobian determinant.
    pub const fn determinant_lower_bound(self) -> f64 {
        self.determinant_lower_bound
    }

    /// Swap the two parameter intervals while preserving the same geometric proof.
    pub const fn swapped(self) -> Self {
        Self {
            first_range: self.second_range,
            second_range: self.first_range,
            projection_plane: self.projection_plane,
            determinant_lower_bound: self.determinant_lower_bound,
        }
    }
}

impl CurvePairCandidateCell {
    /// Exact first subcurve.
    pub const fn first_curve(&self) -> &NurbsCurve {
        &self.first
    }

    /// Exact second subcurve.
    pub const fn second_curve(&self) -> &NurbsCurve {
        &self.second
    }

    /// First source parameter interval.
    pub fn first_range(&self) -> ParamRange {
        self.first.param_range()
    }

    /// Second source parameter interval.
    pub fn second_range(&self) -> ParamRange {
        self.second.param_range()
    }

    /// Conservative first source-range position enclosure.
    pub const fn first_bounds(&self) -> Aabb3 {
        self.first_bounds
    }

    /// Conservative second source-range position enclosure.
    pub const fn second_bounds(&self) -> Aabb3 {
        self.second_bounds
    }

    /// Number of pair-subdivision rounds from the requested root pair.
    pub const fn depth(&self) -> u32 {
        self.depth
    }

    /// Certify one exact unique transverse root in this retained cell.
    ///
    /// Full-source cells may use representation proofs directly. Partial cells
    /// use exact source-curve samples/full-multiplicity knots or source-range
    /// interval face signs, together with a source-range P-matrix bound.
    /// Rounded generated controls never establish a source root. Other
    /// non-coplanar, tangent, singular, and interval-inconclusive cells return
    /// `None` without weakening the cover.
    pub fn certify_unique_root(&self) -> Option<CurvePairRootCertificate> {
        self.certify_unique_root_with_config(CurvePairAlgebraicSearchConfig::default())
    }

    /// Certify one exact root under an explicit algebraic-search ceiling.
    ///
    /// This is the candidate-cell counterpart of
    /// [`certify_curve_pair_unique_root_with_config`]. The cell retains its
    /// original source representations; rounded subdivision controls never
    /// participate in the algebraic proof.
    pub fn certify_unique_root_with_config(
        &self,
        algebraic_search: CurvePairAlgebraicSearchConfig,
    ) -> Option<CurvePairRootCertificate> {
        if self.first_source.degree() == 0 || self.second_source.degree() == 0 {
            return None;
        }
        let first_range = self.first_range();
        let second_range = self.second_range();
        if first_range == self.first_source.param_range()
            && second_range == self.second_source.param_range()
        {
            return certify_full_source_pair_with_config(
                &self.first_source,
                &self.second_source,
                first_range,
                second_range,
                algebraic_search,
            );
        }
        certify_partial_source_pair_with_config(
            &self.first_source,
            first_range,
            &self.second_source,
            second_range,
            algebraic_search,
        )
    }
}

fn certify_full_source_pair_with_config(
    first: &NurbsCurve,
    second: &NurbsCurve,
    first_range: ParamRange,
    second_range: ParamRange,
    algebraic_search: CurvePairAlgebraicSearchConfig,
) -> Option<CurvePairRootCertificate> {
    let certificate = |projection_plane, determinant_lower_bound| CurvePairRootCertificate {
        first_range,
        second_range,
        projection_plane,
        determinant_lower_bound,
    };
    if let Some((projection_plane, determinant_lower_bound)) =
        super::spatial_interior_root::certify_spatial_interior_root(first, second)
    {
        return Some(certificate(projection_plane, determinant_lower_bound));
    }
    if let Some((projection_plane, determinant_lower_bound)) =
        super::spatial_curve_pair::certify_spatial_common_corner(first, second)
    {
        return Some(certificate(projection_plane, determinant_lower_bound));
    }
    if let Some(first_midpoint) = strict_midpoint(first_range)
        && let Some(second_midpoint) = strict_midpoint(second_range)
        && source_samples_equal(first, first_midpoint, second, second_midpoint)
        && let Some((projection_plane, determinant_lower_bound)) =
            super::spatial_curve_pair::certify_injective_projection(first, second)
    {
        return Some(certificate(projection_plane, determinant_lower_bound));
    }
    if let Some((projection_plane, determinant_lower_bound)) =
        super::spatial_algebraic_correspondence::certify_algebraic_spatial_root(
            first,
            first_range,
            second,
            second_range,
            algebraic_search,
        )
    {
        return Some(certificate(projection_plane, determinant_lower_bound));
    }
    let (plane, axes) = exact_common_plane_projection(first, second)?;
    for axes in [axes, [axes[1], axes[0]]] {
        if let Some(determinant_lower_bound) = certify_projected_unique_root(first, second, axes) {
            return Some(certificate(plane, determinant_lower_bound));
        }
    }
    None
}

fn certify_partial_source_pair_with_config(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    algebraic_search: CurvePairAlgebraicSearchConfig,
) -> Option<CurvePairRootCertificate> {
    if let Some((projection_plane, determinant_lower_bound)) =
        super::spatial_curve_pair::certify_injective_projection_in_ranges(
            first,
            first_range,
            second,
            second_range,
        )
    {
        let certificate = || CurvePairRootCertificate {
            first_range,
            second_range,
            projection_plane,
            determinant_lower_bound,
        };
        let first_parameters = range_witness_parameters(first_range);
        let second_parameters = range_witness_parameters(second_range);
        for &first_parameter in &first_parameters {
            for &second_parameter in &second_parameters {
                if source_samples_equal(first, first_parameter, second, second_parameter) {
                    return Some(certificate());
                }
            }
        }
        if super::spatial_interior_root::exact_interior_witness_in_ranges(
            first,
            first_range,
            second,
            second_range,
        )
        .is_some()
        {
            return Some(certificate());
        }
    }

    if let Some((projection_plane, determinant_lower_bound)) =
        super::spatial_algebraic_correspondence::certify_algebraic_spatial_root(
            first,
            first_range,
            second,
            second_range,
            algebraic_search,
        )
    {
        return Some(CurvePairRootCertificate {
            first_range,
            second_range,
            projection_plane,
            determinant_lower_bound,
        });
    }

    let (plane, axes) = exact_common_plane_projection(first, second)?;
    for axes in [axes, [axes[1], axes[0]]] {
        if let Some(determinant_lower_bound) =
            certify_projected_unique_root_in_ranges(first, first_range, second, second_range, axes)
        {
            return Some(CurvePairRootCertificate {
                first_range,
                second_range,
                projection_plane: plane,
                determinant_lower_bound,
            });
        }
    }
    None
}

fn range_witness_parameters(range: ParamRange) -> Vec<f64> {
    let mut parameters = Vec::with_capacity(3);
    if let Some(midpoint) = strict_midpoint(range) {
        parameters.push(midpoint);
    }
    for endpoint in [range.lo, range.hi] {
        if !parameters.contains(&endpoint) {
            parameters.push(endpoint);
        }
    }
    parameters
}

fn source_samples_equal(
    first: &NurbsCurve,
    first_parameter: f64,
    second: &NurbsCurve,
    second_parameter: f64,
) -> bool {
    let Some(witness) = super::spatial_exact_sample::certify_exact_spatial_sample(
        first,
        first_parameter,
        second,
        second_parameter,
    ) else {
        return false;
    };
    debug_assert_eq!(witness.first_parameter(), first_parameter);
    debug_assert_eq!(witness.second_parameter(), second_parameter);
    true
}

fn strict_midpoint(range: ParamRange) -> Option<f64> {
    let width = range.hi - range.lo;
    let midpoint = if width.is_finite() {
        range.lo + width / 2.0
    } else {
        range.lo / 2.0 + range.hi / 2.0
    };
    (midpoint.is_finite() && range.lo < midpoint && midpoint < range.hi).then_some(midpoint)
}

/// Certify one exact unique transverse root over caller-supplied parameter
/// ranges.
///
/// This range-level entry supports proof regions formed by joining adjacent
/// isolation cells around a shared parameter boundary. Partial ranges use only
/// exact evaluations or outward interval bounds of the original source
/// representations. It returns `None` when the proof is inconclusive and never
/// treats a failed proof as an empty-domain certificate.
pub fn certify_curve_pair_unique_root(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
) -> Result<Option<CurvePairRootCertificate>> {
    certify_curve_pair_unique_root_with_config(
        first,
        first_range,
        second,
        second_range,
        CurvePairAlgebraicSearchConfig::default(),
    )
}

/// Certify one exact unique transverse root under an explicit algebraic-search
/// ceiling.
///
/// All non-algebraic certificate families are unchanged. The configuration
/// only controls the inclusive coefficient magnitude reached by the canonical
/// primitive-integer carrier/residual enumeration. Unsupported, broken, or
/// arithmetic-inconclusive forms still fail closed.
pub fn certify_curve_pair_unique_root_with_config(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    algebraic_search: CurvePairAlgebraicSearchConfig,
) -> Result<Option<CurvePairRootCertificate>> {
    validate_inputs(first, first_range, second, second_range, 0.0)?;
    if !first.knots().is_clamped() || !second.knots().is_clamped() {
        return Err(Error::InvalidGeometry {
            reason: "curve-pair root certification requires clamped NURBS curves",
        });
    }
    if first_range == first.param_range() && second_range == second.param_range() {
        Ok(certify_full_source_pair_with_config(
            first,
            second,
            first_range,
            second_range,
            algebraic_search,
        ))
    } else {
        Ok(certify_partial_source_pair_with_config(
            first,
            first_range,
            second,
            second_range,
            algebraic_search,
        ))
    }
}

fn certify_projected_unique_root(
    first: &NurbsCurve,
    second: &NurbsCurve,
    axes: [usize; 2],
) -> Option<f64> {
    let (second_axis_min, second_axis_max) = control_component_bounds(second, axes[0]);
    let first_points = first.points();
    let second_points = second.points();
    let first_face_sign = face_orientation(
        exact_difference_sign(
            component_value(first_points.first().copied()?, axes[0]),
            second_axis_max,
        ),
        exact_difference_sign(
            component_value(first_points.first().copied()?, axes[0]),
            second_axis_min,
        ),
        exact_difference_sign(
            component_value(first_points.last().copied()?, axes[0]),
            second_axis_max,
        ),
        exact_difference_sign(
            component_value(first_points.last().copied()?, axes[0]),
            second_axis_min,
        ),
    );

    let (first_other_min, first_other_max) = control_component_bounds(first, axes[1]);
    let second_low = component_value(second_points.first().copied()?, axes[1]);
    let second_high = component_value(second_points.last().copied()?, axes[1]);
    let second_face_sign = face_orientation(
        exact_difference_sign(first_other_min, second_low),
        exact_difference_sign(first_other_max, second_low),
        exact_difference_sign(first_other_min, second_high),
        exact_difference_sign(first_other_max, second_high),
    );

    if let (Some(first_face_sign), Some(second_face_sign)) = (first_face_sign, second_face_sign)
        && let Some(bound) =
            certify_p_matrix(first, second, axes, [first_face_sign, second_face_sign])
    {
        return Some(bound);
    }
    if has_exact_common_corner(first, second) {
        for signs in [[1.0, 1.0], [1.0, -1.0], [-1.0, 1.0], [-1.0, -1.0]] {
            if let Some(bound) = certify_p_matrix(first, second, axes, signs) {
                return Some(bound);
            }
        }
    }
    None
}

pub(super) fn certify_projected_unique_root_in_ranges(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    axes: [usize; 2],
) -> Option<f64> {
    let first_low = super::source_range_interval::position_component_interval(
        first,
        ParamRange::new(first_range.lo, first_range.lo),
        axes[0],
    )? - super::source_range_interval::position_component_interval(
        second,
        second_range,
        axes[0],
    )?;
    let first_high = super::source_range_interval::position_component_interval(
        first,
        ParamRange::new(first_range.hi, first_range.hi),
        axes[0],
    )? - super::source_range_interval::position_component_interval(
        second,
        second_range,
        axes[0],
    )?;
    let second_low =
        super::source_range_interval::position_component_interval(first, first_range, axes[1])?
            - super::source_range_interval::position_component_interval(
                second,
                ParamRange::new(second_range.lo, second_range.lo),
                axes[1],
            )?;
    let second_high =
        super::source_range_interval::position_component_interval(first, first_range, axes[1])?
            - super::source_range_interval::position_component_interval(
                second,
                ParamRange::new(second_range.hi, second_range.hi),
                axes[1],
            )?;
    let signs = [
        interval_face_orientation(first_low, first_high)?,
        interval_face_orientation(second_low, second_high)?,
    ];
    certify_p_matrix_in_ranges(first, first_range, second, second_range, axes, signs)
}

fn interval_face_orientation(low: Interval, high: Interval) -> Option<f64> {
    if low.hi() <= 0.0 && high.lo() >= 0.0 {
        Some(1.0)
    } else if low.lo() >= 0.0 && high.hi() <= 0.0 {
        Some(-1.0)
    } else {
        None
    }
}

pub(super) fn certify_p_matrix(
    first: &NurbsCurve,
    second: &NurbsCurve,
    axes: [usize; 2],
    signs: [f64; 2],
) -> Option<f64> {
    let sign_first = Interval::point(signs[0]);
    let sign_second = Interval::point(signs[1]);
    let j00 = sign_first * derivative_component_interval(first, axes[0])?;
    let j01 = sign_first * -derivative_component_interval(second, axes[0])?;
    let j10 = sign_second * derivative_component_interval(first, axes[1])?;
    let j11 = sign_second * -derivative_component_interval(second, axes[1])?;
    if j00.lo() <= 0.0 || j11.lo() <= 0.0 {
        return None;
    }
    let determinant = j00 * j11 - j01 * j10;
    (determinant.lo() > 0.0).then_some(determinant.lo())
}

pub(super) fn certify_p_matrix_in_ranges(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    axes: [usize; 2],
    signs: [f64; 2],
) -> Option<f64> {
    let sign_first = Interval::point(signs[0]);
    let sign_second = Interval::point(signs[1]);
    let j00 = sign_first
        * super::source_range_interval::derivative_component_interval(first, first_range, axes[0])?;
    let j01 = sign_first
        * -super::source_range_interval::derivative_component_interval(
            second,
            second_range,
            axes[0],
        )?;
    let j10 = sign_second
        * super::source_range_interval::derivative_component_interval(first, first_range, axes[1])?;
    let j11 = sign_second
        * -super::source_range_interval::derivative_component_interval(
            second,
            second_range,
            axes[1],
        )?;
    if j00.lo() <= 0.0 || j11.lo() <= 0.0 {
        return None;
    }
    let determinant = j00 * j11 - j01 * j10;
    (determinant.lo() > 0.0).then_some(determinant.lo())
}

pub(super) fn has_exact_common_corner(first: &NurbsCurve, second: &NurbsCurve) -> bool {
    let first = [first.points().first(), first.points().last()];
    let second = [second.points().first(), second.points().last()];
    first
        .into_iter()
        .flatten()
        .any(|first| second.into_iter().flatten().any(|second| first == second))
}

fn face_orientation(low_min: i8, low_max: i8, high_min: i8, high_max: i8) -> Option<f64> {
    if low_max <= 0 && high_min >= 0 {
        Some(1.0)
    } else if low_min >= 0 && high_max <= 0 {
        Some(-1.0)
    } else {
        None
    }
}

fn exact_difference_sign(first: f64, second: f64) -> i8 {
    let (rounded, residue) = expansion::two_diff(first, second);
    expansion::sign(&expansion::from_two(rounded, residue))
}

fn exact_common_plane_projection(
    first: &NurbsCurve,
    second: &NurbsCurve,
) -> Option<(CurvePairProjectionPlane, [usize; 2])> {
    let first_count = first.points().len();
    let point_count = first_count + second.points().len();
    let point = |index: usize| {
        if index < first_count {
            first.points()[index]
        } else {
            second.points()[index - first_count]
        }
    };
    for (plane, axes) in [
        (CurvePairProjectionPlane::Xy, [0, 1]),
        (CurvePairProjectionPlane::Xz, [0, 2]),
        (CurvePairProjectionPlane::Yz, [1, 2]),
    ] {
        for first_index in 0..point_count.saturating_sub(2) {
            for second_index in first_index + 1..point_count.saturating_sub(1) {
                for third_index in second_index + 1..point_count {
                    let a = point(first_index);
                    let b = point(second_index);
                    let c = point(third_index);
                    if orient2d(
                        projected_point(a, axes),
                        projected_point(b, axes),
                        projected_point(c, axes),
                    ) == Orientation::Zero
                    {
                        continue;
                    }
                    let a = point_coordinates(a);
                    let b = point_coordinates(b);
                    let c = point_coordinates(c);
                    let coplanar = (0..point_count).all(|index| {
                        orient3d(a, b, c, point_coordinates(point(index))) == Orientation::Zero
                    });
                    return coplanar.then_some((plane, axes));
                }
            }
        }
    }
    None
}

fn projected_point(point: crate::vec::Point3, axes: [usize; 2]) -> [f64; 2] {
    [
        component_value(point, axes[0]),
        component_value(point, axes[1]),
    ]
}

fn point_coordinates(point: crate::vec::Point3) -> [f64; 3] {
    [point.x, point.y, point.z]
}

fn control_component_bounds(curve: &NurbsCurve, axis: usize) -> (f64, f64) {
    let mut values = curve
        .points()
        .iter()
        .map(|point| component_value(*point, axis));
    let first = values.next().expect("validated NURBS has control points");
    let (lo, hi) = values.fold((first, first), |(lo, hi), value| {
        (lo.min(value), hi.max(value))
    });
    (lo, hi)
}

fn derivative_component_interval(curve: &NurbsCurve, axis: usize) -> Option<Interval> {
    let Some(weights) = curve.weights() else {
        return polynomial_derivative_component_interval(curve, axis);
    };
    let coordinate = homogeneous_control_interval(curve, axis, weights);
    let weight = scalar_control_interval(weights);
    let coordinate_derivative = homogeneous_derivative_component_interval(curve, axis, weights)?;
    let weight_derivative = scalar_derivative_interval(curve, weights)?;
    let numerator = coordinate_derivative * weight - coordinate * weight_derivative;
    numerator.checked_div(weight * weight)
}

fn polynomial_derivative_component_interval(curve: &NurbsCurve, axis: usize) -> Option<Interval> {
    derivative_control_interval(curve, |index| {
        Interval::point(component_value(curve.points()[index], axis))
    })
}

fn homogeneous_control_interval(curve: &NurbsCurve, axis: usize, weights: &[f64]) -> Interval {
    hull_intervals(curve.points().iter().zip(weights).map(|(point, weight)| {
        Interval::point(component_value(*point, axis)) * Interval::point(*weight)
    }))
}

fn scalar_control_interval(values: &[f64]) -> Interval {
    hull_intervals(values.iter().map(|value| Interval::point(*value)))
}

fn homogeneous_derivative_component_interval(
    curve: &NurbsCurve,
    axis: usize,
    weights: &[f64],
) -> Option<Interval> {
    derivative_control_interval(curve, |index| {
        Interval::point(component_value(curve.points()[index], axis))
            * Interval::point(weights[index])
    })
}

fn scalar_derivative_interval(curve: &NurbsCurve, values: &[f64]) -> Option<Interval> {
    derivative_control_interval(curve, |index| Interval::point(values[index]))
}

fn derivative_control_interval(
    curve: &NurbsCurve,
    value: impl Fn(usize) -> Interval,
) -> Option<Interval> {
    let degree = curve.degree();
    let knots = curve.knots().as_slice();
    let mut hull: Option<Interval> = None;
    for index in 0..curve.points().len().checked_sub(1)? {
        let numerator = Interval::point(degree as f64) * (value(index + 1) - value(index));
        let denominator =
            Interval::point(knots[index + degree + 1]) - Interval::point(knots[index + 1]);
        let derivative = numerator.checked_div(denominator)?;
        hull = Some(match hull {
            Some(current) => Interval::new(
                current.lo().min(derivative.lo()),
                current.hi().max(derivative.hi()),
            ),
            None => derivative,
        });
    }
    hull
}

fn hull_intervals(values: impl IntoIterator<Item = Interval>) -> Interval {
    values
        .into_iter()
        .reduce(|current, value| {
            Interval::new(current.lo().min(value.lo()), current.hi().max(value.hi()))
        })
        .expect("validated NURBS has control values")
}

fn component_value(point: crate::vec::Point3, axis: usize) -> f64 {
    match axis {
        0 => point.x,
        1 => point.y,
        2 => point.z,
        _ => unreachable!("3D coordinate axis"),
    }
}

/// Structured reasons a conservative pair cover stopped early.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CurvePairIsolationLimits {
    subdivision_work: Option<LimitSnapshot>,
    candidate_cells: Option<LimitSnapshot>,
    subdivision_depth: Option<LimitSnapshot>,
    parameter_resolution: bool,
    subdivision_unavailable: bool,
}

impl CurvePairIsolationLimits {
    /// Exact subdivision-work crossing, if reached.
    pub const fn subdivision_work(self) -> Option<LimitSnapshot> {
        self.subdivision_work
    }

    /// Exact retained-candidate crossing, if reached.
    pub const fn candidate_cells(self) -> Option<LimitSnapshot> {
        self.candidate_cells
    }

    /// Exact subdivision-depth crossing, if reached.
    pub const fn subdivision_depth(self) -> Option<LimitSnapshot> {
        self.subdivision_depth
    }

    /// Whether a mathematical midpoint rounded to an existing endpoint.
    pub const fn parameter_resolution(self) -> bool {
        self.parameter_resolution
    }

    /// Whether a valid degree-zero curve prevented binary subdivision.
    pub const fn subdivision_unavailable(self) -> bool {
        self.subdivision_unavailable
    }

    /// True when no configured, numeric, or method stop occurred.
    pub const fn is_empty(self) -> bool {
        self.subdivision_work.is_none()
            && self.candidate_cells.is_none()
            && self.subdivision_depth.is_none()
            && !self.parameter_resolution
            && !self.subdivision_unavailable
    }
}

/// Conservative cover of every possible tolerance-level NURBS curve contact.
#[derive(Debug, Clone, PartialEq)]
pub struct CurvePairIsolation {
    candidates: Vec<CurvePairCandidateCell>,
    requested_depth: u32,
    limits: CurvePairIsolationLimits,
}

impl CurvePairIsolation {
    /// Retained cells in deterministic first-range/second-range order.
    pub fn candidates(&self) -> &[CurvePairCandidateCell] {
        &self.candidates
    }

    /// Requested pair-subdivision depth.
    pub const fn requested_depth(&self) -> u32 {
        self.requested_depth
    }

    /// Structured early-stop evidence.
    pub const fn limits(&self) -> CurvePairIsolationLimits {
        self.limits
    }

    /// True when every retained cell reached the requested depth.
    pub fn is_complete(&self) -> bool {
        self.limits.is_empty()
            && self
                .candidates
                .iter()
                .all(|candidate| candidate.depth == self.requested_depth)
    }

    /// True only when complete source-range enclosure pruning excluded every pair.
    pub fn is_proven_empty(&self) -> bool {
        self.is_complete() && self.candidates.is_empty()
    }
}

/// Failure to construct or account a contextual curve-pair cover.
#[derive(Debug, Clone, PartialEq)]
pub enum ContextCurvePairIsolationError {
    /// Invalid geometry or exact NURBS processing failure.
    Kernel(Error),
    /// Invalid or exhausted operation policy before a conservative cover exists.
    Policy(OperationPolicyError),
}

impl From<Error> for ContextCurvePairIsolationError {
    fn from(error: Error) -> Self {
        Self::Kernel(error)
    }
}

impl From<OperationPolicyError> for ContextCurvePairIsolationError {
    fn from(error: OperationPolicyError) -> Self {
        Self::Policy(error)
    }
}

#[derive(Debug)]
struct WorkCell {
    cell: CurvePairCandidateCell,
    blocked: bool,
}

/// Isolate a conservative source-range cover of every possible contact.
pub fn isolate_curve_pair_candidates_in_scope(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    margin: f64,
    requested_depth: u32,
    scope: &mut OperationScope<'_, '_>,
) -> core::result::Result<CurvePairIsolation, ContextCurvePairIsolationError> {
    validate_inputs(first, first_range, second, second_range, margin)?;
    require_profile(scope)?;
    let initial_work = source_range_work_units(first, second, 1)?;
    scope
        .ledger_mut()
        .charge(NURBS_CURVE_PAIR_SUBDIVISIONS, initial_work)?;

    let first_source = Arc::new(first.clone());
    let second_source = Arc::new(second.clone());
    let first = first_source.restricted_to(first_range)?;
    let second = second_source.restricted_to(second_range)?;
    let mut cells = initial_cells(first, second, first_source, second_source, margin);
    let subdivision_unavailable = requested_depth > 0
        && !cells.is_empty()
        && (cells[0].cell.first.degree() == 0 || cells[0].cell.second.degree() == 0);
    let mut limits = CurvePairIsolationLimits {
        subdivision_unavailable,
        ..CurvePairIsolationLimits::default()
    };
    if subdivision_unavailable {
        return Ok(isolation_result(cells, requested_depth, limits));
    }
    if observe_limit(
        scope,
        NURBS_CURVE_PAIR_CANDIDATES,
        ResourceKind::Items,
        usize_to_u64(
            cells.len(),
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
        )?,
    )?
    .is_some_and(|snapshot| {
        limits.candidate_cells = Some(snapshot);
        true
    }) {
        return Ok(isolation_result(cells, requested_depth, limits));
    }
    if observe_limit(scope, NURBS_CURVE_PAIR_DEPTH, ResourceKind::Depth, 0)?.is_some_and(
        |snapshot| {
            limits.subdivision_depth = Some(snapshot);
            true
        },
    ) {
        return Ok(isolation_result(cells, requested_depth, limits));
    }

    for _ in 0..requested_depth {
        if cells.is_empty() || cells.iter().all(|work| work.blocked) {
            break;
        }
        let previous = core::mem::take(&mut cells);
        let previous_len = previous.len();
        let mut next = Vec::with_capacity(previous_len.saturating_mul(4));
        for (position, mut work) in previous.into_iter().enumerate() {
            if work.blocked
                || limits.subdivision_work.is_some()
                || limits.candidate_cells.is_some()
                || limits.subdivision_depth.is_some()
            {
                work.blocked = true;
                next.push(work);
                continue;
            }
            let attempted_depth = u64::from(work.cell.depth).saturating_add(1);
            if let Some(snapshot) = observe_limit(
                scope,
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                attempted_depth,
            )? {
                limits.subdivision_depth = Some(snapshot);
                work.blocked = true;
                next.push(work);
                continue;
            }
            let subdivision_work =
                source_range_work_units(&work.cell.first_source, &work.cell.second_source, 2)?;
            match scope
                .ledger_mut()
                .charge(NURBS_CURVE_PAIR_SUBDIVISIONS, subdivision_work)
            {
                Ok(()) => {}
                Err(OperationPolicyError::LimitReached(snapshot)) => {
                    limits.subdivision_work = Some(snapshot);
                    work.blocked = true;
                    next.push(work);
                    continue;
                }
                Err(error) => return Err(error.into()),
            }
            let Some(children) = split_children(&work.cell, margin)? else {
                limits.parameter_resolution = true;
                scope.record_numeric_resolution(NURBS_CURVE_PAIR_DEPTH);
                work.blocked = true;
                next.push(work);
                continue;
            };
            let remaining = previous_len - position - 1;
            let attempted = next
                .len()
                .checked_add(children.len())
                .and_then(|count| count.checked_add(remaining))
                .ok_or(OperationPolicyError::AccountingOverflow {
                    stage: NURBS_CURVE_PAIR_CANDIDATES,
                    resource: ResourceKind::Items,
                })?;
            if let Some(snapshot) = observe_limit(
                scope,
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                usize_to_u64(attempted, NURBS_CURVE_PAIR_CANDIDATES, ResourceKind::Items)?,
            )? {
                limits.candidate_cells = Some(snapshot);
                work.blocked = true;
                next.push(work);
            } else {
                next.extend(children.into_iter().map(|cell| WorkCell {
                    cell,
                    blocked: false,
                }));
            }
        }
        cells = next;
    }
    Ok(isolation_result(cells, requested_depth, limits))
}

fn validate_inputs(
    first: &NurbsCurve,
    first_range: ParamRange,
    second: &NurbsCurve,
    second_range: ParamRange,
    margin: f64,
) -> Result<()> {
    if !margin.is_finite() || margin < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "curve-pair isolation margin must be finite and nonnegative",
        });
    }
    for (curve, range) in [(first, first_range), (second, second_range)] {
        let domain = curve.param_range();
        if !range.is_finite()
            || range.width() <= 0.0
            || range.lo < domain.lo
            || range.hi > domain.hi
        {
            return Err(Error::InvalidGeometry {
                reason: "curve-pair isolation ranges must be finite, positive, and inside the curve domains",
            });
        }
    }
    Ok(())
}

fn require_profile(
    scope: &OperationScope<'_, '_>,
) -> core::result::Result<(), OperationPolicyError> {
    NurbsCurvePairBudgetProfile::validate(&scope.context().effective_budget())
}

fn initial_cells(
    first: NurbsCurve,
    second: NurbsCurve,
    first_source: Arc<NurbsCurve>,
    second_source: Arc<NurbsCurve>,
    margin: f64,
) -> Vec<WorkCell> {
    candidate_cell_with_sources(first, second, first_source, second_source, 0, margin)
        .map(|cell| {
            vec![WorkCell {
                cell,
                blocked: false,
            }]
        })
        .unwrap_or_default()
}

#[cfg(test)]
fn candidate_cell(
    first: NurbsCurve,
    second: NurbsCurve,
    depth: u32,
    margin: f64,
) -> Option<CurvePairCandidateCell> {
    let first_source = Arc::new(first.clone());
    let second_source = Arc::new(second.clone());
    candidate_cell_with_sources(first, second, first_source, second_source, depth, margin)
}

fn candidate_cell_with_sources(
    first: NurbsCurve,
    second: NurbsCurve,
    first_source: Arc<NurbsCurve>,
    second_source: Arc<NurbsCurve>,
    depth: u32,
    margin: f64,
) -> Option<CurvePairCandidateCell> {
    let first_bounds =
        super::source_range_interval::position_range_aabb(&first_source, first.param_range());
    let second_bounds =
        super::source_range_interval::position_range_aabb(&second_source, second.param_range());
    candidate_cell_with_source_bounds(
        first,
        second,
        first_source,
        second_source,
        first_bounds,
        second_bounds,
        depth,
        margin,
    )
}

#[allow(clippy::too_many_arguments)]
fn candidate_cell_with_source_bounds(
    first: NurbsCurve,
    second: NurbsCurve,
    first_source: Arc<NurbsCurve>,
    second_source: Arc<NurbsCurve>,
    first_bounds: Aabb3,
    second_bounds: Aabb3,
    depth: u32,
    margin: f64,
) -> Option<CurvePairCandidateCell> {
    let axis_candidate = first_bounds.inflated(margin).intersects(second_bounds);
    let margin_squared = (Interval::point(margin) * Interval::point(margin)).hi();
    (axis_candidate && first_bounds.squared_distance_lower_bound(second_bounds) <= margin_squared)
        .then_some(CurvePairCandidateCell {
            first,
            second,
            first_source,
            second_source,
            first_bounds,
            second_bounds,
            depth,
        })
}

fn split_children(
    parent: &CurvePairCandidateCell,
    margin: f64,
) -> Result<Option<Vec<CurvePairCandidateCell>>> {
    let first_range = parent.first.param_range();
    let second_range = parent.second.param_range();
    let first_mid = first_range.lo + first_range.width() / 2.0;
    let second_mid = second_range.lo + second_range.width() / 2.0;
    if !(first_range.lo < first_mid
        && first_mid < first_range.hi
        && second_range.lo < second_mid
        && second_mid < second_range.hi)
    {
        return Ok(None);
    }
    let (first_low, first_high) = parent.first.split_at(first_mid)?;
    let (second_low, second_high) = parent.second.split_at(second_mid)?;
    let first = [first_low, first_high].map(|curve| {
        let bounds = super::source_range_interval::position_range_aabb(
            &parent.first_source,
            curve.param_range(),
        );
        (curve, bounds)
    });
    let second = [second_low, second_high].map(|curve| {
        let bounds = super::source_range_interval::position_range_aabb(
            &parent.second_source,
            curve.param_range(),
        );
        (curve, bounds)
    });
    let mut children = Vec::with_capacity(4);
    for (first, first_bounds) in first {
        for (second, second_bounds) in &second {
            if let Some(cell) = candidate_cell_with_source_bounds(
                first.clone(),
                second.clone(),
                Arc::clone(&parent.first_source),
                Arc::clone(&parent.second_source),
                first_bounds,
                *second_bounds,
                parent.depth + 1,
                margin,
            ) {
                children.push(cell);
            }
        }
    }
    Ok(Some(children))
}

fn isolation_result(
    mut cells: Vec<WorkCell>,
    requested_depth: u32,
    limits: CurvePairIsolationLimits,
) -> CurvePairIsolation {
    let mut candidates = cells.drain(..).map(|work| work.cell).collect::<Vec<_>>();
    candidates.sort_by(|a, b| {
        a.first_range()
            .lo
            .total_cmp(&b.first_range().lo)
            .then(a.second_range().lo.total_cmp(&b.second_range().lo))
    });
    CurvePairIsolation {
        candidates,
        requested_depth,
        limits,
    }
}

fn observe_limit(
    scope: &mut OperationScope<'_, '_>,
    stage: StageId,
    resource: ResourceKind,
    value: u64,
) -> core::result::Result<Option<LimitSnapshot>, OperationPolicyError> {
    match scope.ledger_mut().observe(stage, resource, value) {
        Ok(()) => Ok(None),
        Err(OperationPolicyError::LimitReached(snapshot)) => Ok(Some(snapshot)),
        Err(error) => Err(error),
    }
}

fn usize_to_u64(
    value: usize,
    stage: StageId,
    resource: ResourceKind,
) -> core::result::Result<u64, OperationPolicyError> {
    u64::try_from(value).map_err(|_| OperationPolicyError::AccountingOverflow { stage, resource })
}

fn source_range_work_units(
    first: &NurbsCurve,
    second: &NurbsCurve,
    evaluations_per_source: u64,
) -> core::result::Result<u64, OperationPolicyError> {
    let first = usize_to_u64(
        super::source_range_interval::position_range_work_units(first),
        NURBS_CURVE_PAIR_SUBDIVISIONS,
        ResourceKind::Work,
    )?;
    let second = usize_to_u64(
        super::source_range_interval::position_range_work_units(second),
        NURBS_CURVE_PAIR_SUBDIVISIONS,
        ResourceKind::Work,
    )?;
    first
        .checked_add(second)
        .and_then(|spans| spans.checked_mul(evaluations_per_source))
        .and_then(|range_work| range_work.checked_add(1))
        .ok_or(OperationPolicyError::AccountingOverflow {
            stage: NURBS_CURVE_PAIR_SUBDIVISIONS,
            resource: ResourceKind::Work,
        })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationOutcome, OperationReport, SessionPolicy};
    use kcore::tolerance::Tolerances;

    use super::*;
    use crate::vec::Point3;

    fn line(y: f64) -> NurbsCurve {
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::new(-1.0, y, 0.0), Point3::new(1.0, y, 0.0)],
            None,
        )
        .unwrap()
    }

    fn arch() -> NurbsCurve {
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(0.0, 2.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap()
    }

    fn segment(start: Point3, end: Point3, weights: Option<Vec<f64>>) -> NurbsCurve {
        NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], weights).unwrap()
    }

    fn run(
        first: &NurbsCurve,
        second: &NurbsCurve,
        depth: u32,
        overrides: BudgetPlan,
    ) -> OperationOutcome<CurvePairIsolation, ContextCurvePairIsolationError> {
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults())
            .with_budget_overrides(overrides);
        let mut scope = OperationScope::new(&context);
        let result = isolate_curve_pair_candidates_in_scope(
            first,
            first.param_range(),
            second,
            second.param_range(),
            Tolerances::default().linear(),
            depth,
            &mut scope,
        );
        scope.finish_typed(result)
    }

    fn usage(report: &OperationReport, stage: StageId, resource: ResourceKind) -> LimitSnapshot {
        *report
            .usage()
            .iter()
            .find(|usage| usage.stage == stage && usage.resource == resource)
            .unwrap()
    }

    #[test]
    fn adaptive_cover_proves_hidden_miss_and_retains_crossing_candidates() {
        let first = arch();
        let separated = line(1.5);
        assert!(
            first
                .bounding_box(first.param_range())
                .intersects(separated.bounding_box(separated.param_range()))
        );
        let miss = run(&first, &separated, 3, BudgetPlan::empty());
        assert!(miss.result().unwrap().is_proven_empty());

        let crossing = line(0.5);
        let first_run = run(&first, &crossing, 3, BudgetPlan::empty());
        let second_run = run(&first, &crossing, 3, BudgetPlan::empty());
        assert_eq!(first_run, second_run);
        let isolation = first_run.result().unwrap();
        assert!(isolation.is_complete());
        assert!(!isolation.candidates().is_empty());
        assert!(
            isolation
                .candidates()
                .iter()
                .all(|candidate| candidate.depth() == 3)
        );
        assert!(isolation.candidates().windows(2).all(|pair| {
            (pair[0].first_range().lo, pair[0].second_range().lo)
                <= (pair[1].first_range().lo, pair[1].second_range().lo)
        }));

        let swapped = run(&crossing, &first, 3, BudgetPlan::empty());
        let swapped = swapped.result().unwrap();
        assert!(swapped.is_complete());
        let forward_ranges = isolation
            .candidates()
            .iter()
            .map(|cell| (cell.first_range(), cell.second_range()))
            .collect::<Vec<_>>();
        let mut swapped_ranges = swapped
            .candidates()
            .iter()
            .map(|cell| (cell.second_range(), cell.first_range()))
            .collect::<Vec<_>>();
        swapped_ranges.sort_by(|a, b| a.0.lo.total_cmp(&b.0.lo).then(a.1.lo.total_cmp(&b.1.lo)));
        assert_eq!(swapped_ranges, forward_ranges);
    }

    #[test]
    fn euclidean_control_hull_distance_excludes_diagonal_gaps_but_keeps_boundary() {
        let constant = |point| segment(point, point, None);
        let origin = constant(Point3::new(0.0, 0.0, 0.0));
        let diagonal = constant(Point3::new(0.75, 0.75, 0.0));
        assert!(
            origin
                .bounding_box(origin.param_range())
                .inflated(1.0)
                .intersects(diagonal.bounding_box(diagonal.param_range()))
        );
        assert!(candidate_cell(origin.clone(), diagonal.clone(), 0, 1.0).is_none());
        assert!(candidate_cell(diagonal, origin.clone(), 0, 1.0).is_none());

        let boundary = constant(Point3::new(3.0, 4.0, 0.0));
        assert!(candidate_cell(origin.clone(), boundary.clone(), 0, 5.0).is_some());
        assert!(candidate_cell(boundary, origin, 0, 5.0).is_some());
    }

    #[test]
    fn rounded_subdivision_hulls_cannot_exclude_an_exact_source_contact() {
        let contact_z = 9_007_199_254_740_991.0;
        let first = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1.0, 0.0, 9_007_199_254_740_360.0),
                Point3::new(-1.0 / 3.0, 0.0, 9_007_199_254_740_978.0),
                Point3::new(1.0 / 3.0, 0.0, 9_007_199_254_741_648.0),
                Point3::new(1.0, 0.0, 9_007_199_254_739_690.0),
            ],
            None,
        )
        .unwrap();
        let second = segment(
            Point3::new(0.0, -1.0, contact_z),
            Point3::new(0.0, 1.0, contact_z),
            None,
        );

        assert!(
            certify_curve_pair_unique_root(
                &first,
                first.param_range(),
                &second,
                second.param_range(),
            )
            .unwrap()
            .is_some(),
            "the original dyadic source representations meet exactly at both midpoints"
        );
        let (first_low, first_high) = first.split_at(0.5).unwrap();
        assert_eq!(first_low.points().last().unwrap().z, contact_z - 1.0);
        assert_eq!(first_high.points().first().unwrap().z, contact_z - 1.0);
        assert!(
            first_low
                .points()
                .iter()
                .chain(first_high.points())
                .all(|point| point.z < contact_z),
            "the rounded child hulls deliberately lose the exact source midpoint"
        );

        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(NurbsCurvePairBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let isolation = isolate_curve_pair_candidates_in_scope(
            &first,
            first.param_range(),
            &second,
            second.param_range(),
            0.0,
            1,
            &mut scope,
        )
        .unwrap();
        let exact_contact = Point3::new(0.0, 0.0, contact_z);

        assert!(isolation.is_complete());
        assert!(!isolation.is_proven_empty());
        assert!(isolation.candidates().iter().any(|candidate| {
            candidate.first_range().contains(0.5)
                && candidate.second_range().contains(0.5)
                && candidate.first_bounds().contains(exact_contact)
                && candidate.second_bounds().contains(exact_contact)
        }));
    }

    #[test]
    fn interval_certificate_proves_unique_coplanar_polynomial_and_rational_roots() {
        let diagonal = segment(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            None,
        );
        let horizontal = line(0.0);
        let cell = candidate_cell(diagonal.clone(), horizontal.clone(), 0, 0.0).unwrap();
        let certificate = cell.certify_unique_root().unwrap();
        assert_eq!(certificate.first_range(), ParamRange::new(0.0, 1.0));
        assert_eq!(certificate.second_range(), ParamRange::new(0.0, 1.0));
        assert_eq!(certificate.projection_plane(), CurvePairProjectionPlane::Xy);
        assert!(certificate.determinant_lower_bound() > 0.0);

        let swapped = candidate_cell(horizontal.clone(), diagonal, 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(swapped.projection_plane(), CurvePairProjectionPlane::Xy);
        assert!(swapped.determinant_lower_bound() > 0.0);

        for parameter_scale in [1.0e-13, 1.0, 1.0e13] {
            for model_scale in [1.0e-6, 1.0, 1.0e2] {
                let diagonal = NurbsCurve::new(
                    1,
                    vec![0.0, 0.0, parameter_scale, parameter_scale],
                    vec![
                        Point3::new(7.0 - model_scale, -3.0 - model_scale, 2.0),
                        Point3::new(7.0 + model_scale, -3.0 + model_scale, 2.0),
                    ],
                    None,
                )
                .unwrap();
                let horizontal = NurbsCurve::new(
                    1,
                    vec![0.0, 0.0, parameter_scale, parameter_scale],
                    vec![
                        Point3::new(7.0 - 2.0 * model_scale, -3.0, 2.0),
                        Point3::new(7.0 + 2.0 * model_scale, -3.0, 2.0),
                    ],
                    None,
                )
                .unwrap();
                let certificate = candidate_cell(diagonal, horizontal, 0, 0.0)
                    .unwrap()
                    .certify_unique_root()
                    .unwrap();
                assert_eq!(
                    certificate.first_range(),
                    ParamRange::new(0.0, parameter_scale)
                );
                assert!(certificate.determinant_lower_bound() > 0.0);
            }
        }

        let rational = segment(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Some(vec![1.0, 1.5]),
        );
        let rational = candidate_cell(rational, horizontal.clone(), 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(rational.projection_plane(), CurvePairProjectionPlane::Xy);
        assert!(rational.determinant_lower_bound() > 0.0);
        for parameter_scale in [1.0e-13, 1.0, 1.0e13] {
            for weight_scale in [1.0e-50, 1.0, 1.0e50] {
                let rational = NurbsCurve::new(
                    1,
                    vec![0.0, 0.0, parameter_scale, parameter_scale],
                    vec![Point3::new(-1.0, -1.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
                    Some(vec![weight_scale, 1.5 * weight_scale]),
                )
                .unwrap();
                let horizontal = NurbsCurve::new(
                    1,
                    vec![0.0, 0.0, parameter_scale, parameter_scale],
                    vec![Point3::new(-2.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
                    None,
                )
                .unwrap();
                let certificate = candidate_cell(rational, horizontal, 0, 0.0)
                    .unwrap()
                    .certify_unique_root()
                    .unwrap();
                assert!(certificate.determinant_lower_bound() > 0.0);
            }
        }

        let two_roots = candidate_cell(arch(), line(0.5), 0, 0.0).unwrap();
        assert!(two_roots.certify_unique_root().is_none());

        let spatial = segment(
            Point3::new(0.0, -1.0, -1.0),
            Point3::new(0.0, 1.0, 1.0),
            None,
        );
        let tilted = candidate_cell(horizontal.clone(), spatial.clone(), 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(tilted.projection_plane(), CurvePairProjectionPlane::Xy);
        assert!(tilted.determinant_lower_bound() > 0.0);
        let tilted_swapped = candidate_cell(spatial, horizontal.clone(), 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(
            tilted_swapped.projection_plane(),
            CurvePairProjectionPlane::Xy
        );
        assert!(tilted_swapped.determinant_lower_bound() > 0.0);

        let non_coplanar = NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
                Point3::new(1.0, 1.0, 0.0),
            ],
            None,
        )
        .unwrap();
        assert!(
            candidate_cell(horizontal, non_coplanar, 0, 0.0)
                .unwrap()
                .certify_unique_root()
                .is_none()
        );
    }

    #[test]
    fn rational_derivative_hulls_enclose_evaluated_first_derivatives() {
        let curve = segment(
            Point3::new(-2.0, -1.0, 0.5),
            Point3::new(3.0, 4.0, 0.5),
            Some(vec![1.0, 10.0]),
        );
        let intervals = [
            derivative_component_interval(&curve, 0).unwrap(),
            derivative_component_interval(&curve, 1).unwrap(),
            derivative_component_interval(&curve, 2).unwrap(),
        ];
        for sample in 0..=100 {
            let parameter = f64::from(sample) / 100.0;
            let derivative = curve.eval_derivs(parameter, 1).d[1];
            for (interval, component) in
                intervals
                    .iter()
                    .zip([derivative.x, derivative.y, derivative.z])
            {
                assert!(interval.lo() <= component && component <= interval.hi());
            }
        }
    }

    #[test]
    fn range_certificate_joins_boundary_cells_with_source_interval_proof() {
        let rational = segment(
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Some(vec![1.0, 1.5]),
        );
        let horizontal = line(0.0);
        let cover = run(&rational, &horizontal, 6, BudgetPlan::empty());
        let cover = cover.result().unwrap();
        assert!(cover.is_complete());
        assert!(
            cover
                .candidates()
                .iter()
                .any(|cell| cell.certify_unique_root().is_none())
        );
        let first_range = cover
            .candidates()
            .iter()
            .map(CurvePairCandidateCell::first_range)
            .reduce(|a, b| ParamRange::new(a.lo.min(b.lo), a.hi.max(b.hi)))
            .unwrap();
        let second_range = cover
            .candidates()
            .iter()
            .map(CurvePairCandidateCell::second_range)
            .reduce(|a, b| ParamRange::new(a.lo.min(b.lo), a.hi.max(b.hi)))
            .unwrap();
        let certificate =
            certify_curve_pair_unique_root(&rational, first_range, &horizontal, second_range)
                .unwrap()
                .unwrap();
        assert_eq!(certificate.first_range(), first_range);
        assert_eq!(certificate.second_range(), second_range);
        assert!(certificate.determinant_lower_bound() > 0.0);
        let swapped =
            certify_curve_pair_unique_root(&horizontal, second_range, &rational, first_range)
                .unwrap()
                .unwrap();
        assert_eq!(swapped.first_range(), second_range);
        assert_eq!(swapped.second_range(), first_range);
        assert_eq!(swapped.projection_plane(), certificate.projection_plane());
        assert!(swapped.determinant_lower_bound() > 0.0);

        let separated = line(10.0);
        assert!(
            certify_curve_pair_unique_root(
                &rational,
                rational.param_range(),
                &separated,
                separated.param_range(),
            )
            .unwrap()
            .is_none()
        );
    }

    #[test]
    fn exact_affine_plane_detection_selects_an_injective_coordinate_projection() {
        let x_axis = segment(
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            None,
        );
        let z_axis = segment(
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
            None,
        );
        let xz = candidate_cell(x_axis, z_axis, 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(xz.projection_plane(), CurvePairProjectionPlane::Xz);

        let y_axis = segment(
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            None,
        );
        let z_axis = segment(
            Point3::new(0.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
            None,
        );
        let yz = candidate_cell(y_axis, z_axis, 0, 0.0)
            .unwrap()
            .certify_unique_root()
            .unwrap();
        assert_eq!(yz.projection_plane(), CurvePairProjectionPlane::Yz);
    }

    #[test]
    fn profile_is_an_exact_stable_contract() {
        let profile = NurbsCurvePairBudgetProfile::v1_defaults();
        assert_eq!(profile.limits().len(), 3);
        assert_eq!(
            profile
                .limits()
                .iter()
                .map(|limit| (limit.stage, limit.resource, limit.mode, limit.allowed))
                .collect::<Vec<_>>(),
            vec![
                (
                    NURBS_CURVE_PAIR_CANDIDATES,
                    ResourceKind::Items,
                    AccountingMode::HighWater,
                    4_096,
                ),
                (
                    NURBS_CURVE_PAIR_DEPTH,
                    ResourceKind::Depth,
                    AccountingMode::HighWater,
                    6,
                ),
                (
                    NURBS_CURVE_PAIR_SUBDIVISIONS,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    6_828,
                ),
            ]
        );
    }

    #[test]
    fn source_span_scans_are_pre_admitted_at_exact_work_boundaries() {
        let first = NurbsCurve::new(
            1,
            vec![0.0, 0.0, 0.25, 0.75, 1.0, 1.0],
            vec![
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-0.25, 0.0, 0.0),
                Point3::new(0.25, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            None,
        )
        .unwrap();
        let second = segment(
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            None,
        );

        // Setup evaluates one three-slot and one one-slot source enclosure.
        // The first split evaluates two ranges for each source: 1 + 2*(3+1).
        let exact_work = 5 + 9;
        let exact = run(
            &first,
            &second,
            1,
            BudgetPlan::new([LimitSpec::new(
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                exact_work,
            )])
            .unwrap(),
        );
        assert!(exact.result().unwrap().is_complete());
        assert_eq!(
            usage(
                exact.report(),
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
            )
            .consumed,
            exact_work,
        );

        let low = run(
            &first,
            &second,
            1,
            BudgetPlan::new([LimitSpec::new(
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                exact_work - 1,
            )])
            .unwrap(),
        );
        let isolation = low.result().unwrap();
        assert!(!isolation.is_complete());
        assert_eq!(isolation.candidates().len(), 1);
        assert_eq!(
            low.report().limit_events(),
            &[LimitSnapshot {
                stage: NURBS_CURVE_PAIR_SUBDIVISIONS,
                resource: ResourceKind::Work,
                consumed: exact_work,
                allowed: exact_work - 1,
            }],
        );
    }

    #[test]
    fn work_candidate_and_depth_boundaries_retain_conservative_cover() {
        let first = arch();
        let second = line(0.5);
        let baseline = run(&first, &second, 2, BudgetPlan::empty());
        let work = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_SUBDIVISIONS,
            ResourceKind::Work,
        );
        let candidates = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_CANDIDATES,
            ResourceKind::Items,
        );
        let depth = usage(
            baseline.report(),
            NURBS_CURVE_PAIR_DEPTH,
            ResourceKind::Depth,
        );
        assert!(work.consumed > 1);
        assert!(candidates.consumed > 1);
        assert_eq!(depth.consumed, 2);

        for (stage, resource, mode, consumed) in [
            (
                NURBS_CURVE_PAIR_SUBDIVISIONS,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                work.consumed,
            ),
            (
                NURBS_CURVE_PAIR_CANDIDATES,
                ResourceKind::Items,
                AccountingMode::HighWater,
                candidates.consumed,
            ),
            (
                NURBS_CURVE_PAIR_DEPTH,
                ResourceKind::Depth,
                AccountingMode::HighWater,
                depth.consumed,
            ),
        ] {
            let exact = BudgetPlan::new([LimitSpec::new(stage, resource, mode, consumed)]).unwrap();
            let exact = run(&first, &second, 2, exact);
            assert!(exact.result().unwrap().is_complete());
            assert!(exact.report().limit_events().is_empty());

            let low =
                BudgetPlan::new([LimitSpec::new(stage, resource, mode, consumed - 1)]).unwrap();
            let low = run(&first, &second, 2, low);
            let isolation = low.result().unwrap();
            assert!(!isolation.is_complete());
            assert!(!isolation.candidates().is_empty());
            let crossing = *low.report().limit_events().last().unwrap();
            assert_eq!(crossing.stage, stage);
            assert_eq!(
                (crossing.consumed, crossing.allowed),
                (consumed, consumed - 1)
            );
        }
    }

    #[test]
    fn missing_profile_is_rejected_before_a_root_level_empty_proof() {
        let first = line(0.0);
        let second = line(10.0);
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let mut scope = OperationScope::new(&context);
        let result = isolate_curve_pair_candidates_in_scope(
            &first,
            first.param_range(),
            &second,
            second.param_range(),
            Tolerances::default().linear(),
            2,
            &mut scope,
        );
        assert!(matches!(
            result,
            Err(ContextCurvePairIsolationError::Policy(
                OperationPolicyError::UnknownLimit { .. }
            ))
        ));
    }

    #[test]
    fn unrepresentable_midpoint_retains_cover_and_records_numeric_resolution() {
        let lo = 1.0e16_f64;
        let hi = lo.next_up();
        let first = NurbsCurve::new(
            1,
            vec![lo, lo, hi, hi],
            vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
            None,
        )
        .unwrap();
        let second = NurbsCurve::new(
            1,
            vec![lo, lo, hi, hi],
            vec![Point3::new(0.0, -1.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            None,
        )
        .unwrap();
        let outcome = run(&first, &second, 1, BudgetPlan::empty());
        let isolation = outcome.result().unwrap();
        assert!(!isolation.is_complete());
        assert!(isolation.limits().parameter_resolution());
        assert_eq!(isolation.candidates().len(), 1);
        assert_eq!(
            outcome.report().numeric_resolution_stages(),
            &[NURBS_CURVE_PAIR_DEPTH]
        );
    }
}
