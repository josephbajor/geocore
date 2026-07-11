//! Immutable session precision, numerical, and execution policy.

use std::num::NonZeroUsize;

use crate::tolerance::{ANGULAR_RESOLUTION, LINEAR_RESOLUTION, SIZE_BOX_HALF};

use super::budget::BudgetPlan;
use super::id::{OperationPolicyError, PolicyVersion};

/// Fixed session precision and size-box data.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SessionPrecision {
    linear_resolution: f64,
    angular_resolution: f64,
    size_box_half: f64,
}

impl SessionPrecision {
    /// The supported production v1 Parasolid-compatible precision regime.
    pub const fn parasolid() -> Self {
        Self {
            linear_resolution: LINEAR_RESOLUTION,
            angular_resolution: ANGULAR_RESOLUTION,
            size_box_half: SIZE_BOX_HALF,
        }
    }

    /// Constructs a validated precision regime.
    ///
    /// Custom regimes are useful for policy testing; production v1 sessions
    /// should use [`Self::parasolid`].
    pub fn try_new(
        linear_resolution: f64,
        angular_resolution: f64,
        size_box_half: f64,
    ) -> core::result::Result<Self, OperationPolicyError> {
        if !linear_resolution.is_finite()
            || linear_resolution <= 0.0
            || !angular_resolution.is_finite()
            || angular_resolution <= 0.0
            || !size_box_half.is_finite()
            || size_box_half <= 0.0
        {
            return Err(OperationPolicyError::InvalidSessionPrecision);
        }
        Ok(Self {
            linear_resolution,
            angular_resolution,
            size_box_half,
        })
    }

    /// Returns the linear session resolution in meters.
    pub const fn linear_resolution(self) -> f64 {
        self.linear_resolution
    }

    /// Returns the angular session resolution in radians.
    pub const fn angular_resolution(self) -> f64 {
        self.angular_resolution
    }

    /// Returns the positive half-extent of the session size box in meters.
    pub const fn size_box_half(self) -> f64 {
        self.size_box_half
    }
}

impl Default for SessionPrecision {
    fn default() -> Self {
        Self::parasolid()
    }
}

/// Semantic kind of a scale-aware arithmetic guard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NumericGuardKind {
    /// Ensures iterative parameter progress.
    ParameterProgress,
    /// Bounds cancellation in coefficient arithmetic.
    CoefficientCancellation,
    /// Guards linear solves and Jacobian calculations.
    LinearSolve,
    /// Guards periodic parameter normalization.
    PeriodicNormalization,
    /// Guards conversion and arithmetic in resource accounting.
    BudgetAccounting,
}

/// Scale data used to derive a parameter-space stopping threshold.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParameterScale {
    /// Magnitude of the represented parameter coordinate.
    pub coordinate_magnitude: f64,
    /// Width of the active parameter span.
    pub span: f64,
    /// Optional upper bound on model-space output change per parameter unit.
    pub output_rate_upper: Option<f64>,
}

/// Scale-aware parameter thresholds with no model-acceptance authority.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParameterTolerance {
    /// Suggested iteration termination step.
    pub termination_step: f64,
    /// Threshold arising only from floating-point rounding.
    pub rounding_floor: f64,
    /// Step derived from a model-space output tolerance and output rate.
    pub metric_driven_step: Option<f64>,
}

/// Versioned recipes for arithmetic guards and conditioning checks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NumericalPolicy {
    rounding_factor: f64,
    progress_factor: f64,
    reciprocal_condition_floor: f64,
}

impl NumericalPolicy {
    /// Returns the version-1 numerical policy.
    pub const fn v1() -> Self {
        Self {
            rounding_factor: 32.0,
            progress_factor: 64.0,
            reciprocal_condition_floor: 128.0 * f64::EPSILON,
        }
    }

    /// Constructs a validated numerical policy.
    pub fn try_new(
        rounding_factor: f64,
        progress_factor: f64,
        reciprocal_condition_floor: f64,
    ) -> core::result::Result<Self, OperationPolicyError> {
        if !rounding_factor.is_finite()
            || rounding_factor <= 0.0
            || !progress_factor.is_finite()
            || progress_factor <= 0.0
            || !reciprocal_condition_floor.is_finite()
            || reciprocal_condition_floor <= 0.0
            || reciprocal_condition_floor > 1.0
        {
            return Err(OperationPolicyError::InvalidNumericalPolicy);
        }
        Ok(Self {
            rounding_factor,
            progress_factor,
            reciprocal_condition_floor,
        })
    }

    /// Returns a scale-aware floating-point rounding guard.
    ///
    /// The result is an arithmetic threshold only and cannot prove geometric
    /// acceptance.
    pub fn rounding_guard(self, kind: NumericGuardKind, scale: f64) -> f64 {
        let kind_factor = match kind {
            NumericGuardKind::ParameterProgress => 1.0,
            NumericGuardKind::CoefficientCancellation => 2.0,
            NumericGuardKind::LinearSolve => 4.0,
            NumericGuardKind::PeriodicNormalization => 2.0,
            NumericGuardKind::BudgetAccounting => 1.0,
        };
        scale.abs().max(1.0) * f64::EPSILON * self.rounding_factor * kind_factor
    }

    /// Derives parameter-space progress thresholds from local scale data.
    pub fn parameter_tolerance(
        self,
        scale: ParameterScale,
        output_tolerance: f64,
    ) -> core::result::Result<ParameterTolerance, OperationPolicyError> {
        let output_rate_valid = scale
            .output_rate_upper
            .is_none_or(|rate| rate.is_finite() && rate > 0.0);
        if !scale.coordinate_magnitude.is_finite()
            || !scale.span.is_finite()
            || scale.span <= 0.0
            || !output_tolerance.is_finite()
            || output_tolerance <= 0.0
            || !output_rate_valid
        {
            return Err(OperationPolicyError::InvalidNumericalPolicy);
        }
        let local_scale = scale.coordinate_magnitude.abs().max(scale.span);
        let rounding_floor = self.rounding_guard(NumericGuardKind::ParameterProgress, local_scale);
        let progress_floor = local_scale.max(1.0) * f64::EPSILON * self.progress_factor;
        let metric_driven_step = scale
            .output_rate_upper
            .map(|output_rate| output_tolerance / output_rate);
        let termination_step =
            metric_driven_step.map_or(progress_floor, |metric| progress_floor.max(metric));
        Ok(ParameterTolerance {
            termination_step,
            rounding_floor,
            metric_driven_step,
        })
    }

    /// Returns whether a normalized reciprocal condition estimate is usable.
    pub fn reciprocal_condition_is_usable(self, reciprocal_condition: f64) -> bool {
        reciprocal_condition.is_finite() && reciprocal_condition >= self.reciprocal_condition_floor
    }
}

impl Default for NumericalPolicy {
    fn default() -> Self {
        Self::v1()
    }
}

/// Deterministic concurrency policy for operation work.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ExecutionPolicy {
    /// Execute all work serially.
    Serial,
    /// Use no more than the specified number of workers.
    AtMost(NonZeroUsize),
    /// Use the available hardware parallelism.
    #[default]
    Available,
}

impl ExecutionPolicy {
    /// Resolves a worker count for a known number of independent work items.
    pub fn worker_count(self, work_len: usize) -> usize {
        if work_len == 0 {
            return 0;
        }
        let available = std::thread::available_parallelism()
            .map(NonZeroUsize::get)
            .unwrap_or(1);
        match self {
            Self::Serial => 1,
            Self::AtMost(limit) => limit.get().min(available).min(work_len),
            Self::Available => available.min(work_len),
        }
    }
}

/// Immutable validated policy shared by operations in one kernel session.
#[derive(Debug, Clone, PartialEq)]
pub struct SessionPolicy {
    precision: SessionPrecision,
    numerical: NumericalPolicy,
    execution: ExecutionPolicy,
    default_budget: BudgetPlan,
    policy_version: PolicyVersion,
}

impl SessionPolicy {
    /// Constructs a policy from already validated components.
    pub const fn new(
        precision: SessionPrecision,
        numerical: NumericalPolicy,
        execution: ExecutionPolicy,
        default_budget: BudgetPlan,
        policy_version: PolicyVersion,
    ) -> Self {
        Self {
            precision,
            numerical,
            execution,
            default_budget,
            policy_version,
        }
    }

    /// Returns the stable v1 production policy.
    pub fn v1() -> Self {
        Self::new(
            SessionPrecision::parasolid(),
            NumericalPolicy::v1(),
            ExecutionPolicy::Available,
            BudgetPlan::empty(),
            PolicyVersion::V1,
        )
    }

    /// Returns fixed session precision.
    pub const fn precision(&self) -> SessionPrecision {
        self.precision
    }

    /// Returns numerical guard recipes.
    pub const fn numerical(&self) -> NumericalPolicy {
        self.numerical
    }

    /// Returns deterministic execution policy.
    pub const fn execution(&self) -> ExecutionPolicy {
        self.execution
    }

    /// Returns the default operation budget.
    pub const fn default_budget(&self) -> &BudgetPlan {
        &self.default_budget
    }

    /// Returns the stable policy version.
    pub const fn policy_version(&self) -> PolicyVersion {
        self.policy_version
    }
}

impl Default for SessionPolicy {
    fn default() -> Self {
        Self::v1()
    }
}
