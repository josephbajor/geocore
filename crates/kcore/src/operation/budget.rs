//! Deterministic budget plans, work ledgers, and child reservations.

use super::id::{OperationPolicyError, StageId};

/// Stable stage for the synthetic root ceiling over cumulative work.
pub const TOTAL_WORK_STAGE: StageId = match StageId::new("kcore.operation.total-work") {
    Ok(stage) => stage,
    Err(_) => panic!("valid total-work stage identifier"),
};

/// The category of deterministically accounted resource.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[non_exhaustive]
pub enum ResourceKind {
    /// Abstract deterministic algorithm work units.
    Work,
    /// Retained or emitted item count.
    Items,
    /// Scratch or retained byte count.
    Bytes,
    /// Recursion or dependency depth.
    Depth,
}

/// How a resource is accounted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AccountingMode {
    /// Values are added over the operation.
    Cumulative,
    /// Only the largest observed value is retained.
    HighWater,
}

/// A deterministic limit for one stage/resource pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LimitSpec {
    /// Stable stage identifier.
    pub stage: StageId,
    /// Resource being limited.
    pub resource: ResourceKind,
    /// Accounting mode for the resource.
    pub mode: AccountingMode,
    /// Largest permitted usage value, inclusive.
    pub allowed: u64,
}

impl LimitSpec {
    /// Creates a limit specification.
    pub const fn new(
        stage: StageId,
        resource: ResourceKind,
        mode: AccountingMode,
        allowed: u64,
    ) -> Self {
        Self {
            stage,
            resource,
            mode,
            allowed,
        }
    }
}

/// Usage and allowance for one stage/resource pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LimitSnapshot {
    /// Stable stage identifier.
    pub stage: StageId,
    /// Accounted resource.
    pub resource: ResourceKind,
    /// Consumed or observed amount.
    pub consumed: u64,
    /// Configured inclusive allowance.
    pub allowed: u64,
}

/// Validated, deterministically ordered resource limits.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BudgetPlan {
    limits: Vec<LimitSpec>,
    total_work_allowed: Option<u64>,
}

impl BudgetPlan {
    /// Returns a plan with no stage limits or total-work ceiling.
    pub const fn empty() -> Self {
        Self {
            limits: Vec::new(),
            total_work_allowed: None,
        }
    }

    /// Validates and deterministically orders limit specifications.
    pub fn new(
        limits: impl IntoIterator<Item = LimitSpec>,
    ) -> core::result::Result<Self, OperationPolicyError> {
        let mut limits: Vec<_> = limits.into_iter().collect();
        limits.sort_by_key(|limit| (limit.stage, limit.resource));
        for limit in &limits {
            let mode_valid = match limit.resource {
                ResourceKind::Work => limit.mode == AccountingMode::Cumulative,
                ResourceKind::Depth | ResourceKind::Bytes => {
                    limit.mode == AccountingMode::HighWater
                }
                ResourceKind::Items => true,
            };
            if !mode_valid {
                return Err(OperationPolicyError::InvalidLimitMode {
                    stage: limit.stage,
                    resource: limit.resource,
                });
            }
        }
        for pair in limits.windows(2) {
            if pair[0].stage == pair[1].stage && pair[0].resource == pair[1].resource {
                return Err(OperationPolicyError::DuplicateLimit {
                    stage: pair[0].stage,
                    resource: pair[0].resource,
                });
            }
        }
        Ok(Self {
            limits,
            total_work_allowed: None,
        })
    }

    /// Adds an inclusive root ceiling over all cumulative `Work` stages.
    pub fn with_total_work_limit(mut self, allowed: u64) -> Self {
        self.total_work_allowed = Some(allowed);
        self
    }

    /// Returns limits in canonical stage/resource order.
    pub fn limits(&self) -> &[LimitSpec] {
        &self.limits
    }

    /// Requires a configured stage/resource pair with the expected
    /// accounting mode.
    ///
    /// This is a read-only configuration check for algorithms that must
    /// reject an incompatible effective plan before starting work.
    pub fn require_limit(
        &self,
        stage: StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        let index = self
            .limits
            .binary_search_by_key(&(stage, resource), |limit| (limit.stage, limit.resource))
            .map_err(|_| OperationPolicyError::UnknownLimit { stage, resource })?;
        require_accounting_mode(self.limits[index], mode)
    }

    /// Returns the root total-work ceiling, when configured.
    pub const fn total_work_limit(&self) -> Option<u64> {
        self.total_work_allowed
    }

    /// Overlays this plan with replacements and additions from `overrides`.
    pub fn overlaid(&self, overrides: &Self) -> Self {
        let mut limits = self.limits.clone();
        for replacement in &overrides.limits {
            match limits.binary_search_by_key(&(replacement.stage, replacement.resource), |limit| {
                (limit.stage, limit.resource)
            }) {
                Ok(index) => limits[index] = *replacement,
                Err(index) => limits.insert(index, *replacement),
            }
        }
        Self {
            limits,
            total_work_allowed: overrides.total_work_allowed.or(self.total_work_allowed),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct UsageEntry {
    spec: LimitSpec,
    consumed: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Reservation {
    ordinal: u64,
    plan: BudgetPlan,
}

/// Mutable deterministic resource accounting for one operation or child.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WorkLedger {
    entries: Vec<UsageEntry>,
    total_work_allowed: Option<u64>,
    total_work_consumed: u64,
    limit_events: Vec<LimitSnapshot>,
    numeric_resolution_stages: Vec<StageId>,
    reservations: Vec<Reservation>,
}

impl WorkLedger {
    /// Creates a fresh ledger with zero usage.
    pub fn new(plan: BudgetPlan) -> Self {
        Self {
            entries: plan
                .limits
                .into_iter()
                .map(|spec| UsageEntry { spec, consumed: 0 })
                .collect(),
            total_work_allowed: plan.total_work_allowed,
            total_work_consumed: 0,
            limit_events: Vec::new(),
            numeric_resolution_stages: Vec::new(),
            reservations: Vec::new(),
        }
    }

    /// Charges cumulative `Work` units at a stage.
    pub fn charge(
        &mut self,
        stage: StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.charge_resource(stage, ResourceKind::Work, amount)
    }

    /// Requires a configured stage/resource pair with the expected
    /// accounting mode without mutating usage or evidence.
    ///
    /// Shared-scope algorithms use this to validate the actual ledger they
    /// will charge before inspecting operation inputs or evaluating geometry.
    pub fn require_limit(
        &self,
        stage: StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        let index = self.entry_index(stage, resource)?;
        require_accounting_mode(self.entries[index].spec, mode)
    }

    /// Checks whether cumulative `Work` could be charged without mutating
    /// accepted usage or limit-event evidence.
    ///
    /// This supports algorithms whose semantic unit is completed work: they
    /// can preserve limit precedence before starting an atomic unit, then
    /// perform the real [`Self::charge`] only after that unit succeeds.
    pub fn check_charge(
        &self,
        stage: StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.check_charge_resource(stage, ResourceKind::Work, amount)
    }

    /// Checks whether a cumulative resource charge would fit without
    /// mutating this ledger.
    pub fn check_charge_resource(
        &self,
        stage: StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        let index = self.entry_index(stage, resource)?;
        if self.entries[index].spec.mode != AccountingMode::Cumulative {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
        let attempted = self.entries[index]
            .consumed
            .checked_add(amount)
            .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?;
        if let Some(snapshot) = self.stage_limit_crossing(index, attempted)? {
            return Err(OperationPolicyError::LimitReached(snapshot));
        }
        if resource == ResourceKind::Work {
            let attempted_total = self
                .total_work_consumed
                .checked_add(amount)
                .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?;
            if let Some(allowed) = self.total_work_allowed {
                let usable = allowed.saturating_sub(self.reserved_total_work()?);
                if attempted_total > usable {
                    return Err(OperationPolicyError::LimitReached(LimitSnapshot {
                        stage: TOTAL_WORK_STAGE,
                        resource,
                        consumed: attempted_total,
                        allowed: usable,
                    }));
                }
            }
        }
        Ok(())
    }

    /// Charges a cumulative resource at a stage.
    pub fn charge_resource(
        &mut self,
        stage: StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        if let Err(error) = self.check_charge_resource(stage, resource, amount) {
            if let OperationPolicyError::LimitReached(snapshot) = error {
                self.record_limit(snapshot);
                return Err(OperationPolicyError::LimitReached(snapshot));
            }
            return Err(error);
        }
        let index = self.entry_index(stage, resource)?;
        let current = self.entries[index].consumed;
        let attempted = current
            .checked_add(amount)
            .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?;
        let attempted_total = if resource == ResourceKind::Work {
            Some(
                self.total_work_consumed
                    .checked_add(amount)
                    .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?,
            )
        } else {
            None
        };
        self.entries[index].consumed = attempted;
        if let Some(total) = attempted_total {
            self.total_work_consumed = total;
        }
        Ok(())
    }

    /// Observes a high-water value at a stage.
    pub fn observe(
        &mut self,
        stage: StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        let index = self.entry_index(stage, resource)?;
        if self.entries[index].spec.mode != AccountingMode::HighWater {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
        let attempted = self.entries[index].consumed.max(value);
        if let Some(snapshot) = self.stage_limit_crossing(index, attempted)? {
            self.record_limit(snapshot);
            return Err(OperationPolicyError::LimitReached(snapshot));
        }
        self.entries[index].consumed = attempted;
        Ok(())
    }

    /// Starts a strictly sequential nested ledger with invocation-local caps.
    ///
    /// Unlike [`Self::reserve_child`], this does not reserve capacity or defer
    /// accounting until a join. Every accepted unit is reflected in the
    /// parent immediately, while `plan` independently limits this one nested
    /// invocation. Borrowing the parent for the ledger's lifetime makes that
    /// real-time composition strictly sequential.
    pub fn sequential(
        &mut self,
        plan: BudgetPlan,
    ) -> core::result::Result<SequentialWorkLedger<'_>, OperationPolicyError> {
        SequentialWorkLedger::new(self, plan)
    }

    /// Reserves deterministic child allowances under a stable work ordinal.
    ///
    /// Reservations must be planned in increasing ordinal order. Parent work
    /// cannot consume reserved capacity until children are merged. Additive
    /// resources reserve the checked sum of child allowances. `Depth` is a
    /// branch high-water value, so parent observations and child allowances
    /// reserve only their maximum.
    ///
    /// When the parent has a root total-work ceiling and the child plan omits
    /// one, the child root reservation is inferred as the checked sum of its
    /// cumulative `Work` allowances. An explicit child root ceiling is used as
    /// written, including when it is stricter than that sum; the inferred sum
    /// is never added to an explicit ceiling.
    pub fn reserve_child(
        &mut self,
        ordinal: u64,
        mut plan: BudgetPlan,
    ) -> core::result::Result<ChildWorkLedger, OperationPolicyError> {
        if self
            .reservations
            .last()
            .is_some_and(|reservation| reservation.ordinal >= ordinal)
        {
            return Err(OperationPolicyError::InvalidChildOrdinal);
        }
        if self.total_work_allowed.is_some() && plan.total_work_allowed.is_none() {
            let inferred = plan
                .limits
                .iter()
                .filter(|spec| spec.resource == ResourceKind::Work)
                .try_fold(0_u64, |sum, spec| {
                    sum.checked_add(spec.allowed)
                        .ok_or(OperationPolicyError::AccountingOverflow {
                            stage: TOTAL_WORK_STAGE,
                            resource: ResourceKind::Work,
                        })
                })?;
            plan.total_work_allowed = Some(inferred);
        }
        for spec in &plan.limits {
            let index = self.entry_index(spec.stage, spec.resource)?;
            let parent = self.entries[index];
            if parent.spec.mode != spec.mode {
                return Err(OperationPolicyError::AccountingModeMismatch {
                    stage: spec.stage,
                    resource: spec.resource,
                });
            }
            let already_reserved = self.reserved_for(spec.stage, spec.resource)?;
            let fits = if spec.resource == ResourceKind::Depth {
                parent.consumed.max(already_reserved).max(spec.allowed) <= parent.spec.allowed
            } else {
                let required = already_reserved.checked_add(spec.allowed).ok_or(
                    OperationPolicyError::AccountingOverflow {
                        stage: spec.stage,
                        resource: spec.resource,
                    },
                )?;
                required <= parent.spec.allowed.saturating_sub(parent.consumed)
            };
            if !fits {
                return Err(OperationPolicyError::ChildReservationExceeded {
                    stage: spec.stage,
                    resource: spec.resource,
                });
            }
        }
        if let Some(child_total) = plan.total_work_allowed {
            let reserved_total = self.reserved_total_work()?;
            let required = reserved_total.checked_add(child_total).ok_or(
                OperationPolicyError::AccountingOverflow {
                    stage: TOTAL_WORK_STAGE,
                    resource: ResourceKind::Work,
                },
            )?;
            let available = self
                .total_work_allowed
                .ok_or(OperationPolicyError::UnknownLimit {
                    stage: TOTAL_WORK_STAGE,
                    resource: ResourceKind::Work,
                })?
                .saturating_sub(self.total_work_consumed);
            if required > available {
                return Err(OperationPolicyError::ChildReservationExceeded {
                    stage: TOTAL_WORK_STAGE,
                    resource: ResourceKind::Work,
                });
            }
        }
        self.reservations.push(Reservation {
            ordinal,
            plan: plan.clone(),
        });
        Ok(ChildWorkLedger {
            ordinal,
            ledger: Self::new(plan),
        })
    }

    /// Merges completed child ledgers in stable ordinal order.
    ///
    /// The input order has no effect on usage or snapshot ordering. All
    /// active reservations must be returned together at the deterministic
    /// join point.
    pub fn merge_children(
        &mut self,
        mut children: Vec<ChildWorkLedger>,
    ) -> core::result::Result<(), OperationPolicyError> {
        children.sort_by_key(ChildWorkLedger::ordinal);
        if children.len() != self.reservations.len()
            || children
                .iter()
                .zip(&self.reservations)
                .any(|(child, reservation)| child.ordinal != reservation.ordinal)
        {
            return Err(OperationPolicyError::UnknownChildReservation);
        }

        let original = self.clone();
        let reservations = core::mem::take(&mut self.reservations);
        for (child, reservation) in children.into_iter().zip(reservations) {
            if child.ledger.entries.len() != reservation.plan.limits.len() {
                *self = original;
                return Err(OperationPolicyError::UnknownChildReservation);
            }
            for entry in child.ledger.entries {
                let result = match entry.spec.mode {
                    AccountingMode::Cumulative => {
                        self.charge_resource(entry.spec.stage, entry.spec.resource, entry.consumed)
                    }
                    AccountingMode::HighWater => {
                        self.observe(entry.spec.stage, entry.spec.resource, entry.consumed)
                    }
                };
                if let Err(error) = result {
                    let attempted_limit = match error {
                        OperationPolicyError::LimitReached(snapshot) => Some(snapshot),
                        _ => None,
                    };
                    *self = original;
                    if let Some(snapshot) = attempted_limit {
                        self.record_limit(snapshot);
                    }
                    return Err(error);
                }
            }
            for snapshot in child.ledger.limit_events {
                self.record_limit(snapshot);
            }
            for stage in child.ledger.numeric_resolution_stages {
                self.record_numeric_resolution(stage);
            }
        }
        Ok(())
    }

    /// Returns canonical stage/resource usage snapshots, including zeros.
    pub fn snapshots(&self) -> Vec<LimitSnapshot> {
        let mut snapshots: Vec<_> = self
            .entries
            .iter()
            .map(|entry| LimitSnapshot {
                stage: entry.spec.stage,
                resource: entry.spec.resource,
                consumed: entry.consumed,
                allowed: entry.spec.allowed,
            })
            .collect();
        if let Some(allowed) = self.total_work_allowed {
            snapshots.push(LimitSnapshot {
                stage: TOTAL_WORK_STAGE,
                resource: ResourceKind::Work,
                consumed: self.total_work_consumed,
                allowed,
            });
            snapshots.sort_by_key(|snapshot| (snapshot.stage, snapshot.resource));
        }
        snapshots
    }

    /// Returns the total cumulative work charged across work stages.
    pub const fn total_work_consumed(&self) -> u64 {
        self.total_work_consumed
    }

    /// Returns the first actual attempted crossing for each configured
    /// stage/resource pair, in deterministic observation order.
    pub fn limit_events(&self) -> &[LimitSnapshot] {
        &self.limit_events
    }

    /// Retains a numeric-resolution stop once in first-observed order.
    pub fn record_numeric_resolution(&mut self, stage: StageId) {
        if !self.numeric_resolution_stages.contains(&stage) {
            self.numeric_resolution_stages.push(stage);
        }
    }

    /// Returns numeric-resolution stops in deterministic observation order.
    pub fn numeric_resolution_stages(&self) -> &[StageId] {
        &self.numeric_resolution_stages
    }

    fn entry_index(
        &self,
        stage: StageId,
        resource: ResourceKind,
    ) -> core::result::Result<usize, OperationPolicyError> {
        self.entries
            .binary_search_by_key(&(stage, resource), |entry| {
                (entry.spec.stage, entry.spec.resource)
            })
            .map_err(|_| OperationPolicyError::UnknownLimit { stage, resource })
    }

    fn stage_limit_crossing(
        &self,
        index: usize,
        attempted: u64,
    ) -> core::result::Result<Option<LimitSnapshot>, OperationPolicyError> {
        let entry = self.entries[index];
        let usable = if entry.spec.resource == ResourceKind::Depth {
            entry.spec.allowed
        } else {
            let reserved = self.reserved_for(entry.spec.stage, entry.spec.resource)?;
            entry.spec.allowed.saturating_sub(reserved)
        };
        if attempted > usable {
            return Ok(Some(LimitSnapshot {
                stage: entry.spec.stage,
                resource: entry.spec.resource,
                consumed: attempted,
                allowed: usable,
            }));
        }
        Ok(None)
    }

    fn record_limit(&mut self, snapshot: LimitSnapshot) {
        if !self
            .limit_events
            .iter()
            .any(|event| event.stage == snapshot.stage && event.resource == snapshot.resource)
        {
            self.limit_events.push(snapshot);
        }
    }

    fn reserved_for(
        &self,
        stage: StageId,
        resource: ResourceKind,
    ) -> core::result::Result<u64, OperationPolicyError> {
        self.reservations
            .iter()
            .try_fold(0_u64, |reserved, reservation| {
                let Some(allowed) = reservation
                    .plan
                    .limits
                    .iter()
                    .find(|spec| spec.stage == stage && spec.resource == resource)
                    .map(|spec| spec.allowed)
                else {
                    return Ok(reserved);
                };
                if resource == ResourceKind::Depth {
                    Ok(reserved.max(allowed))
                } else {
                    reserved
                        .checked_add(allowed)
                        .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })
                }
            })
    }

    fn reserved_total_work(&self) -> core::result::Result<u64, OperationPolicyError> {
        self.reservations
            .iter()
            .filter_map(|reservation| reservation.plan.total_work_allowed)
            .try_fold(0_u64, |sum, value| {
                sum.checked_add(value)
                    .ok_or(OperationPolicyError::AccountingOverflow {
                        stage: TOTAL_WORK_STAGE,
                        resource: ResourceKind::Work,
                    })
            })
    }
}

/// Strictly sequential nested accounting with local and aggregate limits.
///
/// Each charge or observation must fit both this invocation's local plan and
/// the borrowed parent's current aggregate allowance. Local limits are tested
/// first. An accepted unit mutates both ledgers exactly once; a rejected unit
/// mutates neither usage ledger. Limit and numeric-resolution evidence is
/// forwarded to the parent immediately so operation reports remain complete.
#[derive(Debug)]
pub struct SequentialWorkLedger<'parent> {
    parent: &'parent mut WorkLedger,
    local: WorkLedger,
}

impl<'parent> SequentialWorkLedger<'parent> {
    /// Creates a sequential nested ledger after validating its stage schema.
    ///
    /// Every local stage/resource pair must exist in the parent with the same
    /// accounting mode. A local total-work ceiling is independent of whether
    /// the parent has a root ceiling: it is an invocation-local cap, not a
    /// capacity reservation.
    pub fn new(
        parent: &'parent mut WorkLedger,
        plan: BudgetPlan,
    ) -> core::result::Result<Self, OperationPolicyError> {
        for spec in plan.limits() {
            parent.require_limit(spec.stage, spec.resource, spec.mode)?;
        }
        Ok(Self {
            parent,
            local: WorkLedger::new(plan),
        })
    }

    /// Requires a configured local stage/resource pair and accounting mode.
    pub fn require_limit(
        &self,
        stage: StageId,
        resource: ResourceKind,
        mode: AccountingMode,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.local.require_limit(stage, resource, mode)
    }

    /// Checks a cumulative local and parent work charge without mutation.
    pub fn check_charge(
        &self,
        stage: StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.check_charge_resource(stage, ResourceKind::Work, amount)
    }

    /// Checks a cumulative local and parent resource charge without mutation.
    pub fn check_charge_resource(
        &self,
        stage: StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.local.check_charge_resource(stage, resource, amount)?;
        self.parent.check_charge_resource(stage, resource, amount)
    }

    /// Charges cumulative work against the local and parent plans atomically.
    pub fn charge(
        &mut self,
        stage: StageId,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        self.charge_resource(stage, ResourceKind::Work, amount)
    }

    /// Charges a cumulative resource against both plans atomically.
    pub fn charge_resource(
        &mut self,
        stage: StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        if let Err(error) = self.local.check_charge_resource(stage, resource, amount) {
            if let OperationPolicyError::LimitReached(snapshot) = error {
                self.local.record_limit(snapshot);
                self.parent.record_limit(snapshot);
                return Err(OperationPolicyError::LimitReached(snapshot));
            }
            return Err(error);
        }
        if let Err(error) = self.parent.check_charge_resource(stage, resource, amount) {
            if let OperationPolicyError::LimitReached(snapshot) = error {
                self.parent.record_limit(snapshot);
                return Err(OperationPolicyError::LimitReached(snapshot));
            }
            return Err(error);
        }

        apply_checked_charge(&mut self.local, stage, resource, amount);
        apply_checked_charge(self.parent, stage, resource, amount);
        Ok(())
    }

    /// Observes a high-water value against both plans atomically.
    pub fn observe(
        &mut self,
        stage: StageId,
        resource: ResourceKind,
        value: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        let local_index = self.local.entry_index(stage, resource)?;
        if self.local.entries[local_index].spec.mode != AccountingMode::HighWater {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
        let local_attempted = self.local.entries[local_index].consumed.max(value);
        if let Some(snapshot) = self
            .local
            .stage_limit_crossing(local_index, local_attempted)?
        {
            self.local.record_limit(snapshot);
            self.parent.record_limit(snapshot);
            return Err(OperationPolicyError::LimitReached(snapshot));
        }

        let parent_index = self.parent.entry_index(stage, resource)?;
        if self.parent.entries[parent_index].spec.mode != AccountingMode::HighWater {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
        let parent_attempted = self.parent.entries[parent_index].consumed.max(value);
        if let Some(snapshot) = self
            .parent
            .stage_limit_crossing(parent_index, parent_attempted)?
        {
            self.parent.record_limit(snapshot);
            return Err(OperationPolicyError::LimitReached(snapshot));
        }

        self.local.entries[local_index].consumed = local_attempted;
        self.parent.entries[parent_index].consumed = parent_attempted;
        Ok(())
    }

    /// Retains numeric-resolution evidence locally and in the parent.
    pub fn record_numeric_resolution(&mut self, stage: StageId) {
        self.local.record_numeric_resolution(stage);
        self.parent.record_numeric_resolution(stage);
    }

    /// Returns invocation-local usage snapshots, including zeros.
    pub fn snapshots(&self) -> Vec<LimitSnapshot> {
        self.local.snapshots()
    }

    /// Returns invocation-local limit events in first-observed order.
    pub fn limit_events(&self) -> &[LimitSnapshot] {
        self.local.limit_events()
    }

    /// Returns invocation-local numeric-resolution evidence.
    pub fn numeric_resolution_stages(&self) -> &[StageId] {
        self.local.numeric_resolution_stages()
    }
}

fn apply_checked_charge(
    ledger: &mut WorkLedger,
    stage: StageId,
    resource: ResourceKind,
    amount: u64,
) {
    let index = ledger
        .entry_index(stage, resource)
        .expect("a checked charge retains its configured stage");
    ledger.entries[index].consumed = ledger.entries[index]
        .consumed
        .checked_add(amount)
        .expect("a checked charge cannot overflow stage usage");
    if resource == ResourceKind::Work {
        ledger.total_work_consumed = ledger
            .total_work_consumed
            .checked_add(amount)
            .expect("a checked charge cannot overflow total work");
    }
}

fn require_accounting_mode(
    spec: LimitSpec,
    mode: AccountingMode,
) -> core::result::Result<(), OperationPolicyError> {
    if spec.mode != mode {
        return Err(OperationPolicyError::AccountingModeMismatch {
            stage: spec.stage,
            resource: spec.resource,
        });
    }
    Ok(())
}

/// A deterministically reserved child ledger.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChildWorkLedger {
    ordinal: u64,
    ledger: WorkLedger,
}

impl ChildWorkLedger {
    /// Returns the stable work-item ordinal.
    pub const fn ordinal(&self) -> u64 {
        self.ordinal
    }

    /// Returns the child's mutable ledger.
    pub fn ledger_mut(&mut self) -> &mut WorkLedger {
        &mut self.ledger
    }

    /// Returns the child's immutable ledger.
    pub const fn ledger(&self) -> &WorkLedger {
        &self.ledger
    }
}

#[cfg(test)]
mod internal_tests {
    use super::*;

    const WORK_STAGE: StageId = match StageId::new("kcore.test.merge-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid work stage"),
    };
    const NUMERIC_STAGE: StageId = match StageId::new("kcore.test.merge-numeric") {
        Ok(stage) => stage,
        Err(_) => panic!("valid numeric stage"),
    };

    #[test]
    fn failed_merge_rolls_back_usage_and_reservations_but_retains_attempted_limit() {
        let parent_plan = BudgetPlan::new([LimitSpec::new(
            WORK_STAGE,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            10,
        )])
        .unwrap();
        let child_plan = BudgetPlan::new([LimitSpec::new(
            WORK_STAGE,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            5,
        )])
        .unwrap();
        let mut parent = WorkLedger::new(parent_plan);
        parent.record_numeric_resolution(NUMERIC_STAGE);
        let mut child = parent.reserve_child(1, child_plan).unwrap();
        child.ledger.record_numeric_resolution(WORK_STAGE);

        // Simulate a corrupted/foreign child result that violated its reserved
        // allowance. Safe public child accounting cannot construct this state;
        // the test pins failure rollback and attempted-event retention if a
        // merge nevertheless encounters it.
        child.ledger.entries[0].consumed = 11;
        let snapshot = match parent.merge_children(vec![child]) {
            Err(OperationPolicyError::LimitReached(snapshot)) => snapshot,
            other => panic!("unexpected merge result: {other:?}"),
        };
        assert_eq!(snapshot.stage, WORK_STAGE);
        assert_eq!(snapshot.consumed, 11);
        assert_eq!(snapshot.allowed, 10);
        assert_eq!(parent.snapshots()[0].consumed, 0);
        assert_eq!(parent.reservations.len(), 1);
        assert_eq!(parent.limit_events(), &[snapshot]);
        assert_eq!(parent.numeric_resolution_stages(), &[NUMERIC_STAGE]);
    }
}
