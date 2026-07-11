//! Deterministic budget plans, work ledgers, and child reservations.

use super::id::{OperationPolicyError, StageId};

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

    /// Charges a cumulative resource at a stage.
    pub fn charge_resource(
        &mut self,
        stage: StageId,
        resource: ResourceKind,
        amount: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        let index = self.entry_index(stage, resource)?;
        if self.entries[index].spec.mode != AccountingMode::Cumulative {
            return Err(OperationPolicyError::AccountingModeMismatch { stage, resource });
        }
        let current = self.entries[index].consumed;
        let attempted = current
            .checked_add(amount)
            .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?;
        self.ensure_within_reserved_capacity(index, attempted)?;
        let attempted_total = if resource == ResourceKind::Work {
            Some(
                self.total_work_consumed
                    .checked_add(amount)
                    .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })?,
            )
        } else {
            None
        };
        if let (Some(total), Some(allowed)) = (attempted_total, self.total_work_allowed) {
            let usable = allowed.saturating_sub(self.reserved_total_work()?);
            if total > usable {
                return Err(OperationPolicyError::LimitReached(LimitSnapshot {
                    stage: total_work_stage(),
                    resource,
                    consumed: total,
                    allowed: usable,
                }));
            }
        }
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
        self.ensure_within_reserved_capacity(index, attempted)?;
        self.entries[index].consumed = attempted;
        Ok(())
    }

    /// Reserves deterministic child allowances under a stable work ordinal.
    ///
    /// Reservations must be planned in increasing ordinal order. Parent work
    /// cannot consume reserved capacity until children are merged.
    pub fn reserve_child(
        &mut self,
        ordinal: u64,
        plan: BudgetPlan,
    ) -> core::result::Result<ChildWorkLedger, OperationPolicyError> {
        if self
            .reservations
            .last()
            .is_some_and(|reservation| reservation.ordinal >= ordinal)
        {
            return Err(OperationPolicyError::InvalidChildOrdinal);
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
            let required = already_reserved.checked_add(spec.allowed).ok_or(
                OperationPolicyError::AccountingOverflow {
                    stage: spec.stage,
                    resource: spec.resource,
                },
            )?;
            let available = parent.spec.allowed.saturating_sub(parent.consumed);
            if required > available {
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
                    stage: total_work_stage(),
                    resource: ResourceKind::Work,
                },
            )?;
            let available = self
                .total_work_allowed
                .ok_or(OperationPolicyError::UnknownLimit {
                    stage: total_work_stage(),
                    resource: ResourceKind::Work,
                })?
                .saturating_sub(self.total_work_consumed);
            if required > available {
                return Err(OperationPolicyError::ChildReservationExceeded {
                    stage: total_work_stage(),
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
                    *self = original;
                    return Err(error);
                }
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
                stage: total_work_stage(),
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

    fn ensure_within_reserved_capacity(
        &self,
        index: usize,
        attempted: u64,
    ) -> core::result::Result<(), OperationPolicyError> {
        let entry = self.entries[index];
        let reserved = self.reserved_for(entry.spec.stage, entry.spec.resource)?;
        let usable = entry.spec.allowed.saturating_sub(reserved);
        if attempted > usable {
            return Err(OperationPolicyError::LimitReached(LimitSnapshot {
                stage: entry.spec.stage,
                resource: entry.spec.resource,
                consumed: attempted,
                allowed: usable,
            }));
        }
        Ok(())
    }

    fn reserved_for(
        &self,
        stage: StageId,
        resource: ResourceKind,
    ) -> core::result::Result<u64, OperationPolicyError> {
        self.reservations
            .iter()
            .filter_map(|reservation| {
                reservation
                    .plan
                    .limits
                    .iter()
                    .find(|spec| spec.stage == stage && spec.resource == resource)
                    .map(|spec| spec.allowed)
            })
            .try_fold(0_u64, |sum, value| {
                sum.checked_add(value)
                    .ok_or(OperationPolicyError::AccountingOverflow { stage, resource })
            })
    }

    fn reserved_total_work(&self) -> core::result::Result<u64, OperationPolicyError> {
        self.reservations
            .iter()
            .filter_map(|reservation| reservation.plan.total_work_allowed)
            .try_fold(0_u64, |sum, value| {
                sum.checked_add(value)
                    .ok_or(OperationPolicyError::AccountingOverflow {
                        stage: total_work_stage(),
                        resource: ResourceKind::Work,
                    })
            })
    }
}

const fn total_work_stage() -> StageId {
    // Kept private: it identifies the ledger's synthetic aggregate ceiling.
    match StageId::new("kcore.operation.total-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid internal stage identifier"),
    }
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
