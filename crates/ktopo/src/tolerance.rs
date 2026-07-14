//! Entity tolerance values with durable origin and growth provenance.
//!
//! Metric tolerance is data, not a global epsilon. Imported tolerant entities
//! retain their origin, while modeling operations may enlarge tolerances only
//! through transaction-owned budgets recorded in the committed journal.
//! Growth is measured above the session linear-resolution floor: turning an
//! exact entity into a resolution-tolerant entity consumes zero growth, while
//! every enlargement beyond that floor is charged.

use kcore::error::{Error, Result};
use kcore::tolerance::Tolerances;

/// Where an entity first acquired a non-null tolerance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ToleranceOrigin {
    /// Read from a Parasolid XT entity tolerance field.
    ImportedXt,
    /// Created by a kernel operation with this stable operation name.
    Operation(&'static str),
}

/// A validated per-entity metric tolerance and its retained provenance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EntityTolerance {
    value: f64,
    origin: ToleranceOrigin,
    origin_value: f64,
    accumulated_growth: f64,
    last_operation: Option<&'static str>,
}

impl EntityTolerance {
    /// Tolerance read from a Parasolid XT entity field.
    pub fn imported_xt(value: f64) -> Result<Self> {
        Self::new(value, ToleranceOrigin::ImportedXt)
    }

    /// Tolerance first created by a named modeling operation.
    pub fn operation(value: f64, operation: &'static str) -> Result<Self> {
        if operation.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "tolerance operation name must not be empty",
            });
        }
        Self::new(value, ToleranceOrigin::Operation(operation))
    }

    fn new(value: f64, origin: ToleranceOrigin) -> Result<Self> {
        Tolerances::default().entity_tolerance(value)?;
        Ok(Self {
            value,
            origin,
            origin_value: value,
            accumulated_growth: 0.0,
            last_operation: match origin {
                ToleranceOrigin::ImportedXt => None,
                ToleranceOrigin::Operation(operation) => Some(operation),
            },
        })
    }

    /// Current metric tolerance in model units.
    pub fn value(self) -> f64 {
        self.value
    }

    /// Durable source that first introduced the tolerance.
    pub fn origin(self) -> ToleranceOrigin {
        self.origin
    }

    /// Value when this entity first became tolerant.
    pub fn origin_value(self) -> f64 {
        self.origin_value
    }

    /// Sum of committed enlargements since the tolerance was introduced.
    pub fn accumulated_growth(self) -> f64 {
        self.accumulated_growth
    }

    /// Most recent operation that introduced or enlarged this tolerance.
    pub fn last_operation(self) -> Option<&'static str> {
        self.last_operation
    }

    pub(crate) fn grown_to(self, value: f64, operation: &'static str) -> Result<Self> {
        Tolerances::default().entity_tolerance(value)?;
        if operation.is_empty() {
            return Err(Error::InvalidGeometry {
                reason: "tolerance operation name must not be empty",
            });
        }
        if value < self.value {
            return Err(Error::InvalidGeometry {
                reason: "tolerance growth cannot reduce an entity tolerance",
            });
        }
        Ok(Self {
            value,
            accumulated_growth: value - self.origin_value,
            last_operation: Some(operation),
            ..self
        })
    }

    /// Select the larger inherited tolerance without manufacturing growth.
    ///
    /// The returned index identifies the selected input. Equal tolerant
    /// values deterministically select the first input, and two exact inputs
    /// return `(None, None)`.
    pub(crate) fn inherited_max_with_source(
        inputs: [Option<Self>; 2],
    ) -> (Option<usize>, Option<Self>) {
        match inputs {
            [Some(first), Some(second)] if second.value > first.value => (Some(1), Some(second)),
            [Some(first), Some(_)] | [Some(first), None] => (Some(0), Some(first)),
            [None, Some(second)] => (Some(1), Some(second)),
            [None, None] => (None, None),
        }
    }

    #[cfg(test)]
    pub(crate) fn unchecked(value: f64) -> Self {
        Self {
            value,
            origin: ToleranceOrigin::Operation("malformed-test-input"),
            origin_value: value,
            accumulated_growth: 0.0,
            last_operation: Some("malformed-test-input"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kcore::tolerance::LINEAR_RESOLUTION;

    #[test]
    fn origin_survives_later_growth() {
        let imported = EntityTolerance::imported_xt(LINEAR_RESOLUTION * 2.0).unwrap();
        let grown = imported.grown_to(LINEAR_RESOLUTION * 5.0, "sew").unwrap();
        assert_eq!(grown.origin(), ToleranceOrigin::ImportedXt);
        assert_eq!(grown.origin_value(), LINEAR_RESOLUTION * 2.0);
        assert_eq!(
            grown.accumulated_growth(),
            LINEAR_RESOLUTION * 5.0 - LINEAR_RESOLUTION * 2.0
        );
        assert_eq!(grown.last_operation(), Some("sew"));
    }

    #[test]
    fn inherited_max_selects_larger_and_breaks_ties_toward_first() {
        let imported = EntityTolerance::imported_xt(LINEAR_RESOLUTION * 2.0).unwrap();
        let operation = EntityTolerance::operation(LINEAR_RESOLUTION * 3.0, "split").unwrap();
        assert_eq!(
            EntityTolerance::inherited_max_with_source([Some(imported), Some(operation)]),
            (Some(1), Some(operation))
        );
        assert_eq!(
            EntityTolerance::inherited_max_with_source([Some(operation), Some(operation)]),
            (Some(0), Some(operation))
        );
        assert_eq!(
            EntityTolerance::inherited_max_with_source([None, None]),
            (None, None)
        );
    }
}
