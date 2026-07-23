//! Charged analytic-cylinder discovery and exact pair routing.

use kcore::operation::{AccountingMode, OperationScope, ResourceKind};
use kgeom::vec::Vec3;
use ktopo::geom::SurfaceGeom;

use super::parallel_cylinder_relation::vectors_are_exactly_parallel;
use crate::BodyId;
use crate::error::{Error, Result};
use crate::session::PartEdit;

/// Cylinder carriers retained by the already-charged source dispatch scan.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CylinderOperandScan {
    mask: [bool; 2],
    axes: [Option<Vec3>; 2],
    axes_consistent: [bool; 2],
}

impl CylinderOperandScan {
    /// Whether each operand contains an analytic Cylinder carrier.
    pub(crate) const fn mask(self) -> [bool; 2] {
        self.mask
    }

    /// Exact parallel/nonparallel routing for a two-Cylinder pair.
    pub(crate) fn pair_axes_exactly_parallel(self) -> Option<bool> {
        if !self
            .axes_consistent
            .into_iter()
            .all(|consistent| consistent)
        {
            return None;
        }
        let [Some(first), Some(second)] = self.axes else {
            return None;
        };
        Some(vectors_are_exactly_parallel(first, second))
    }
}

/// Detect cylinder carriers under the enclosing operation's source budget.
pub(crate) fn cylinder_operands_in_scope(
    edit: &PartEdit<'_>,
    bodies: [&BodyId; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<CylinderOperandScan> {
    scope
        .ledger()
        .require_limit(
            super::extract::PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )
        .map_err(Error::from)?;
    let mut mask = [false; 2];
    let mut axes = [None; 2];
    let mut axes_consistent = [true; 2];
    for (operand, body) in bodies.into_iter().enumerate() {
        let faces = edit
            .state
            .store
            .faces_of_body(body.raw())
            .map_err(|source| Error::InconsistentTopology { source })?;
        charge_source_scan(scope, faces.len())?;
        for face_id in faces {
            let face = edit
                .state
                .store
                .get(face_id)
                .map_err(|source| Error::InconsistentTopology { source })?;
            let SurfaceGeom::Cylinder(cylinder) = edit
                .state
                .store
                .surface(face.surface())
                .map_err(|source| Error::InconsistentTopology { source })?
            else {
                continue;
            };
            mask[operand] = true;
            retain_consistent_axis(
                &mut axes[operand],
                &mut axes_consistent[operand],
                cylinder.frame().z(),
            );
        }
    }
    Ok(CylinderOperandScan {
        mask,
        axes,
        axes_consistent,
    })
}

fn retain_consistent_axis(axis: &mut Option<Vec3>, consistent: &mut bool, candidate: Vec3) {
    let Some(retained) = *axis else {
        *axis = Some(candidate);
        return;
    };
    if !vectors_are_exactly_parallel(retained, candidate) {
        *consistent = false;
    }
}

fn charge_source_scan(scope: &mut OperationScope<'_, '_>, amount: usize) -> Result<()> {
    let amount = u64::try_from(amount).map_err(|_| Error::Core {
        source: kcore::error::Error::InvalidGeometry {
            reason: "curved Boolean source dispatch exceeds u64 accounting",
        },
    })?;
    scope
        .ledger_mut()
        .charge(super::extract::PLANAR_SOURCE_EXTRACTION_WORK, amount)
        .map_err(Error::from)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pair(first: Vec3, second: Vec3) -> CylinderOperandScan {
        CylinderOperandScan {
            mask: [true, true],
            axes: [Some(first), Some(second)],
            axes_consistent: [true, true],
        }
    }

    #[test]
    fn pair_axis_routing_distinguishes_parallel_antiparallel_and_tiny_tilt() {
        let z = Vec3::new(0.0, 0.0, 1.0);
        assert_eq!(pair(z, z).pair_axes_exactly_parallel(), Some(true));
        assert_eq!(pair(z, -z).pair_axes_exactly_parallel(), Some(true));
        assert_eq!(
            pair(z, Vec3::new(1.0, 0.0, 0.0)).pair_axes_exactly_parallel(),
            Some(false)
        );
        assert_eq!(
            pair(z, Vec3::new(f64::EPSILON, 0.0, 1.0)).pair_axes_exactly_parallel(),
            Some(false)
        );
    }

    #[test]
    fn inconsistent_operand_axes_fail_closed_in_every_discovery_order() {
        let z = Vec3::new(0.0, 0.0, 1.0);
        let x = Vec3::new(1.0, 0.0, 0.0);
        for candidates in [[z, -z, x], [x, z, -z]] {
            let mut retained = None;
            let mut consistent = true;
            for candidate in candidates {
                retain_consistent_axis(&mut retained, &mut consistent, candidate);
            }
            let scan = CylinderOperandScan {
                mask: [true, true],
                axes: [retained, Some(z)],
                axes_consistent: [consistent, true],
            };
            assert_eq!(scan.pair_axes_exactly_parallel(), None);
        }
    }
}
