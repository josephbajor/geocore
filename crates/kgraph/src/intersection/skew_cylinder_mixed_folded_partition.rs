//! Partition mixed folded support by certified roots and the authored seam.
//!
//! Root order and positive-component membership come from the sealed cyclic
//! theorem. Each resulting cell must fit a finite half-angle chart; metric
//! insets only choose guarded representatives, never decide connectivity.

use super::*;

pub(super) type FoldedBranchSpec = (
    SkewCylinderSheet,
    SkewCylinderHalfAngleChart,
    crate::exact::bounded_polynomial::RootBracket,
    ParamRange,
    [PersistentSkewCylinderFoldedSupportEndpoint; 2],
);

#[derive(Clone, Copy)]
struct Boundary {
    angular: ParamRange,
    root: Option<SkewCylinderDiscriminantRoot>,
    endpoint: PersistentSkewCylinderFoldedSupportEndpoint,
}

impl Boundary {
    fn root(
        root: SkewCylinderDiscriminantRoot,
        endpoint: PersistentSkewCylinderFoldedSupportEndpoint,
    ) -> Self {
        let angular = root.angular_bracket();
        Self {
            angular: ParamRange::new(angular.lo, angular.hi),
            root: Some(root),
            endpoint,
        }
    }

    fn seam(longitude: f64) -> Self {
        Self {
            angular: ParamRange::new(longitude, longitude),
            root: None,
            endpoint: PersistentSkewCylinderFoldedSupportEndpoint::Seam(SkewCylinderSheet::Lower),
        }
    }
}

pub(super) struct MixedFoldedCell {
    boundaries: [Boundary; 2],
    chart: SkewCylinderHalfAngleChart,
}

/// Partition the exact positive component at every interior touching root and
/// seam. A cell that crosses both chart poles remains unsupported.
pub(super) fn mixed_folded_cells(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
) -> Option<Vec<MixedFoldedCell>> {
    use PersistentSkewCylinderFoldedSupportEndpoint::{Root, TouchingRoot};
    let ordinal = topology.interior_touching_root_ordinal()?;
    let root = *topology.topology().roots().get(ordinal)?;
    // The repeated-root corridor currently requires a finite tangent
    // coordinate. A pole root needs a corridor reissued in the other chart;
    // do not let chart selection advertise that unresolved proof as usable.
    tangent_projective_interval(root.bracket())?;
    let touching = Boundary::root(
        root,
        TouchingRoot {
            root_ordinal: ordinal,
            continuation: 0,
        },
    );
    if touching.angular.lo <= 0.0 || touching.angular.hi >= TAU {
        return None;
    }
    let [first, second] = topology.roots();
    let first = Boundary::root(first, Root(0));
    let second = Boundary::root(second, Root(1));
    let components = match topology.positive_cell() {
        SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots => vec![[first, second]],
        SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam => {
            // A simple root on the seam needs a period-lifted root guard.
            if first.angular.lo <= 0.0 || second.angular.hi >= TAU {
                return None;
            }
            vec![[Boundary::seam(0.0), first], [second, Boundary::seam(TAU)]]
        }
    };
    let mut cells = Vec::new();
    let mut touching_splits = 0;
    for [start, end] in components {
        let pieces = if start.angular.hi < touching.angular.lo
            && touching.angular.hi < end.angular.lo
        {
            touching_splits += 1;
            let after = Boundary {
                endpoint: TouchingRoot {
                    root_ordinal: ordinal,
                    continuation: 1,
                },
                ..touching
            };
            vec![[start, touching], [after, end]]
        } else if touching.angular.hi < start.angular.lo || end.angular.hi < touching.angular.lo {
            vec![[start, end]]
        } else {
            return None;
        };
        for boundaries @ [start, end] in pieces {
            let chart = if end.angular.hi < core::f64::consts::PI
                || start.angular.lo > core::f64::consts::PI
            {
                SkewCylinderHalfAngleChart::Tangent
            } else if start.angular.lo > 0.0 && end.angular.hi < TAU {
                SkewCylinderHalfAngleChart::Cotangent
            } else {
                return None;
            };
            cells.push(MixedFoldedCell { boundaries, chart });
        }
    }
    (touching_splits == 1).then_some(cells)
}

pub(super) fn mixed_folded_branch_specs(
    topology: &SkewCylinderFoldedSupportTopologyCertificate,
    authored: ParamRange,
    tolerance: f64,
) -> Result<Vec<FoldedBranchSpec>, IntersectionCertificateError> {
    if authored.lo.to_bits() != 0.0_f64.to_bits() || authored.hi.to_bits() != TAU.to_bits() {
        return Err(unsupported());
    }
    let cells = mixed_folded_cells(topology).ok_or_else(unsupported)?;
    let radius = topology
        .formula_cylinders()
        .into_iter()
        .map(|c| c.radius())
        .fold(0.0, f64::max);
    let inset = tolerance / (64.0 * radius);
    if !inset.is_finite() || inset <= 0.0 {
        return Err(unsupported());
    }
    let mut specs = Vec::with_capacity(2 * cells.len());
    for sheet in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper] {
        for cell in &cells {
            let [start, end] = cell.boundaries;
            let range = ParamRange::new(
                guarded_boundary(start, cell.chart, true, inset).ok_or_else(unsupported)?,
                guarded_boundary(end, cell.chart, false, inset).ok_or_else(unsupported)?,
            );
            if !strict_guarded_range(range, authored)
                || range.lo <= start.angular.hi
                || range.hi >= end.angular.lo
            {
                return Err(unsupported());
            }
            let guard = projective_guard(cell.chart, range).ok_or_else(unsupported)?;
            let endpoints = cell.boundaries.map(|boundary| match boundary.endpoint {
                PersistentSkewCylinderFoldedSupportEndpoint::Seam(_) => {
                    PersistentSkewCylinderFoldedSupportEndpoint::Seam(sheet)
                }
                endpoint => endpoint,
            });
            specs.push((sheet, cell.chart, guard, range, endpoints));
        }
    }
    Ok(specs)
}

fn guarded_boundary(
    boundary: Boundary,
    chart: SkewCylinderHalfAngleChart,
    start: bool,
    inset: f64,
) -> Option<f64> {
    use PersistentSkewCylinderFoldedSupportEndpoint::{Root, Seam, TouchingRoot};
    match boundary.endpoint {
        Seam(_) => Some(if start {
            0.0_f64.next_up()
        } else {
            (TAU - inset).next_down()
        }),
        TouchingRoot { .. } => {
            let angle = boundary.root?.angular_bracket().representative();
            Some(if start {
                (angle + inset).next_up()
            } else {
                (angle - inset).next_down()
            })
        }
        Root(_) => {
            let root = boundary.root?;
            let projective = match chart {
                SkewCylinderHalfAngleChart::Tangent => tangent_projective_interval(root.bracket())?,
                SkewCylinderHalfAngleChart::Cotangent => {
                    cotangent_projective_interval(root.bracket())?
                }
            };
            let increasing = chart == SkewCylinderHalfAngleChart::Tangent;
            let value = if start == increasing {
                projective.hi()
            } else {
                projective.lo()
            };
            // Exact pole roots need a quantitative guard: the adjacent
            // subnormal would underflow the stored positive radicand.
            let guarded = if value == 0.0 {
                if start == increasing {
                    f64::EPSILON / 16.0
                } else {
                    -f64::EPSILON / 16.0
                }
            } else if start == increasing {
                value.next_up()
            } else {
                value.next_down()
            };
            let angle = match chart {
                SkewCylinderHalfAngleChart::Tangent => {
                    let angle = 2.0 * kcore::math::atan2(guarded, 1.0);
                    if boundary.angular.lo > core::f64::consts::PI {
                        angle + TAU
                    } else {
                        angle
                    }
                }
                SkewCylinderHalfAngleChart::Cotangent => 2.0 * kcore::math::atan2(1.0, guarded),
            };
            Some(if start {
                angle.next_up()
            } else {
                angle.next_down()
            })
        }
        _ => None,
    }
}
