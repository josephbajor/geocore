//! Exact periodic-face proof for bounded skew-cylinder pcurves.
//!
//! Every proof path consumes the sealed 256 guarded cells and both physical-
//! root continuation corridors. Topology uses exact-source enclosures. Stored
//! evaluator enclosures must independently support the same monotone and
//! separation decisions; guarded numeric evaluations remain trim evidence
//! only.

use super::*;

use crate::section::{
    SectionSkewCylinderBranchPcurve, SectionSkewCylinderInterval,
    SectionSkewCylinderPcurveCellCertificate, SectionSkewCylinderPcurveEnclosure,
};

pub(super) const FRAGMENT_WORK: u64 = kgraph::SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK
    + 2 * kgraph::SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK;

#[derive(Clone, Copy)]
enum CorridorKind {
    Guarded,
    PhysicalRoot,
}

#[derive(Clone, Copy)]
struct ProofCell {
    parameter: Interval,
    integration_length: Interval,
    stored_uv: [Interval; 2],
    source_uv: [Interval; 2],
    stored_derivative: [Interval; 2],
    source_derivative: [Interval; 2],
    kind: CorridorKind,
}

impl ProofCell {
    fn shifted(mut self, delta: Interval) -> Self {
        self.stored_uv[0] = self.stored_uv[0] + delta;
        self.source_uv[0] = self.source_uv[0] + delta;
        self
    }

    fn source_bounds(self) -> Bounds2 {
        Bounds2 {
            u: self.source_uv[0],
            v: self.source_uv[1],
        }
    }

    fn stored_bounds(self) -> Bounds2 {
        Bounds2 {
            u: self.stored_uv[0],
            v: self.stored_uv[1],
        }
    }

    fn orientation_contribution(self) -> Option<Interval> {
        let integrand = self.source_uv[0] * self.source_derivative[1]
            - self.source_uv[1] * self.source_derivative[0];
        match self.kind {
            CorridorKind::Guarded
                if self.integration_length.lo() >= 0.0 && self.integration_length.hi() > 0.0 => {}
            // The exact physical root may occupy any point in its isolating
            // interval, so its suffix/prefix length must include zero.
            CorridorKind::PhysicalRoot if self.integration_length.lo() == 0.0 => {}
            _ => return None,
        }
        Some(integrand * self.integration_length)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum StrictSign {
    Negative,
    Positive,
}

impl StrictSign {
    fn contains(self, value: Interval) -> bool {
        match self {
            Self::Negative => value.hi() < 0.0,
            Self::Positive => value.lo() > 0.0,
        }
    }
}

#[derive(Clone)]
pub(super) struct ProceduralFragmentProof {
    fragment: usize,
    endpoint_ids: [usize; 2],
    physical_uv: [[Interval; 2]; 2],
    physical_stored_derivatives: [[Interval; 2]; 2],
    physical_derivatives: [[Interval; 2]; 2],
    guarded_parameters: [f64; 2],
    guarded_points: [Point3; 2],
    carrier: SectionCarrier,
    pcurve: SectionSkewCylinderBranchPcurve,
    cells: Vec<ProofCell>,
    monotone_coordinate: usize,
    monotone_sign: StrictSign,
    period_shift: i64,
}

impl ProceduralFragmentProof {
    pub(super) fn certify(
        fragment_index: usize,
        fragment: &SectionCurveFragment,
        branch: &SectionBranch,
        operand: usize,
    ) -> Result<Self, SectionPeriodicEmbeddingGap> {
        let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = fragment.span() else {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceUnavailable {
                    fragment: fragment_index,
                },
            );
        };
        let certificate = branch.embedding_certificate().ok_or(
            SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceUnavailable {
                fragment: fragment_index,
            },
        )?;
        let SectionUvCurve::SkewCylinderBranch(pcurve) = branch.pcurves()[operand] else {
            return Err(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve {
                fragment: fragment_index,
            });
        };
        let SectionCarrier::SkewCylinderBranch(_) = branch.carrier() else {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        };
        if certificate.range() != branch.range()
            || pcurve.range() != branch.range()
            || certificate.guarded_cell_count() != kgraph::SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS
            || certificate.total_work() != FRAGMENT_WORK
        {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }

        let endpoint_ids = endpoints.each_ref().map(|end| end.endpoint());
        let guarded_parameters = endpoints
            .each_ref()
            .map(|end| end.inside_carrier_parameter());
        let guarded_points = endpoints.each_ref().map(|end| end.inside_point());
        if guarded_parameters.map(f64::to_bits)
            != [branch.range().lo.to_bits(), branch.range().hi.to_bits()]
        {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }

        let roots = [certificate.root_corridor(0), certificate.root_corridor(1)];
        let [Some(start_root), Some(end_root)] = roots else {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        };
        if start_root.section_end() != 0
            || end_root.section_end() != 1
            || start_root.work() != certificate.root_corridor_work()
            || end_root.work() != certificate.root_corridor_work()
        {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }
        let start_parameter = interval(start_root.root_parameter());
        let end_parameter = interval(end_root.root_parameter());
        if start_parameter.hi() >= branch.range().lo || end_parameter.lo() <= branch.range().hi {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }

        let mut cells = Vec::with_capacity(certificate.guarded_cell_count() + 2);
        cells.push(cell(
            fragment_index,
            start_root.corridor(),
            operand,
            CorridorKind::PhysicalRoot,
        )?);
        let mut guarded_work = 0_u64;
        for index in 0..certificate.guarded_cell_count() {
            let issued = certificate.guarded_cell(index).ok_or(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            )?;
            guarded_work = guarded_work.checked_add(issued.work()).ok_or(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            )?;
            cells.push(cell(
                fragment_index,
                issued,
                operand,
                CorridorKind::Guarded,
            )?);
        }
        cells.push(cell(
            fragment_index,
            end_root.corridor(),
            operand,
            CorridorKind::PhysicalRoot,
        )?);
        if !assign_integration_lengths(&mut cells, branch.range(), start_parameter, end_parameter) {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }
        if guarded_work != certificate.all_guarded_cells_work()
            || !contiguous_parameters(&cells)
            || !contains_interval(cells[0].parameter, start_parameter)
            || !cells[0].parameter.contains(branch.range().lo)
            || !cells[1].parameter.contains(branch.range().lo)
            || !cells[cells.len() - 2].parameter.contains(branch.range().hi)
            || !cells[cells.len() - 1].parameter.contains(branch.range().hi)
            || !contains_interval(cells[cells.len() - 1].parameter, end_parameter)
        {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed {
                    fragment: fragment_index,
                },
            );
        }

        let physical = [start_root, end_root].map(|root| root.root_pcurves()[operand]);
        let physical_uv = physical
            .each_ref()
            .map(|value| intervals(value.source_uv()));
        let physical_stored_derivatives = physical
            .each_ref()
            .map(|value| intervals(value.stored_derivative()));
        let physical_derivatives = physical
            .each_ref()
            .map(|value| intervals(value.source_derivative()));
        let Some((monotone_coordinate, monotone_sign)) =
            fixed_monotone_coordinate(&cells, &physical)
        else {
            return Err(
                SectionPeriodicEmbeddingGap::ProceduralPcurveMonotonicityIndeterminate {
                    fragment: fragment_index,
                },
            );
        };

        Ok(Self {
            fragment: fragment_index,
            endpoint_ids,
            physical_uv,
            physical_stored_derivatives,
            physical_derivatives,
            guarded_parameters,
            guarded_points,
            carrier: branch.carrier(),
            pcurve,
            cells,
            monotone_coordinate,
            monotone_sign,
            period_shift: 0,
        })
    }

    pub(super) const fn endpoint_ids(&self) -> [usize; 2] {
        self.endpoint_ids
    }

    pub(super) const fn physical_uv(&self) -> [[Interval; 2]; 2] {
        self.physical_uv
    }

    pub(super) const fn guarded_parameters(&self) -> [f64; 2] {
        self.guarded_parameters
    }

    pub(super) fn shift(&mut self, shift: i64) -> bool {
        let Some(delta) = integer_period_interval(shift) else {
            return false;
        };
        let Some(period_shift) = self.period_shift.checked_add(shift) else {
            return false;
        };
        self.cells = self
            .cells
            .iter()
            .copied()
            .map(|cell| cell.shifted(delta))
            .collect();
        for endpoint in &mut self.physical_uv {
            endpoint[0] = endpoint[0] + delta;
        }
        self.period_shift = period_shift;
        true
    }

    pub(super) fn endpoint_derivative(&self, end: usize) -> Option<[Interval; 2]> {
        let derivative = self.physical_derivatives.get(end).copied()?;
        self.monotone_sign
            .contains(derivative[self.monotone_coordinate])
            .then_some(derivative)
    }

    /// Prove that two Section-forward fragments sharing `self`'s high root
    /// and `other`'s low root cannot meet anywhere else. The same exact
    /// coordinate is strictly monotone in the same direction over both
    /// complete physical-root domains, so the first fragment lies strictly
    /// before the shared coordinate and the second lies strictly after it.
    pub(super) fn certifies_forward_join(&self, other: &Self) -> bool {
        self.monotone_coordinate == other.monotone_coordinate
            && self.monotone_sign == other.monotone_sign
    }

    /// Prove that the complete nonlinear fragment crosses an infinite affine
    /// carrier at most once. The affine line equation has a derivative with
    /// one fixed strict sign over every stored and exact-source cell, including
    /// both physical roots; its topology-owned zero is therefore unique.
    pub(super) fn certifies_unique_line_intersection(&self, line_direction: [f64; 2]) -> bool {
        if line_direction.into_iter().any(|value| !value.is_finite()) {
            return false;
        }
        [StrictSign::Negative, StrictSign::Positive]
            .into_iter()
            .any(|sign| {
                self.cells.iter().all(|cell| {
                    sign.contains(line_derivative(line_direction, cell.stored_derivative))
                        && sign.contains(line_derivative(line_direction, cell.source_derivative))
                }) && (0..2).all(|end| {
                    sign.contains(line_derivative(
                        line_direction,
                        self.physical_stored_derivatives[end],
                    )) && sign.contains(line_derivative(
                        line_direction,
                        self.physical_derivatives[end],
                    ))
                })
            })
    }

    pub(super) fn orientation_integral(&self) -> Option<Interval> {
        self.cells
            .iter()
            .copied()
            .try_fold(Interval::point(0.0), |sum, cell| {
                Some(sum + cell.orientation_contribution()?)
            })
    }

    pub(super) fn trim_scalars(
        &self,
    ) -> Result<[SectionCarrierTrimScalarEvidence; 2], SectionPeriodicEmbeddingGap> {
        Ok([self.trim_scalar(0)?, self.trim_scalar(1)?])
    }

    fn trim_scalar(
        &self,
        end: usize,
    ) -> Result<SectionCarrierTrimScalarEvidence, SectionPeriodicEmbeddingGap> {
        let endpoint = self.endpoint_ids[end];
        let parameter = self.guarded_parameters[end];
        let point = carrier_point(self.carrier, parameter).ok_or(
            SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
                fragment: self.fragment,
                endpoint,
            },
        )?;
        if point.to_array().map(f64::to_bits)
            != self.guarded_points[end].to_array().map(f64::to_bits)
        {
            return Err(SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
                fragment: self.fragment,
                endpoint,
            });
        }
        let evaluated = self.pcurve.eval(parameter);
        if !evaluated.x.is_finite() || !evaluated.y.is_finite() {
            return Err(SectionPeriodicEmbeddingGap::CarrierTrimScalarUnavailable {
                fragment: self.fragment,
                endpoint,
            });
        }
        let corridor = if end == 0 {
            &self.cells[0]
        } else {
            &self.cells[self.cells.len() - 1]
        };
        let guarded = if end == 0 {
            &self.cells[1]
        } else {
            &self.cells[self.cells.len() - 2]
        };
        let delta = integer_period_interval(self.period_shift).ok_or(
            SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                fragment: self.fragment,
                endpoint,
            },
        )?;
        let raw = [evaluated.x, evaluated.y];
        let lifted_raw = [Interval::point(raw[0]) + delta, Interval::point(raw[1])];
        let mut lifted_uv = lifted_raw;
        for axis in 0..2 {
            let corridor_uv = corridor.stored_uv[axis];
            let guarded_uv = guarded.stored_uv[axis];
            if !corridor_uv.intersects(lifted_raw[axis]) || !guarded_uv.intersects(lifted_raw[axis])
            {
                return Err(
                    SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                        fragment: self.fragment,
                        endpoint,
                    },
                );
            }
            // The guard evaluation and the two independently outward-rounded
            // cells enclose the same exact stored pcurve image. Their common
            // intersection is a tighter guard-only witness; nesting is not
            // required after reflected reversal arithmetic.
            let lo = corridor_uv
                .lo()
                .max(guarded_uv.lo())
                .max(lifted_raw[axis].lo());
            let hi = corridor_uv
                .hi()
                .min(guarded_uv.hi())
                .min(lifted_raw[axis].hi());
            if lo > hi {
                return Err(
                    SectionPeriodicEmbeddingGap::CarrierTrimScalarPcurveMismatch {
                        fragment: self.fragment,
                        endpoint,
                    },
                );
            }
            lifted_uv[axis] = Interval::new(lo, hi);
        }
        Ok(SectionCarrierTrimScalarEvidence {
            endpoint,
            carrier_parameter: parameter,
            carrier_interval: SectionCarrierParameterInterval::from_interval(Interval::point(
                parameter,
            )),
            point,
            lifted_uv: lifted_uv.map(SectionUvParameterInterval::from_interval),
        })
    }

    pub(super) fn strictly_disjoint_line(&self, line: Bounds2, periodic: bool) -> bool {
        self.cells.iter().all(|cell| {
            disjoint(cell.source_bounds(), line, periodic)
                && disjoint(cell.stored_bounds(), line, periodic)
        })
    }

    pub(super) fn strictly_disjoint(&self, other: &Self, periodic: bool) -> bool {
        self.cells.iter().all(|left| {
            other.cells.iter().all(|right| {
                disjoint(left.source_bounds(), right.source_bounds(), periodic)
                    && disjoint(left.stored_bounds(), right.stored_bounds(), periodic)
            })
        })
    }

    pub(super) fn ordered_before_line(&self, line: Bounds2, axis: usize) -> bool {
        self.cells.iter().all(|cell| {
            bounds_axis(cell.source_bounds(), axis).hi() < bounds_axis(line, axis).lo()
                && bounds_axis(cell.stored_bounds(), axis).hi() < bounds_axis(line, axis).lo()
        })
    }

    pub(super) fn ordered_after_line(&self, line: Bounds2, axis: usize) -> bool {
        self.cells.iter().all(|cell| {
            bounds_axis(line, axis).hi() < bounds_axis(cell.source_bounds(), axis).lo()
                && bounds_axis(line, axis).hi() < bounds_axis(cell.stored_bounds(), axis).lo()
        })
    }

    pub(super) fn ordered_before(&self, other: &Self, axis: usize) -> bool {
        self.cells.iter().all(|left| {
            other.cells.iter().all(|right| {
                bounds_axis(left.source_bounds(), axis).hi()
                    < bounds_axis(right.source_bounds(), axis).lo()
                    && bounds_axis(left.stored_bounds(), axis).hi()
                        < bounds_axis(right.stored_bounds(), axis).lo()
            })
        })
    }
}

fn cell(
    fragment: usize,
    certificate: SectionSkewCylinderPcurveCellCertificate,
    operand: usize,
    kind: CorridorKind,
) -> Result<ProofCell, SectionPeriodicEmbeddingGap> {
    let Some(pcurve) = certificate.pcurves().get(operand) else {
        return Err(SectionPeriodicEmbeddingGap::ProceduralPcurveEvidenceMalformed { fragment });
    };
    if !pcurve.stored_is_strictly_regular() || !pcurve.source_is_strictly_regular() {
        return Err(
            SectionPeriodicEmbeddingGap::ProceduralPcurveMonotonicityIndeterminate { fragment },
        );
    }
    Ok(ProofCell {
        parameter: interval(certificate.parameter()),
        integration_length: Interval::point(0.0),
        stored_uv: intervals(pcurve.stored_uv()),
        source_uv: intervals(pcurve.source_uv()),
        stored_derivative: intervals(pcurve.stored_derivative()),
        source_derivative: intervals(pcurve.source_derivative()),
        kind,
    })
}

fn fixed_monotone_coordinate(
    cells: &[ProofCell],
    physical: &[SectionSkewCylinderPcurveEnclosure; 2],
) -> Option<(usize, StrictSign)> {
    for coordinate in 0..2 {
        for sign in [StrictSign::Negative, StrictSign::Positive] {
            if cells.iter().all(|cell| {
                sign.contains(cell.stored_derivative[coordinate])
                    && sign.contains(cell.source_derivative[coordinate])
            }) && physical.iter().all(|root| {
                sign.contains(intervals(root.stored_derivative())[coordinate])
                    && sign.contains(intervals(root.source_derivative())[coordinate])
            }) {
                return Some((coordinate, sign));
            }
        }
    }
    None
}

fn contiguous_parameters(cells: &[ProofCell]) -> bool {
    !cells.is_empty()
        && cells.iter().all(|cell| {
            cell.parameter.lo().is_finite()
                && cell.parameter.hi().is_finite()
                && cell.parameter.lo() <= cell.parameter.hi()
        })
        && cells.windows(2).all(|pair| {
            // Indexed issuance is sealed in increasing Section order. The
            // reflected outward boxes may share a rounded lo/hi at boundary-
            // graded cells, so continuity requires overlap but not strict
            // floating-endpoint growth.
            pair[0].parameter.intersects(pair[1].parameter)
        })
}

fn assign_integration_lengths(
    cells: &mut [ProofCell],
    range: kgeom::param::ParamRange,
    start_root: Interval,
    end_root: Interval,
) -> bool {
    if cells.len() != kgraph::SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS + 2 {
        return false;
    }

    let start_max = range.lo - start_root.lo();
    let end_max = end_root.hi() - range.hi;
    if !start_max.is_finite() || !end_max.is_finite() || start_max <= 0.0 || end_max <= 0.0 {
        return false;
    }
    cells[0].integration_length = Interval::new(0.0, start_max.next_up());
    let last = cells.len() - 1;
    cells[last].integration_length = Interval::new(0.0, end_max.next_up());

    let guarded_count = cells.len() - 2;
    let mut boundaries = Vec::with_capacity(guarded_count + 1);
    boundaries.push(Interval::point(range.lo));
    for boundary in 1..guarded_count {
        let left = cells[boundary].parameter;
        let right = cells[boundary + 1].parameter;
        let lo = left.lo().max(right.lo());
        let hi = left.hi().min(right.hi());
        if lo > hi {
            return false;
        }
        boundaries.push(Interval::new(lo, hi));
    }
    boundaries.push(Interval::point(range.hi));

    for index in 0..guarded_count {
        let length = boundaries[index + 1] - boundaries[index];
        if !length.lo().is_finite() || !length.hi().is_finite() || length.hi() <= 0.0 {
            return false;
        }
        cells[index + 1].integration_length = Interval::new(length.lo().max(0.0), length.hi());
    }
    true
}

fn disjoint(first: Bounds2, second: Bounds2, periodic: bool) -> bool {
    if periodic {
        periodically_strictly_disjoint(first, second)
    } else {
        strictly_disjoint(first, second)
    }
}

fn bounds_axis(bounds: Bounds2, axis: usize) -> Interval {
    if axis == 0 { bounds.u } else { bounds.v }
}

fn contains_interval(container: Interval, value: Interval) -> bool {
    container.lo() <= value.lo() && value.hi() <= container.hi()
}

fn line_derivative(line_direction: [f64; 2], curve_derivative: [Interval; 2]) -> Interval {
    Interval::point(line_direction[0]) * curve_derivative[1]
        - Interval::point(line_direction[1]) * curve_derivative[0]
}

fn interval(value: SectionSkewCylinderInterval) -> Interval {
    Interval::new(value.lo(), value.hi())
}

fn intervals(values: &[SectionSkewCylinderInterval; 2]) -> [Interval; 2] {
    values.map(interval)
}
