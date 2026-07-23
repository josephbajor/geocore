//! Indexed interval cells for certified skew-cylinder pcurves.
//!
//! The retained branch certificate stays compact.  Arrangement consumers
//! recertify any guarded cell by index and debit one logical work unit.  A
//! physical-root continuation recertifies both the isolating root interval
//! and its closed corridor to the selected guarded end, for two logical work
//! units.  No point samples participate in either proof.

use super::*;

/// Logical work for one indexed guarded pcurve cell.
pub const SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK: u64 = 1;

/// Logical work for recertifying every guarded pcurve cell once.
pub const SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK: u64 =
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS as u64 * SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK;

/// Logical work for one physical-root interval plus its continuation corridor.
pub const SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK: u64 = 2;

/// Guarded carrier-range end adjacent to a physical root interval.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SkewCylinderBranchGuardedEnd {
    /// The root interval lies strictly below the retained carrier range.
    Lower,
    /// The root interval lies strictly above the retained carrier range.
    Upper,
}

/// Outward pcurve value and first-derivative enclosures for one source trace.
///
/// `stored_*` encloses the certifier-minted procedural evaluator. `source_*`
/// independently encloses the same construction with every determinant and
/// derived coefficient interpreted through exact-source interval arithmetic.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchPcurveEnclosure {
    operand: usize,
    stored_uv: [Interval; 2],
    stored_derivative: [Interval; 2],
    source_uv: [Interval; 2],
    source_derivative: [Interval; 2],
}

impl SkewCylinderBranchPcurveEnclosure {
    /// Canonical source operand represented by this enclosure.
    pub const fn operand(self) -> usize {
        self.operand
    }

    /// Procedural-evaluator longitude/height enclosure.
    pub const fn stored_uv(self) -> [Interval; 2] {
        self.stored_uv
    }

    /// Procedural-evaluator first-derivative enclosure.
    pub const fn stored_derivative(self) -> [Interval; 2] {
        self.stored_derivative
    }

    /// Exact-source longitude/height enclosure.
    pub const fn source_uv(self) -> [Interval; 2] {
        self.source_uv
    }

    /// Exact-source first-derivative enclosure.
    pub const fn source_derivative(self) -> [Interval; 2] {
        self.source_derivative
    }

    /// Whether the stored derivative box excludes the zero vector.
    pub fn stored_is_strictly_regular(self) -> bool {
        derivative_box_is_regular(self.stored_derivative)
    }

    /// Whether the exact-source derivative box excludes the zero vector.
    pub fn source_is_strictly_regular(self) -> bool {
        derivative_box_is_regular(self.source_derivative)
    }
}

/// One sealed indexed cell of a retained skew-cylinder branch certificate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchPcurveCellCertificate {
    parameter: Interval,
    pcurves: [SkewCylinderBranchPcurveEnclosure; 2],
}

impl SkewCylinderBranchPcurveCellCertificate {
    /// Closed canonical carrier interval covered by this cell.
    pub const fn parameter(self) -> Interval {
        self.parameter
    }

    /// Pcurve enclosures in the certificate's current trace order.
    pub const fn pcurves(self) -> [SkewCylinderBranchPcurveEnclosure; 2] {
        self.pcurves
    }

    /// Fixed caller work debit for this cell.
    pub const fn work(self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_CELL_WORK
    }
}

/// Exact-root and root-to-guard pcurve evidence in one retained chart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SkewCylinderBranchPcurveRootCorridorCertificate {
    guarded_end: SkewCylinderBranchGuardedEnd,
    root_parameter: Interval,
    root_pcurves: [SkewCylinderBranchPcurveEnclosure; 2],
    corridor: SkewCylinderBranchPcurveCellCertificate,
}

impl SkewCylinderBranchPcurveRootCorridorCertificate {
    /// Guarded end joined by this continuation.
    pub const fn guarded_end(self) -> SkewCylinderBranchGuardedEnd {
        self.guarded_end
    }

    /// Caller-provided exact-source root enclosure in canonical longitude.
    pub const fn root_parameter(self) -> Interval {
        self.root_parameter
    }

    /// Root-only pcurve enclosures in current trace order.
    pub const fn root_pcurves(self) -> [SkewCylinderBranchPcurveEnclosure; 2] {
        self.root_pcurves
    }

    /// Closed interval cell from the complete root enclosure to the guard.
    pub const fn corridor(self) -> SkewCylinderBranchPcurveCellCertificate {
        self.corridor
    }

    /// Fixed caller work debit for the root and corridor recertifications.
    pub const fn work(self) -> u64 {
        SKEW_CYLINDER_BRANCH_PCURVE_ROOT_CORRIDOR_WORK
    }
}

impl PairedSkewCylinderBranchResidualCertificate {
    /// Reissue outward UV and first-derivative proof for one retained cell.
    ///
    /// Valid indices are `0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS`. Calling
    /// every index once costs
    /// [`SKEW_CYLINDER_BRANCH_PCURVE_ALL_CELLS_WORK`] logical work units.
    pub fn certify_pcurve_cell(
        &self,
        index: usize,
    ) -> Result<SkewCylinderBranchPcurveCellCertificate, IntersectionCertificateError> {
        if index >= SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
            return Err(unsupported(
                "skew Cylinder/Cylinder pcurve cell index is outside the retained partition",
            ));
        }
        let lo =
            subrange::proof_cell_boundary(self.carrier_range, index, self.boundary_graded_cells);
        let hi = subrange::proof_cell_boundary(
            self.carrier_range,
            index + 1,
            self.boundary_graded_cells,
        );
        certify_interval(self, Interval::new(lo, hi))
    }

    /// Reissue physical-root and root-to-lower-guard pcurve evidence.
    pub fn certify_lower_pcurve_root_corridor(
        &self,
        root_parameter: Interval,
    ) -> Result<SkewCylinderBranchPcurveRootCorridorCertificate, IntersectionCertificateError> {
        self.certify_pcurve_root_corridor(root_parameter, SkewCylinderBranchGuardedEnd::Lower)
    }

    /// Reissue physical-root and root-to-upper-guard pcurve evidence.
    pub fn certify_upper_pcurve_root_corridor(
        &self,
        root_parameter: Interval,
    ) -> Result<SkewCylinderBranchPcurveRootCorridorCertificate, IntersectionCertificateError> {
        self.certify_pcurve_root_corridor(root_parameter, SkewCylinderBranchGuardedEnd::Upper)
    }

    /// Reissue physical-root and root-to-guard pcurve evidence.
    ///
    /// The complete root enclosure must be strictly outside the selected end.
    /// The closed hull from the root to the guard must stay strictly inside
    /// the canonical authored chart. Both pcurves then re-prove positive
    /// stored/exact radicands, regular derivative divisors, and the opposite
    /// longitude lift originally bound into this certificate.
    pub fn certify_pcurve_root_corridor(
        &self,
        root_parameter: Interval,
        guarded_end: SkewCylinderBranchGuardedEnd,
    ) -> Result<SkewCylinderBranchPcurveRootCorridorCertificate, IntersectionCertificateError> {
        if !finite(root_parameter) {
            return Err(IntersectionCertificateError::InvalidCarrierRange);
        }
        let corridor_parameter = match guarded_end {
            SkewCylinderBranchGuardedEnd::Lower if root_parameter.hi() < self.carrier_range.lo => {
                Interval::new(root_parameter.lo(), self.carrier_range.lo)
            }
            SkewCylinderBranchGuardedEnd::Upper if root_parameter.lo() > self.carrier_range.hi => {
                Interval::new(self.carrier_range.hi, root_parameter.hi())
            }
            _ => {
                return Err(unsupported(
                    "skew Cylinder/Cylinder physical-root enclosure is not strictly outside the selected guarded end",
                ));
            }
        };
        let authored = self.chart_windows[0];
        if corridor_parameter.lo() <= authored.lo || corridor_parameter.hi() >= authored.hi {
            return Err(unsupported(
                "skew Cylinder/Cylinder physical-root corridor leaves the retained canonical longitude chart",
            ));
        }
        let root = certify_interval(self, root_parameter)?;
        let corridor = certify_interval(self, corridor_parameter)?;
        Ok(SkewCylinderBranchPcurveRootCorridorCertificate {
            guarded_end,
            root_parameter,
            root_pcurves: root.pcurves,
            corridor,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct RootJetIntervals {
    value: Interval,
    derivative: Interval,
}

#[derive(Debug, Clone, Copy)]
struct PcurvePairIntervals {
    canonical: SkewCylinderBranchPcurveEnclosure,
    opposite: SkewCylinderBranchPcurveEnclosure,
}

fn certify_interval(
    certificate: &PairedSkewCylinderBranchResidualCertificate,
    parameter: Interval,
) -> Result<SkewCylinderBranchPcurveCellCertificate, IntersectionCertificateError> {
    if !finite(parameter) || parameter.width() < 0.0 {
        return Err(IntersectionCertificateError::InvalidCarrierRange);
    }
    let authored = certificate.chart_windows[0];
    if parameter.lo() < authored.lo || parameter.hi() > authored.hi {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve interval leaves the retained canonical longitude chart",
        ));
    }
    let pair = enclose_pair(certificate, parameter)?;
    let canonical = [pair.canonical, pair.opposite];
    let pcurves = certificate
        .traces
        .map(|trace| canonical[trace.pcurve.operand as usize]);
    Ok(SkewCylinderBranchPcurveCellCertificate { parameter, pcurves })
}

fn enclose_pair(
    certificate: &PairedSkewCylinderBranchResidualCertificate,
    parameter: Interval,
) -> Result<PcurvePairIntervals, IntersectionCertificateError> {
    let algebra = certificate.carrier.algebra;
    let coefficients =
        coefficient_proof(algebra).ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    if algebra.a <= 0.0
        || algebra.e == 0.0
        || coefficients.a_true.lo() <= 0.0
        || coefficients.e_true.contains_zero()
    {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell has no regular source/evaluator divisors",
        ));
    }
    let cosine = trig_interval(parameter.lo(), parameter.hi(), false);
    let sine = trig_interval(parameter.lo(), parameter.hi(), true);
    let roots = cell_root_enclosures(algebra, coefficients, cosine, sine).ok_or_else(|| {
        unsupported(
            "skew Cylinder/Cylinder pcurve cell has no positive source/evaluator radicand margin",
        )
    })?;
    let stored_root = root_jet(
        algebra.m,
        algebra.l,
        Interval::point(algebra.a),
        roots.stored_h,
        cosine,
        sine,
        algebra.sheet,
    )?;
    let source_root = interval_root_jet(
        coefficients.m_true,
        coefficients.l_true,
        coefficients.a_true,
        roots.exact_h,
        cosine,
        sine,
        algebra.sheet,
    )?;

    let canonical = SkewCylinderBranchPcurveEnclosure {
        operand: 0,
        stored_uv: [parameter, stored_root.value],
        stored_derivative: [Interval::point(1.0), stored_root.derivative],
        source_uv: [parameter, source_root.value],
        source_derivative: [Interval::point(1.0), source_root.derivative],
    };
    let stored_dual = dual_coordinates(
        [algebra.x0, algebra.y0, algebra.z0],
        [algebra.dx, algebra.dy, algebra.dz].map(Interval::point),
        stored_root,
        cosine,
        sine,
    )?;
    let source_dual = interval_dual_coordinates(
        coefficients.harmonics_true,
        coefficients.directions_true,
        source_root,
        cosine,
        sine,
    )?;
    validate_common_opposite_chart(
        stored_dual,
        Interval::point(algebra.e),
        source_dual,
        coefficients.e_true,
    )?;
    let stored_opposite = opposite_pcurve(
        stored_dual,
        Interval::point(algebra.e),
        algebra.longitude_offset,
        certificate.chart_windows[1],
    )?;
    let source_opposite = opposite_pcurve(
        source_dual,
        coefficients.e_true,
        algebra.longitude_offset,
        certificate.chart_windows[1],
    )?;
    let opposite = SkewCylinderBranchPcurveEnclosure {
        operand: 1,
        stored_uv: stored_opposite.0,
        stored_derivative: stored_opposite.1,
        source_uv: source_opposite.0,
        source_derivative: source_opposite.1,
    };
    if !canonical.stored_is_strictly_regular()
        || !canonical.source_is_strictly_regular()
        || !opposite.stored_is_strictly_regular()
        || !opposite.source_is_strictly_regular()
    {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell has no strict source/evaluator first-derivative margin",
        ));
    }
    Ok(PcurvePairIntervals {
        canonical,
        opposite,
    })
}

fn root_jet(
    m: Harmonic,
    l: Harmonic,
    a: Interval,
    h: Interval,
    cosine: Interval,
    sine: Interval,
    sheet: SkewCylinderSheet,
) -> Result<RootJetIntervals, IntersectionCertificateError> {
    interval_root_jet(
        IntervalHarmonic {
            constant: Interval::point(m.constant),
            cosine: Interval::point(m.cosine),
            sine: Interval::point(m.sine),
        },
        IntervalHarmonic {
            constant: Interval::point(l.constant),
            cosine: Interval::point(l.cosine),
            sine: Interval::point(l.sine),
        },
        a,
        h,
        cosine,
        sine,
        sheet,
    )
}

fn interval_root_jet(
    m: IntervalHarmonic,
    l: IntervalHarmonic,
    a: Interval,
    h: Interval,
    cosine: Interval,
    sine: Interval,
    sheet: SkewCylinderSheet,
) -> Result<RootJetIntervals, IntersectionCertificateError> {
    let m_value = m
        .interval(cosine, sine)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let l_value = l
        .interval(cosine, sine)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let m_derivative = harmonic_derivative(m, cosine, sine)?;
    let l_derivative = harmonic_derivative(l, cosine, sine)?;
    let h_derivative =
        finite_interval((-l_value * l_derivative).checked_div(h).ok_or_else(|| {
            unsupported("skew Cylinder/Cylinder pcurve cell square-root derivative is singular")
        })?)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let sign = Interval::point(sheet.sign());
    let value = finite_interval((-m_value + sign * h).checked_div(a).ok_or_else(|| {
        unsupported("skew Cylinder/Cylinder pcurve cell root divisor is singular")
    })?)
    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let derivative = finite_interval(
        (-m_derivative + sign * h_derivative)
            .checked_div(a)
            .ok_or_else(|| {
                unsupported("skew Cylinder/Cylinder pcurve cell derivative divisor is singular")
            })?,
    )
    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    Ok(RootJetIntervals { value, derivative })
}

fn harmonic_derivative(
    harmonic: IntervalHarmonic,
    cosine: Interval,
    sine: Interval,
) -> Result<Interval, IntersectionCertificateError> {
    finite_interval(-harmonic.cosine * sine + harmonic.sine * cosine)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)
}

fn dual_coordinates(
    harmonics: [Harmonic; 3],
    directions: [Interval; 3],
    root: RootJetIntervals,
    cosine: Interval,
    sine: Interval,
) -> Result<[[Interval; 2]; 3], IntersectionCertificateError> {
    interval_dual_coordinates(
        harmonics.map(|harmonic| IntervalHarmonic {
            constant: Interval::point(harmonic.constant),
            cosine: Interval::point(harmonic.cosine),
            sine: Interval::point(harmonic.sine),
        }),
        directions,
        root,
        cosine,
        sine,
    )
}

fn interval_dual_coordinates(
    harmonics: [IntervalHarmonic; 3],
    directions: [Interval; 3],
    root: RootJetIntervals,
    cosine: Interval,
    sine: Interval,
) -> Result<[[Interval; 2]; 3], IntersectionCertificateError> {
    let coordinates = core::array::from_fn(|axis| {
        let value = harmonics[axis]
            .interval(cosine, sine)
            .and_then(|value| finite_interval(value + directions[axis] * root.value));
        let derivative = harmonic_derivative(harmonics[axis], cosine, sine)
            .ok()
            .and_then(|value| finite_interval(value + directions[axis] * root.derivative));
        value.zip(derivative)
    });
    let [Some(x), Some(y), Some(z)] = coordinates else {
        return Err(IntersectionCertificateError::NonFiniteGeometry);
    };
    Ok([[x.0, x.1], [y.0, y.1], [z.0, z.1]])
}

fn opposite_pcurve(
    dual: [[Interval; 2]; 3],
    determinant: Interval,
    longitude_offset: f64,
    longitude_window: ParamRange,
) -> Result<([Interval; 2], [Interval; 2]), IntersectionCertificateError> {
    let normalized = [0, 1, 2].map(|axis| {
        dual[axis][0]
            .checked_div(determinant)
            .and_then(finite_interval)
    });
    let [Some(x), Some(y), Some(height)] = normalized else {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell normalization divisor is singular",
        ));
    };
    if !same_atan_chart(x, y) {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell leaves the retained opposite longitude chart",
        ));
    }
    let longitude = finite_interval(longitude_interval(x, y) + Interval::point(longitude_offset))
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    if longitude.lo() <= longitude_window.lo || longitude.hi() >= longitude_window.hi {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell escapes the retained opposite longitude lift",
        ));
    }
    let denominator = finite_interval(dual[0][0].square() + dual[1][0].square())
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let longitude_derivative = finite_interval(
        (dual[0][0] * dual[1][1] - dual[1][0] * dual[0][1])
            .checked_div(denominator)
            .ok_or_else(|| {
                unsupported(
                    "skew Cylinder/Cylinder opposite pcurve longitude derivative is singular",
                )
            })?,
    )
    .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    let height_derivative =
        finite_interval(dual[2][1].checked_div(determinant).ok_or_else(|| {
            unsupported("skew Cylinder/Cylinder opposite pcurve height divisor is singular")
        })?)
        .ok_or(IntersectionCertificateError::NonFiniteGeometry)?;
    Ok((
        [longitude, height],
        [longitude_derivative, height_derivative],
    ))
}

fn validate_common_opposite_chart(
    stored: [[Interval; 2]; 3],
    stored_determinant: Interval,
    source: [[Interval; 2]; 3],
    source_determinant: Interval,
) -> Result<(), IntersectionCertificateError> {
    let normalize = |coordinate: Interval, determinant: Interval| {
        coordinate
            .checked_div(determinant)
            .and_then(finite_interval)
    };
    let (Some(stored_x), Some(stored_y), Some(source_x), Some(source_y)) = (
        normalize(stored[0][0], stored_determinant),
        normalize(stored[1][0], stored_determinant),
        normalize(source[0][0], source_determinant),
        normalize(source[1][0], source_determinant),
    ) else {
        return Err(unsupported(
            "skew Cylinder/Cylinder pcurve cell normalization divisor is singular",
        ));
    };
    if (stored_y.lo() > 0.0 && source_y.lo() > 0.0)
        || (stored_y.hi() < 0.0 && source_y.hi() < 0.0)
        || (stored_x.lo() > 0.0 && source_x.lo() > 0.0)
    {
        Ok(())
    } else {
        Err(unsupported(
            "skew Cylinder/Cylinder source/evaluator pcurve cell has no common opposite longitude chart",
        ))
    }
}

fn same_atan_chart(x: Interval, y: Interval) -> bool {
    y.lo() > 0.0 || y.hi() < 0.0 || x.lo() > 0.0
}

fn derivative_box_is_regular(derivative: [Interval; 2]) -> bool {
    !derivative[0].contains_zero() || !derivative[1].contains_zero()
}

fn finite(interval: Interval) -> bool {
    interval.lo().is_finite() && interval.hi().is_finite()
}
