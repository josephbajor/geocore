//! Shared periodic-chart selection and pcurve normalization.
//!
//! Boolean realizers use this module only after exact topology has selected
//! bounded carrier ranges.  It chooses one complete-period cylinder chart for
//! all of those ranges, shifts each pcurve into that chart without changing
//! its carrier parameterization, and validates endpoint-free whole rings
//! before assigning them the same logical range.

use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point2;
use ktopo::analytic_shell::{AnalyticPcurveUse, AnalyticShellPcurve, AnalyticShellSurface};
use ktopo::entity::PcurveChart;

/// Typed refusal from Boolean-local periodic chart preparation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum PeriodicChartError {
    /// A period, range, pcurve, or chart value was not finite and nondegenerate.
    InvalidAnalyticGeometry,
    /// An endpoint-free use was not one exact horizontal complete-period winding.
    InvalidEndpointFreePeriodicUse,
    /// No single complete-period chart contains every bounded use.
    NoCommonPeriodicWindow,
    /// The required integral chart shift is not representable.
    PeriodShiftOverflow,
}

fn surface_periodicity(surface: AnalyticShellSurface) -> [Option<f64>; 2] {
    match surface {
        AnalyticShellSurface::Plane(surface) => surface.periodicity(),
        AnalyticShellSurface::Cylinder(surface) => surface.periodicity(),
    }
}

/// Bound one pcurve over its active edge-carrier range in its authored chart.
pub(super) fn pcurve_bounds(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<(Point2, Point2), PeriodicChartError> {
    let map = pcurve.edge_to_pcurve();
    let endpoints = [map.map(edge_range.lo), map.map(edge_range.hi)];
    let active = ParamRange::new(
        endpoints[0].min(endpoints[1]),
        endpoints[0].max(endpoints[1]),
    );
    if !active.is_finite() || active.lo >= active.hi {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    }
    let bounds = match pcurve.curve() {
        AnalyticShellPcurve::Line(curve) => curve.bounding_box(active),
        AnalyticShellPcurve::Circle(curve) => curve.bounding_box(active),
    };
    let periods = surface_periodicity(surface);
    let min = pcurve
        .chart()
        .apply(bounds.min, periods)
        .map_err(|_| PeriodicChartError::InvalidAnalyticGeometry)?;
    let max = pcurve
        .chart()
        .apply(bounds.max, periods)
        .map_err(|_| PeriodicChartError::InvalidAnalyticGeometry)?;
    Ok((min, max))
}

/// Return the integral period shift that places one closed interval in a window.
pub(super) fn periodic_interval_shift(
    period: f64,
    window: ParamRange,
    interval: (f64, f64),
) -> Result<i32, PeriodicChartError> {
    let (min, max) = interval;
    if !period.is_finite()
        || period <= 0.0
        || !window.is_finite()
        || window.lo >= window.hi
        || window.width() != period
        || !min.is_finite()
        || !max.is_finite()
        || min > max
    {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    }
    let epsilon = 256.0
        * f64::EPSILON
        * window
            .lo
            .abs()
            .max(window.hi.abs())
            .max(min.abs())
            .max(max.abs())
            .max(period)
            .max(1.0);
    if max - min > period + epsilon {
        return Err(PeriodicChartError::NoCommonPeriodicWindow);
    }
    let delta_value = ((window.lo - min - epsilon) / period).ceil();
    if !delta_value.is_finite()
        || delta_value < f64::from(i32::MIN)
        || delta_value > f64::from(i32::MAX)
    {
        return Err(PeriodicChartError::PeriodShiftOverflow);
    }
    let delta = delta_value as i32;
    let shifted_min = min + f64::from(delta) * period;
    let shifted_max = max + f64::from(delta) * period;
    if !shifted_min.is_finite()
        || !shifted_max.is_finite()
        || shifted_min < window.lo - epsilon
        || shifted_max > window.hi + epsilon
    {
        return Err(PeriodicChartError::NoCommonPeriodicWindow);
    }
    Ok(delta)
}

fn exact_periodic_interval_shift(
    period: f64,
    window: ParamRange,
    interval: (f64, f64),
) -> Result<i32, PeriodicChartError> {
    let preferred = periodic_interval_shift(period, window, interval)?;
    let (min, max) = interval;
    for delta in [
        Some(preferred),
        preferred.checked_sub(1),
        preferred.checked_add(1),
    ]
    .into_iter()
    .flatten()
    {
        let shifted_min = min + f64::from(delta) * period;
        let shifted_max = max + f64::from(delta) * period;
        if shifted_min.is_finite()
            && shifted_max.is_finite()
            && window.contains(shifted_min)
            && window.contains(shifted_max)
        {
            return Ok(delta);
        }
    }
    Err(PeriodicChartError::NoCommonPeriodicWindow)
}

fn exact_period_window(lo: f64, period: f64) -> Option<ParamRange> {
    let hi = lo + period;
    (lo.is_finite() && hi.is_finite() && hi > lo && hi - lo == period)
        .then_some(ParamRange::new(lo, hi))
}

fn centered_exact_period_window(seam: f64, period: f64) -> Option<ParamRange> {
    let turns = (-seam / period).round();
    if !turns.is_finite() {
        return None;
    }
    exact_period_window(seam + turns * period, period)
}

fn canonical_periodic_seam(value: f64, origin: f64, period: f64) -> Option<f64> {
    let turns = ((value - origin) / period).floor();
    if !turns.is_finite() {
        return None;
    }
    let mut seam = value - turns * period;
    let upper = origin + period;
    if seam < origin {
        seam += period;
    } else if seam >= upper {
        seam -= period;
    }
    seam.is_finite().then_some(seam)
}

fn intervals_partition_window(period: f64, window: ParamRange, intervals: &[(f64, f64)]) -> bool {
    if intervals.is_empty() {
        return false;
    }
    let mut shifted = intervals
        .iter()
        .map(|&interval| {
            let shift = exact_periodic_interval_shift(period, window, interval).ok()?;
            let delta = f64::from(shift) * period;
            Some((interval.0 + delta, interval.1 + delta))
        })
        .collect::<Option<Vec<_>>>();
    let Some(shifted) = shifted.as_mut() else {
        return false;
    };
    shifted.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.total_cmp(&right.1)));
    shifted[0].0.to_bits() == window.lo.to_bits()
        && shifted[shifted.len() - 1].1.to_bits() == window.hi.to_bits()
        && shifted
            .windows(2)
            .all(|pair| pair[0].1.to_bits() == pair[1].0.to_bits())
}

/// Select a chart for intervals that must form one exact complete-period
/// partition. Unlike the general window selector, this preserves bit-exact
/// adjacent joins and the closing `root + period` identity.
pub(super) fn select_complete_periodic_partition_window(
    period: f64,
    authored: ParamRange,
    intervals: &[(f64, f64)],
) -> Result<ParamRange, PeriodicChartError> {
    if !period.is_finite()
        || period <= 0.0
        || !authored.is_finite()
        || authored.lo >= authored.hi
        || authored.width() != period
        || intervals.is_empty()
    {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    }
    if intervals_partition_window(period, authored, intervals) {
        return Ok(authored);
    }
    let mut seams = intervals
        .iter()
        .flat_map(|&(lo, hi)| [lo, hi])
        .collect::<Vec<_>>();
    seams.sort_by(f64::total_cmp);
    seams.dedup_by(|left, right| left.to_bits() == right.to_bits());
    for &candidate in &seams {
        let Some(window) = centered_exact_period_window(candidate, period) else {
            continue;
        };
        if intervals_partition_window(period, window, intervals) {
            return Ok(window);
        }
    }
    for &candidate in &seams {
        let Some(window) = exact_period_window(candidate, period) else {
            continue;
        };
        if intervals_partition_window(period, window, intervals) {
            return Ok(window);
        }
    }
    Err(PeriodicChartError::NoCommonPeriodicWindow)
}

/// Choose a deterministic common window from intrinsic pcurve coordinates.
/// Existing integer chart shifts are representation choices and therefore do
/// not influence the selected result chart.
pub(super) fn select_common_periodic_window_for_uses(
    surface: AnalyticShellSurface,
    authored: ParamRange,
    uses: &[(AnalyticPcurveUse, ParamRange)],
) -> Result<ParamRange, PeriodicChartError> {
    let AnalyticShellSurface::Cylinder(cylinder) = surface else {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    };
    let Some(period) = cylinder.periodicity()[0] else {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    };
    if uses.is_empty() {
        return select_common_periodic_window(period, authored, &[]);
    }
    let intervals = uses
        .iter()
        .map(|&(pcurve, range)| {
            let [_, v] = pcurve.chart().period_shifts();
            let intrinsic = pcurve.with_chart(PcurveChart::shifted([0, v]));
            pcurve_bounds(surface, intrinsic, range).map(|(min, max)| (min.x, max.x))
        })
        .collect::<Result<Vec<_>, _>>()?;
    match select_complete_periodic_partition_window(period, authored, &intervals) {
        Ok(window) => Ok(window),
        Err(PeriodicChartError::NoCommonPeriodicWindow) => {
            select_common_periodic_window(period, authored, &intervals)
        }
        Err(error) => Err(error),
    }
}

/// Choose one exact complete-period window containing all bounded intervals.
///
/// Each interval may be shifted independently by an integral number of
/// periods. The authored window wins when possible; otherwise selection is
/// deterministic and prefers a seam in an open complement gap.
pub(super) fn select_common_periodic_window(
    period: f64,
    authored: ParamRange,
    intervals: &[(f64, f64)],
) -> Result<ParamRange, PeriodicChartError> {
    if !period.is_finite()
        || period <= 0.0
        || !authored.is_finite()
        || authored.lo >= authored.hi
        || authored.width() != period
    {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    }
    if intervals
        .iter()
        .all(|interval| periodic_interval_shift(period, authored, *interval).is_ok())
    {
        return Ok(authored);
    }

    let mut raw_seams = intervals
        .iter()
        .flat_map(|&(lo, hi)| [lo, hi])
        .collect::<Vec<_>>();
    raw_seams.sort_by(f64::total_cmp);
    raw_seams.dedup_by(|left, right| left.to_bits() == right.to_bits());
    let mut seams = raw_seams
        .iter()
        .map(|&value| {
            canonical_periodic_seam(value, authored.lo, period)
                .ok_or(PeriodicChartError::InvalidAnalyticGeometry)
        })
        .collect::<Result<Vec<_>, _>>()?;
    seams.sort_by(f64::total_cmp);
    seams.dedup_by(|left, right| left.to_bits() == right.to_bits());
    let mut interior_candidates = Vec::new();
    if seams.len() == 1 {
        interior_candidates.push(seams[0] + period / 2.0);
    } else if seams.len() > 1 {
        interior_candidates.extend(
            seams
                .windows(2)
                .map(|pair| pair[0] + (pair[1] - pair[0]) / 2.0),
        );
        let wrap_hi = seams[0] + period;
        interior_candidates.push(seams[seams.len() - 1] + (wrap_hi - seams[seams.len() - 1]) / 2.0);
    }
    interior_candidates.sort_by(f64::total_cmp);
    interior_candidates.dedup_by(|left, right| left.to_bits() == right.to_bits());
    // Prefer a seam in a certified open complement gap. Endpoint seams remain
    // a deterministic fallback because closed interval containment proves no
    // bounded pcurve crosses that face-chart boundary.
    for &candidate in &interior_candidates {
        let Some(window) = centered_exact_period_window(candidate, period) else {
            continue;
        };
        if intervals
            .iter()
            .all(|interval| periodic_interval_shift(period, window, *interval).is_ok())
        {
            return Ok(window);
        }
    }
    for &candidate in &seams {
        let Some(window) = centered_exact_period_window(candidate, period) else {
            continue;
        };
        if intervals
            .iter()
            .all(|interval| periodic_interval_shift(period, window, *interval).is_ok())
        {
            return Ok(window);
        }
    }
    // Some endpoint-rooted windows cannot survive a modulo reduction without
    // reversing one floating-point rounding step. The raw authored lift is a
    // deterministic last resort, accepted only under exact containment.
    for &candidate in &raw_seams {
        let Some(window) = exact_period_window(candidate, period) else {
            continue;
        };
        if intervals
            .iter()
            .all(|interval| exact_periodic_interval_shift(period, window, *interval).is_ok())
        {
            return Ok(window);
        }
    }
    Err(PeriodicChartError::NoCommonPeriodicWindow)
}

/// Shift one bounded pcurve into a selected complete-period cylinder window.
pub(super) fn normalize_periodic_pcurve_chart(
    surface: AnalyticShellSurface,
    window: ParamRange,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<AnalyticPcurveUse, PeriodicChartError> {
    let AnalyticShellSurface::Cylinder(_) = surface else {
        return Ok(pcurve);
    };
    let Some(period) = surface_periodicity(surface)[0] else {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    };
    let (min, max) = pcurve_bounds(surface, pcurve, edge_range)?;
    let delta = periodic_interval_shift(period, window, (min.x, max.x))?;
    let [u, v] = pcurve.chart().period_shifts();
    let u = u
        .checked_add(delta)
        .ok_or(PeriodicChartError::PeriodShiftOverflow)?;
    Ok(pcurve.with_chart(PcurveChart::shifted([u, v])))
}

/// Normalize from intrinsic coordinates when bit-exact partition joins matter.
///
/// Existing chart shifts are representation choices. Stripping them before
/// selecting the final integral lift avoids an extra floating-point period
/// addition and preserves exact complementary-arc endpoints.
pub(super) fn normalize_intrinsic_periodic_pcurve_chart(
    surface: AnalyticShellSurface,
    window: ParamRange,
    pcurve: AnalyticPcurveUse,
    edge_range: ParamRange,
) -> Result<AnalyticPcurveUse, PeriodicChartError> {
    let AnalyticShellSurface::Cylinder(_) = surface else {
        return Ok(pcurve);
    };
    let Some(period) = surface_periodicity(surface)[0] else {
        return Err(PeriodicChartError::InvalidAnalyticGeometry);
    };
    let [_, v] = pcurve.chart().period_shifts();
    let unshifted = pcurve.with_chart(PcurveChart::shifted([0, v]));
    let (min, max) = pcurve_bounds(surface, unshifted, edge_range)?;
    let u = exact_periodic_interval_shift(period, window, (min.x, max.x))?;
    Ok(pcurve.with_chart(PcurveChart::shifted([u, v])))
}

fn validate_endpoint_free_periodic_ring_shape(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    window: ParamRange,
) -> Result<f64, PeriodicChartError> {
    let AnalyticShellSurface::Cylinder(cylinder) = surface else {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    };
    let Some(period) = cylinder.periodicity()[0] else {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    };
    let AnalyticShellPcurve::Line(line) = pcurve.curve() else {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    };
    let Some([winding @ (-1 | 1), 0]) = pcurve.closure_winding() else {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    };
    let map = pcurve.edge_to_pcurve();
    let delta = line.dir() * (map.scale() * window.width());
    if !window.is_finite()
        || window.lo >= window.hi
        || window.width() != period
        || !map.scale().is_finite()
        || !map.offset().is_finite()
        || line.dir().y != 0.0
        || delta.y != 0.0
        || delta.x != f64::from(winding) * period
    {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    }
    Ok(period)
}

/// Validate and chart-shift one endpoint-free cylinder ring onto `window`.
///
/// The caller may safely use `window` as the ring carrier's logical range only
/// after this succeeds. The returned pcurve is normalized into that same
/// window and retains its exact closure winding.
pub(super) fn shift_endpoint_free_periodic_ring(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    window: ParamRange,
) -> Result<AnalyticPcurveUse, PeriodicChartError> {
    let period = validate_endpoint_free_periodic_ring_shape(surface, pcurve, window)?;
    let normalized = normalize_periodic_pcurve_chart(surface, window, pcurve, window)?;
    let (min, max) = pcurve_bounds(surface, normalized, window)?;
    if periodic_interval_shift(period, window, (min.x, max.x))? != 0 {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    }
    Ok(normalized)
}

/// Validate and intrinsically chart-shift one endpoint-free ring.
pub(super) fn shift_endpoint_free_intrinsic_periodic_ring(
    surface: AnalyticShellSurface,
    pcurve: AnalyticPcurveUse,
    window: ParamRange,
) -> Result<AnalyticPcurveUse, PeriodicChartError> {
    let period = validate_endpoint_free_periodic_ring_shape(surface, pcurve, window)?;
    let normalized = normalize_intrinsic_periodic_pcurve_chart(surface, window, pcurve, window)?;
    let (min, max) = pcurve_bounds(surface, normalized, window)?;
    if exact_periodic_interval_shift(period, window, (min.x, max.x))? != 0 {
        return Err(PeriodicChartError::InvalidEndpointFreePeriodicUse);
    }
    Ok(normalized)
}

#[cfg(test)]
mod tests {
    use core::f64::consts::{PI, TAU};

    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;
    use kgeom::vec::{Point2, Vec2};
    use kgraph::AffineParamMap1d;

    use super::*;

    fn cylinder() -> AnalyticShellSurface {
        AnalyticShellSurface::Cylinder(Cylinder::new(Frame::world(), 1.0).unwrap())
    }

    fn horizontal_use() -> AnalyticPcurveUse {
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            AffineParamMap1d::new(1.0, 0.0).unwrap(),
        )
    }

    #[test]
    fn complementary_arcs_share_one_root_seamed_window() {
        let root = PI / 3.0;
        let split = 5.0 * PI / 3.0;
        let lifted_root = root + TAU;
        let authored = ParamRange::new(0.0, TAU);
        let uses = [
            (horizontal_use(), ParamRange::new(root, split)),
            (
                horizontal_use().with_chart(PcurveChart::shifted([-1, 0])),
                ParamRange::new(split, lifted_root),
            ),
        ];
        let intervals = uses
            .iter()
            .map(|(pcurve, range)| {
                pcurve_bounds(cylinder(), *pcurve, *range).map(|(min, max)| (min.x, max.x))
            })
            .collect::<Result<Vec<_>, _>>()
            .unwrap();
        let window = select_common_periodic_window(TAU, authored, &intervals).unwrap();
        assert_eq!(window.width(), TAU);

        let normalized = uses.map(|(pcurve, range)| {
            let pcurve =
                normalize_periodic_pcurve_chart(cylinder(), window, pcurve, range).unwrap();
            pcurve_bounds(cylinder(), pcurve, range).unwrap()
        });
        let min = normalized[0].0.x.min(normalized[1].0.x);
        let max = normalized[0].1.x.max(normalized[1].1.x);
        let epsilon = 256.0 * f64::EPSILON * window.hi.abs().max(1.0);
        assert!((min - window.lo).abs() <= epsilon);
        assert!((max - window.hi).abs() <= epsilon);
    }

    #[test]
    fn seam_crossing_arc_rotates_and_normalizes_without_wrap() {
        let authored = ParamRange::new(0.0, TAU);
        let range = ParamRange::new(5.0 * PI / 3.0, 7.0 * PI / 3.0);
        let bounds = pcurve_bounds(cylinder(), horizontal_use(), range).unwrap();
        let window =
            select_common_periodic_window(TAU, authored, &[(bounds.0.x, bounds.1.x)]).unwrap();
        assert_ne!(window, authored);
        let normalized =
            normalize_periodic_pcurve_chart(cylinder(), window, horizontal_use(), range).unwrap();
        let (min, max) = pcurve_bounds(cylinder(), normalized, range).unwrap();
        assert!(min.x > window.lo);
        assert!(max.x < window.hi);
    }

    #[test]
    fn complementary_lift_rounding_anchors_an_exact_containing_window() {
        for (root, split) in [
            (3.5531094996568497, 5.871668461112051),
            (3.2795296500082523, 6.1452483107611915),
        ] {
            let lifted_root = root + TAU;
            assert_ne!((lifted_root - TAU).to_bits(), root.to_bits());
            let intervals = [(root, split), (split, lifted_root)];
            let window = select_complete_periodic_partition_window(
                TAU,
                ParamRange::new(0.0, TAU),
                &intervals,
            )
            .unwrap();
            assert_eq!(window.lo.to_bits(), root.to_bits());
            assert_eq!(window.hi.to_bits(), lifted_root.to_bits());
            assert!(intervals_partition_window(TAU, window, &intervals));
        }
    }

    #[test]
    fn endpoint_free_ring_is_validated_and_shifted_into_the_selected_window() {
        let window = ParamRange::new(PI / 3.0, 7.0 * PI / 3.0);
        let ring = horizontal_use()
            .with_chart(PcurveChart::shifted([-2, 0]))
            .with_closure_winding([1, 0]);
        let shifted = shift_endpoint_free_periodic_ring(cylinder(), ring, window).unwrap();
        assert_eq!(shifted.chart().period_shifts(), [0, 0]);
        let (min, max) = pcurve_bounds(cylinder(), shifted, window).unwrap();
        assert_eq!(min.x.to_bits(), window.lo.to_bits());
        assert_eq!(max.x.to_bits(), window.hi.to_bits());
    }

    #[test]
    fn invalid_period_window_interval_and_ring_fail_closed() {
        assert_eq!(
            select_common_periodic_window(TAU, ParamRange::new(0.0, PI), &[]),
            Err(PeriodicChartError::InvalidAnalyticGeometry)
        );
        assert_eq!(
            periodic_interval_shift(TAU, ParamRange::new(0.0, TAU), (0.0, TAU + 0.25),),
            Err(PeriodicChartError::NoCommonPeriodicWindow)
        );
        let invalid_ring = horizontal_use().with_closure_winding([0, 0]);
        assert_eq!(
            shift_endpoint_free_periodic_ring(cylinder(), invalid_ring, ParamRange::new(0.0, TAU),),
            Err(PeriodicChartError::InvalidEndpointFreePeriodicUse)
        );
    }
}
