//! Shared validation for edge/pcurve/surface incidence.
//!
//! Both checked topology edits and the body checker use this implementation
//! so an operation cannot accept a pcurve that the checker later rejects.

use crate::entity::{CurveId, FinPcurve, SurfaceId};
use crate::store::Store;
use kgeom::param::ParamRange;

const INCIDENCE_SAMPLES: usize = 5;

/// Classification used by topology operations and checker diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PcurveIssue {
    StaleReference,
    BadRange,
    OffSurface,
}

fn parameter_slack(a: f64, b: f64) -> f64 {
    256.0 * f64::EPSILON * (1.0 + a.abs().max(b.abs()))
}

fn parameter_close(a: f64, b: f64) -> bool {
    (a - b).abs() <= parameter_slack(a, b)
}

/// Validate the pcurve handle and its active range against its own natural
/// parameter domain. Periodic curves may use an unwrapped interval no wider
/// than one period.
pub(crate) fn check_pcurve_definition(
    store: &Store,
    pcurve_use: FinPcurve,
) -> core::result::Result<(), PcurveIssue> {
    let geometry = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?;
    let curve = geometry.as_curve();
    let range = pcurve_use.range();
    let valid = match curve.periodicity() {
        Some(period) => {
            period.is_finite()
                && period > 0.0
                && range.width() <= period + parameter_slack(range.width(), period)
        }
        None => {
            let natural = curve.param_range();
            natural.contains(range.lo) && natural.contains(range.hi)
        }
    };
    if valid {
        Ok(())
    } else {
        Err(PcurveIssue::BadRange)
    }
}

/// Validate a complete `(3D curve, edge range, 2D pcurve, surface)` tuple.
pub(crate) fn check_pcurve_incidence(
    store: &Store,
    curve_id: CurveId,
    bounds: Option<(f64, f64)>,
    surface_id: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> core::result::Result<(), PcurveIssue> {
    check_pcurve_definition(store, pcurve_use)?;
    if !tolerance.is_finite() || tolerance < 0.0 {
        return Err(PcurveIssue::BadRange);
    }
    let curve_geometry = store
        .get(curve_id)
        .map_err(|_| PcurveIssue::StaleReference)?;
    let curve = curve_geometry.as_curve();
    let edge_range = match bounds {
        Some((lo, hi)) if lo.is_finite() && hi.is_finite() && lo < hi => ParamRange::new(lo, hi),
        None => {
            let range = curve.param_range();
            if !range.is_finite() || range.lo >= range.hi {
                return Err(PcurveIssue::BadRange);
            }
            range
        }
        Some(_) => return Err(PcurveIssue::BadRange),
    };

    let pcurve_range = pcurve_use.range();
    let q0 = pcurve_use.parameter_at_edge(edge_range.lo);
    let q1 = pcurve_use.parameter_at_edge(edge_range.hi);
    if !q0.is_finite()
        || !q1.is_finite()
        || !parameter_close(q0.min(q1), pcurve_range.lo)
        || !parameter_close(q0.max(q1), pcurve_range.hi)
    {
        return Err(PcurveIssue::BadRange);
    }

    let pcurve_geometry = store
        .get(pcurve_use.curve())
        .map_err(|_| PcurveIssue::StaleReference)?;
    let pcurve = pcurve_geometry.as_curve();
    let surface_geometry = store
        .get(surface_id)
        .map_err(|_| PcurveIssue::StaleReference)?;
    let surface = surface_geometry.as_surface();
    for i in 0..=INCIDENCE_SAMPLES {
        let t = edge_range.lerp(i as f64 / INCIDENCE_SAMPLES as f64);
        let q = pcurve_use.parameter_at_edge(t);
        if q < pcurve_range.lo - parameter_slack(q, pcurve_range.lo)
            || q > pcurve_range.hi + parameter_slack(q, pcurve_range.hi)
        {
            return Err(PcurveIssue::BadRange);
        }
        let uv = pcurve.eval(q);
        if !uv.x.is_finite() || !uv.y.is_finite() {
            return Err(PcurveIssue::BadRange);
        }
        if surface.eval([uv.x, uv.y]).dist(curve.eval(t)) > tolerance {
            return Err(PcurveIssue::OffSurface);
        }
    }
    Ok(())
}
