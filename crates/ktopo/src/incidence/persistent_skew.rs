//! Exact whole-fin incidence for persistent skew-cylinder open spans.
//!
//! The graph descriptor already binds the normalized carrier to two ordered
//! source surfaces and pcurves. This adapter resolves one live fin back to that
//! binding by exact handles and consumes the descriptor's outward residual
//! bound directly. It deliberately does not sample or add a generic floating
//! point guard to the certifier-owned bound.

use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::vec::Point3;

use crate::entity::{Edge, FinPcurve, ParamMap1d, PcurveEndpointKind, SurfaceId};
use crate::geom::SurfaceGeom;
use crate::store::Store;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PersistentSkewIncidence {
    NotApplicable,
    Certified,
    Indeterminate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PersistentSkewPcurvePrecheck {
    NotApplicable,
    Admissible,
    Indeterminate,
}

/// Reject malformed persistent metadata before generic evaluators see it.
pub(crate) fn precheck_pcurve(
    store: &Store,
    edge: &Edge,
    surface: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> PersistentSkewPcurvePrecheck {
    match pcurve_context(store, edge, surface, pcurve_use, tolerance) {
        Ok(_) => PersistentSkewPcurvePrecheck::Admissible,
        Err(PersistentSkewIncidence::NotApplicable) => PersistentSkewPcurvePrecheck::NotApplicable,
        Err(PersistentSkewIncidence::Certified | PersistentSkewIncidence::Indeterminate) => {
            PersistentSkewPcurvePrecheck::Indeterminate
        }
    }
}

/// Certify one already ownership-validated fin against its persistent graph
/// descriptor.
pub(crate) fn certify_pcurve(
    store: &Store,
    edge: &Edge,
    surface: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> PersistentSkewIncidence {
    let (descriptor, source_slot) =
        match pcurve_context(store, edge, surface, pcurve_use, tolerance) {
            Ok(value) => value,
            Err(result) => return result,
        };
    residual_result(
        descriptor.certificate().residual_bounds()[source_slot],
        tolerance,
    )
}

fn pcurve_context(
    store: &Store,
    edge: &Edge,
    surface: SurfaceId,
    pcurve_use: FinPcurve,
    tolerance: f64,
) -> Result<(kgraph::VerifiedSkewCylinderOpenSpanCurveDescriptor, usize), PersistentSkewIncidence> {
    let (descriptor, source_slot) = match certify_edge_context(store, edge, surface, tolerance) {
        Ok(value) => value,
        Err(result) => return Err(result),
    };
    let certificate = descriptor.certificate();

    if descriptor.pcurves()[source_slot] != pcurve_use.curve() {
        return Err(PersistentSkewIncidence::Indeterminate);
    }
    let Some(live_pcurve) = store
        .pcurve(pcurve_use.curve())
        .ok()
        .and_then(|curve| curve.as_persistent_skew_cylinder_open_span())
    else {
        return Err(PersistentSkewIncidence::Indeterminate);
    };
    if *live_pcurve != certificate.pcurves()[source_slot]
        || !is_logical_range(Some((pcurve_use.range().lo, pcurve_use.range().hi)))
        || !is_identity(pcurve_use.edge_to_pcurve())
        || pcurve_use.endpoint_kinds() != [PcurveEndpointKind::Regular; 2]
        || pcurve_use.chart().period_shifts()[1] != 0
        || pcurve_use.closure_winding().is_some()
        || pcurve_use.seam().is_some()
    {
        return Err(PersistentSkewIncidence::Indeterminate);
    }

    Ok((descriptor, source_slot))
}

/// Certify only the persistent spatial edge against one exact live source.
pub(crate) fn certify_edge(
    store: &Store,
    edge: &Edge,
    surface: SurfaceId,
    tolerance: f64,
) -> PersistentSkewIncidence {
    let (descriptor, source_slot) = match certify_edge_context(store, edge, surface, tolerance) {
        Ok(value) => value,
        Err(result) => return result,
    };
    residual_result(
        descriptor.certificate().residual_bounds()[source_slot],
        tolerance,
    )
}

fn certify_edge_context(
    store: &Store,
    edge: &Edge,
    surface: SurfaceId,
    tolerance: f64,
) -> Result<(kgraph::VerifiedSkewCylinderOpenSpanCurveDescriptor, usize), PersistentSkewIncidence> {
    let Some(curve) = edge.curve() else {
        return Err(PersistentSkewIncidence::NotApplicable);
    };
    let Ok(curve) = store.curve(curve) else {
        return Err(PersistentSkewIncidence::Indeterminate);
    };
    let Some(descriptor) = curve.as_persistent_skew_cylinder_open_span().copied() else {
        return Err(PersistentSkewIncidence::NotApplicable);
    };
    let certificate = descriptor.certificate();

    if !tolerance.is_finite()
        || tolerance < 0.0
        || !is_logical_range(edge.bounds())
        || !valid_logical_vertices(store, edge, certificate.endpoint_points())
    {
        return Err(PersistentSkewIncidence::Indeterminate);
    }

    let required_tolerance = certificate.required_edge_tolerance().max(LINEAR_RESOLUTION);
    if !required_tolerance.is_finite()
        || edge
            .tolerance()
            .is_none_or(|stored| stored.value() < required_tolerance)
    {
        return Err(PersistentSkewIncidence::Indeterminate);
    }

    let source_surfaces = descriptor.source_surfaces();
    let source_slot = match (source_surfaces[0] == surface, source_surfaces[1] == surface) {
        (true, false) => 0,
        (false, true) => 1,
        _ => return Err(PersistentSkewIncidence::Indeterminate),
    };
    let Some(SurfaceGeom::Cylinder(live_source)) = store.surface(surface).ok() else {
        return Err(PersistentSkewIncidence::Indeterminate);
    };
    if *live_source != certificate.carrier().cylinders()[source_slot] {
        return Err(PersistentSkewIncidence::Indeterminate);
    }
    Ok((descriptor, source_slot))
}

fn residual_result(residual_bound: f64, tolerance: f64) -> PersistentSkewIncidence {
    if residual_bound.is_finite() && residual_bound >= 0.0 && residual_bound <= tolerance {
        PersistentSkewIncidence::Certified
    } else {
        PersistentSkewIncidence::Indeterminate
    }
}

fn is_logical_range(bounds: Option<(f64, f64)>) -> bool {
    bounds.is_some_and(|(lo, hi)| {
        lo.to_bits() == 0.0_f64.to_bits() && hi.to_bits() == 1.0_f64.to_bits()
    })
}

fn is_identity(map: ParamMap1d) -> bool {
    map.scale().to_bits() == 1.0_f64.to_bits() && map.offset().to_bits() == 0.0_f64.to_bits()
}

fn valid_logical_vertices(store: &Store, edge: &Edge, expected: [Point3; 2]) -> bool {
    let [Some(first), Some(second)] = edge.vertices() else {
        return false;
    };
    if first == second {
        return false;
    }
    let Ok(first_point) = store.vertex_position(first) else {
        return false;
    };
    let Ok(second_point) = store.vertex_position(second) else {
        return false;
    };
    same_point_bits(first_point, expected[0]) && same_point_bits(second_point, expected[1])
}

fn same_point_bits(left: Point3, right: Point3) -> bool {
    left.x.to_bits() == right.x.to_bits()
        && left.y.to_bits() == right.y.to_bits()
        && left.z.to_bits() == right.z.to_bits()
}
