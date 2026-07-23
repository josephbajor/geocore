//! Deterministic tolerant-edge transport for persistent skew-cylinder spans.
//!
//! Parasolid X_T has no portable payload for the kernel's sealed proof object.
//! The conforming transport is therefore a curve-less tolerant edge with one
//! trimmed SP-curve on each source cylinder. Each SP-curve is a degree-1
//! B-curve over the certificate's exact interval-cell partition. The emitted
//! tolerance encloses both certified source residuals and the complete
//! piecewise-linear approximation error; no graph handle enters the payload.

use super::{Result, XtCapability, XtError};
use kcore::interval::Interval;
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve2d::{Curve2d, NurbsCurve2d};
use kgeom::param::ParamRange;
use kgeom::surface::Surface;
use kgeom::vec::Point2;
use kgraph::{
    PersistentSkewCylinderOpenSpanCertificate, PersistentSkewCylinderOpenSpanOrientation,
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderBranchPcurveEnclosure,
    VerifiedSkewCylinderOpenSpanCurveDescriptor,
};
use ktopo::entity::{Edge, FinId, FinPcurve};
use ktopo::geom::CurveGeom;
use ktopo::store::Store;
use ktopo::tolerance::EntityTolerance;

struct PcurveTransport {
    nurbs: NurbsCurve2d,
    lift_error: f64,
    operand: usize,
}

struct Segment {
    lo: f64,
    hi: f64,
    enclosure: SkewCylinderBranchPcurveEnclosure,
}

/// Whether one graph curve uses the persistent skew-cylinder proof class.
pub(super) fn is_persistent_curve(curve: &CurveGeom) -> bool {
    curve.as_persistent_skew_cylinder_open_span().is_some()
}

/// Validate that a persistent edge can be lowered to paired tolerant SP-curves.
pub(super) fn validate_edge(store: &Store, edge: &Edge) -> Result<()> {
    let descriptor = descriptor(store, edge)?.ok_or(XtError::Unsupported {
        capability: XtCapability::ProceduralCurves,
        what: "persistent skew-cylinder edge lost its graph descriptor",
    })?;
    let certificate = descriptor.certificate();
    if edge.vertices().iter().any(Option::is_none)
        || edge.bounds() != Some((0.0, 1.0))
        || edge.fins().len() != 2
        || edge.tolerance().is_none_or(|tolerance| {
            tolerance.value() < certificate.required_edge_tolerance().max(LINEAR_RESOLUTION)
        })
    {
        return Err(XtError::Unsupported {
            capability: XtCapability::WriterEdgeTopology,
            what: "persistent skew-cylinder X_T transport requires one bounded, tolerant, two-fin logical [0,1] edge",
        });
    }
    let mut seen = [false; 2];
    for &fin in edge.fins() {
        let transport = pcurve_transport(store, fin, descriptor)?;
        if seen[transport.operand] {
            return Err(XtError::InvalidModel {
                what: "persistent skew-cylinder edge repeats one source pcurve",
            });
        }
        seen[transport.operand] = true;
    }
    if !seen.into_iter().all(core::convert::identity) {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder edge does not retain both source pcurves",
        });
    }
    Ok(())
}

/// Replace the internal proof-bearing 3D curve with a null X_T edge curve.
pub(super) fn transmitted_curve(
    store: &Store,
    edge: &Edge,
) -> Result<Option<ktopo::entity::CurveId>> {
    if descriptor(store, edge)?.is_some() {
        Ok(None)
    } else {
        Ok(edge.curve())
    }
}

/// Baked-chart B-curve for one persistent fin, when this special transport applies.
pub(super) fn pcurve_nurbs(store: &Store, fin: FinId) -> Result<Option<NurbsCurve2d>> {
    let edge = store.get(store.get(fin)?.edge())?;
    let Some(descriptor) = descriptor(store, edge)? else {
        return Ok(None);
    };
    Ok(Some(pcurve_transport(store, fin, descriptor)?.nurbs))
}

/// Conservative X_T tolerance for the lowered curve-less edge.
pub(super) fn edge_tolerance(store: &Store, edge: &Edge) -> Result<Option<f64>> {
    let Some(descriptor) = descriptor(store, edge)? else {
        return Ok(edge.tolerance().map(EntityTolerance::value));
    };
    let certificate = descriptor.certificate();
    let mut approximation = [None; 2];
    for &fin in edge.fins() {
        let transport = pcurve_transport(store, fin, descriptor)?;
        approximation[transport.operand] = Some(transport.lift_error);
    }
    let [Some(first), Some(second)] = approximation else {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder edge does not retain both source pcurves",
        });
    };
    let residuals = certificate.residual_bounds();
    let paired = Interval::point(first)
        + Interval::point(second)
        + Interval::point(residuals[0])
        + Interval::point(residuals[1])
        + Interval::point(LINEAR_RESOLUTION);
    let tolerance = edge
        .tolerance()
        .map(EntityTolerance::value)
        .unwrap_or(LINEAR_RESOLUTION)
        .max(certificate.required_edge_tolerance())
        .max(paired.hi());
    if !tolerance.is_finite() {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder X_T tolerance is non-finite",
        });
    }
    Ok(Some(tolerance))
}

fn descriptor(
    store: &Store,
    edge: &Edge,
) -> Result<Option<VerifiedSkewCylinderOpenSpanCurveDescriptor>> {
    let Some(curve) = edge.curve() else {
        return Ok(None);
    };
    Ok(store
        .get(curve)?
        .as_persistent_skew_cylinder_open_span()
        .copied())
}

fn pcurve_transport(
    store: &Store,
    fin_id: FinId,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
) -> Result<PcurveTransport> {
    let fin = store.get(fin_id)?;
    let use_ = fin.pcurve().ok_or(XtError::InvalidModel {
        what: "persistent skew-cylinder fin has no pcurve",
    })?;
    validate_use(use_)?;
    let pcurve = store
        .get(use_.curve())?
        .as_persistent_skew_cylinder_open_span()
        .copied()
        .ok_or(XtError::InvalidModel {
            what: "persistent skew-cylinder fin lost its procedural pcurve",
        })?;
    let operand = pcurve.operand();
    let certificate = descriptor.certificate();
    if operand >= 2
        || descriptor.pcurves()[operand] != use_.curve()
        || certificate.pcurves()[operand] != pcurve
    {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder fin pcurve is not bound to its edge proof",
        });
    }
    let face = store.get(store.get(fin.parent())?.face())?;
    if descriptor.source_surfaces()[operand] != face.surface() {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder fin is attached to the wrong source surface",
        });
    }
    let cylinder =
        store
            .get(face.surface())?
            .as_cylinder()
            .copied()
            .ok_or(XtError::InvalidModel {
                what: "persistent skew-cylinder fin source is not a cylinder",
            })?;
    if cylinder != certificate.carrier().cylinders()[operand] {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder fin source changed after certification",
        });
    }

    let mut segments = certified_segments(certificate, operand)?;
    if certificate.orientation() == PersistentSkewCylinderOpenSpanOrientation::Reversed {
        segments.reverse();
    }
    let representatives = endpoint_representatives(certificate);
    let mut parameters = Vec::with_capacity(segments.len() + 1);
    parameters.push(0.0);
    for (index, segment) in segments.iter().enumerate() {
        let canonical =
            if certificate.orientation() == PersistentSkewCylinderOpenSpanOrientation::Forward {
                segment.hi
            } else {
                segment.lo
            };
        let logical = if index + 1 == segments.len() {
            1.0
        } else {
            logical_parameter(canonical, representatives, certificate.orientation())?
        };
        if logical <= *parameters.last().expect("logical start exists") || logical > 1.0 {
            return Err(XtError::InvalidModel {
                what: "persistent skew-cylinder proof partition is not strictly ordered",
            });
        }
        parameters.push(logical);
    }

    let periods = cylinder.periodicity();
    let points = parameters
        .iter()
        .map(|&parameter| {
            use_.chart()
                .apply(pcurve.eval(parameter), periods)
                .map_err(XtError::Kernel)
        })
        .collect::<Result<Vec<Point2>>>()?;
    let mut knots = Vec::with_capacity(points.len() + 2);
    knots.push(0.0);
    knots.extend(parameters.iter().copied());
    knots.push(1.0);
    let nurbs = NurbsCurve2d::new(1, knots, points, None).map_err(XtError::Kernel)?;
    let lift_error = segments.iter().try_fold(0.0_f64, |bound, segment| {
        segment_lift_error(segment, cylinder.radius()).map(|local| bound.max(local))
    })?;
    Ok(PcurveTransport {
        nurbs,
        lift_error,
        operand,
    })
}

fn validate_use(use_: FinPcurve) -> Result<()> {
    let map = use_.edge_to_pcurve();
    if use_.range() != ParamRange::new(0.0, 1.0)
        || map.scale().to_bits() != 1.0_f64.to_bits()
        || map.offset().to_bits() != 0.0_f64.to_bits()
        || use_.chart().period_shifts()[1] != 0
        || use_.closure_winding().is_some()
        || use_.seam().is_some()
    {
        return Err(XtError::Unsupported {
            capability: XtCapability::PeriodicPcurves,
            what: "persistent skew-cylinder X_T transport requires an open identity-mapped pcurve with only a cylinder-longitude chart shift",
        });
    }
    Ok(())
}

fn certified_segments(
    certificate: PersistentSkewCylinderOpenSpanCertificate,
    operand: usize,
) -> Result<Vec<Segment>> {
    let residual = certificate.residual_certificate();
    let guarded = residual.carrier_range();
    let corridors = certificate.root_corridors();
    let representatives = endpoint_representatives(certificate);
    let mut segments = Vec::with_capacity(SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS + 2);
    segments.push(Segment {
        lo: representatives[0],
        hi: guarded.lo,
        enclosure: corridors[0].corridor().pcurves()[operand],
    });
    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let cell = residual
            .certify_pcurve_cell(index)
            .map_err(|source| XtError::IntersectionCertificate { index: 0, source })?;
        let parameter = cell.parameter();
        segments.push(Segment {
            lo: parameter.lo(),
            hi: parameter.hi(),
            enclosure: cell.pcurves()[operand],
        });
    }
    segments.push(Segment {
        lo: guarded.hi,
        hi: representatives[1],
        enclosure: corridors[1].corridor().pcurves()[operand],
    });
    if segments.iter().any(|segment| {
        !segment.lo.is_finite() || !segment.hi.is_finite() || segment.lo > segment.hi
    }) {
        return Err(XtError::InvalidModel {
            what: "persistent skew-cylinder proof contains an invalid transport segment",
        });
    }
    // Boundary grading can collapse an extreme proof cell to one
    // representable parameter. It covers no open domain and contributes no
    // approximation error or B-curve knot.
    segments.retain(|segment| segment.lo < segment.hi);
    Ok(segments)
}

fn endpoint_representatives(certificate: PersistentSkewCylinderOpenSpanCertificate) -> [f64; 2] {
    certificate.root_corridors().map(|corridor| {
        let root = corridor.root_parameter();
        0.5 * root.lo() + 0.5 * root.hi()
    })
}

fn logical_parameter(
    canonical: f64,
    representatives: [f64; 2],
    orientation: PersistentSkewCylinderOpenSpanOrientation,
) -> Result<f64> {
    let [start, end] = match orientation {
        PersistentSkewCylinderOpenSpanOrientation::Forward => representatives,
        PersistentSkewCylinderOpenSpanOrientation::Reversed => {
            [representatives[1], representatives[0]]
        }
    };
    let logical = (canonical - start) / (end - start);
    if logical.is_finite() {
        Ok(logical)
    } else {
        Err(XtError::InvalidModel {
            what: "persistent skew-cylinder logical transport map is non-finite",
        })
    }
}

fn segment_lift_error(segment: &Segment, radius: f64) -> Result<f64> {
    let span = Interval::point(segment.hi) - Interval::point(segment.lo);
    let derivatives = segment.enclosure.stored_derivative();
    let error = derivatives.map(|derivative| {
        span * (Interval::point(derivative.hi()) - Interval::point(derivative.lo()))
    });
    let model = Interval::point(radius) * error[0] + error[1];
    if model.hi().is_finite() && model.hi() >= 0.0 {
        Ok(model.hi())
    } else {
        Err(XtError::InvalidModel {
            what: "persistent skew-cylinder SP-curve approximation bound is non-finite",
        })
    }
}
