//! Fixed-cell flux integration for theorem-certified bounded-skew lobes.
//!
//! The nonlinear pcurve is never sampled. Each term consumes exact-source UV
//! and first-derivative enclosures reissued by the persistent certificate's
//! 256 guarded cells and its two physical-root corridors.

use super::{
    BodyPropertiesRefusal, Flux, SpanIntegral, finite_interval, integrate_cylinder_span,
    interval_dot, interval_ratio, point_vec, relative_coordinates,
};
use crate::entity::{FaceId, LoopId, ParamMap1d, Sense};
use crate::geom::Curve2dGeom;
use crate::loop_proof::bounded_pcurve_integral::BoundedPcurveSpan;
use crate::shell_proof::bounded_skew_lobe_shell_proof::BoundedSkewLobePropertyWitness;
use crate::store::Store;
use kcore::interval::Interval;
use kcore::math;
use kgeom::surface::Cylinder;
use kgeom::vec::{Point2, Point3};
use kgraph::{
    PersistentSkewCylinderOpenSpanCertificate, PersistentSkewCylinderOpenSpanOrientation,
    SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS, SkewCylinderBranchPcurveEnclosure,
    VerifiedSkewCylinderOpenSpanCurveDescriptor,
};

pub(super) fn integrate_lobe_cylinder_loop(
    store: &Store,
    face_id: FaceId,
    loop_id: LoopId,
    cylinder: Cylinder,
    anchor: Point3,
    witness: BoundedSkewLobePropertyWitness,
    source_slot: usize,
) -> core::result::Result<SpanIntegral, BodyPropertiesRefusal> {
    let loop_ = store
        .get(loop_id)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    if loop_.face != face_id || loop_.fins.len() != 4 || source_slot >= 2 {
        return Err(gap(face_id));
    }
    let mut total = SpanIntegral::zero();
    let mut persistent_count = 0_usize;
    for &fin_id in &loop_.fins {
        let fin = store
            .get(fin_id)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
        let edge = store
            .get(fin.edge)
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
        let use_ = fin
            .pcurve
            .ok_or(BodyPropertiesRefusal::UnsupportedPcurve { face: face_id })?;
        let pcurve = store
            .get(use_.curve())
            .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
        let next = match pcurve {
            Curve2dGeom::PersistentSkewCylinderOpenSpan(pcurve) => {
                persistent_count += 1;
                let descriptor = witness
                    .persistent_descriptor(fin.edge)
                    .ok_or_else(|| gap(face_id))?;
                validate_persistent_use(
                    store,
                    face_id,
                    source_slot,
                    descriptor,
                    pcurve.as_ref(),
                    edge,
                    use_,
                )?;
                let chart_offset = use_
                    .chart()
                    .apply(Point2::default(), [Some(core::f64::consts::TAU), None])
                    .map_err(|_| gap(face_id))?;
                integrate_persistent_span(
                    cylinder,
                    anchor,
                    descriptor.certificate(),
                    source_slot,
                    chart_offset,
                    traversal_sign(fin.sense, descriptor.certificate().orientation()),
                )
                .ok_or_else(|| gap(face_id))?
            }
            Curve2dGeom::Line(_) => {
                let (lo, hi) = edge.bounds.ok_or_else(|| gap(face_id))?;
                if use_.closure_winding().is_some() || use_.seam().is_some() {
                    return Err(gap(face_id));
                }
                let (edge_start, edge_end) = if fin.sense == Sense::Forward {
                    (lo, hi)
                } else {
                    (hi, lo)
                };
                let start = use_.parameter_at_edge(edge_start);
                let end = use_.parameter_at_edge(edge_end);
                let chart_offset = use_
                    .chart()
                    .apply(Point2::default(), [Some(core::f64::consts::TAU), None])
                    .map_err(|_| gap(face_id))?;
                integrate_cylinder_span(
                    cylinder,
                    anchor,
                    BoundedPcurveSpan::new(pcurve, start, end, chart_offset),
                )
                .ok_or_else(|| gap(face_id))?
            }
            _ => return Err(BodyPropertiesRefusal::UnsupportedPcurve { face: face_id }),
        };
        total = total.add(next);
        if !total.finite() {
            return Err(gap(face_id));
        }
    }
    if persistent_count != 2 {
        return Err(gap(face_id));
    }
    Ok(total)
}

fn validate_persistent_use(
    store: &Store,
    face_id: FaceId,
    source_slot: usize,
    descriptor: VerifiedSkewCylinderOpenSpanCurveDescriptor,
    pcurve: &kgraph::PersistentSkewCylinderOpenSpanPcurve,
    edge: &crate::entity::Edge,
    use_: crate::entity::FinPcurve,
) -> core::result::Result<(), BodyPropertiesRefusal> {
    let face = store
        .get(face_id)
        .map_err(|_| BodyPropertiesRefusal::BodyNotFullValid)?;
    let spatial = edge
        .curve
        .and_then(|curve| store.get(curve).ok())
        .and_then(|curve| curve.as_persistent_skew_cylinder_open_span())
        .copied();
    if spatial != Some(descriptor)
        || edge.bounds != Some((0.0, 1.0))
        || descriptor.source_surfaces()[source_slot] != face.surface
        || descriptor.pcurves()[source_slot] != use_.curve()
        || descriptor.certificate().pcurves()[source_slot] != *pcurve
        || descriptor
            .certificate()
            .finite_window_family_membership()
            .is_none()
        || use_.edge_to_pcurve() != ParamMap1d::identity()
        || use_.closure_winding().is_some()
        || use_.seam().is_some()
        || use_.chart().period_shifts()[1] != 0
    {
        return Err(gap(face_id));
    }
    Ok(())
}

fn traversal_sign(sense: Sense, orientation: PersistentSkewCylinderOpenSpanOrientation) -> f64 {
    let fin = if sense == Sense::Forward { 1.0 } else { -1.0 };
    let logical = if orientation == PersistentSkewCylinderOpenSpanOrientation::Forward {
        1.0
    } else {
        -1.0
    };
    fin * logical
}

fn integrate_persistent_span(
    cylinder: Cylinder,
    anchor: Point3,
    certificate: PersistentSkewCylinderOpenSpanCertificate,
    source_slot: usize,
    chart_offset: Point2,
    sign: f64,
) -> Option<SpanIntegral> {
    let residual = certificate.residual_certificate();
    let guarded = residual.carrier_range();
    let corridors = certificate.root_corridors();
    let lower_width = Interval::point(guarded.lo) - corridors[0].root_parameter();
    let upper_width = corridors[1].root_parameter() - Interval::point(guarded.hi);
    if lower_width.lo() <= 0.0 || upper_width.lo() <= 0.0 {
        return None;
    }

    let mut total = integrate_cell(
        cylinder,
        anchor,
        corridors[0].corridor().pcurves()[source_slot],
        lower_width,
        chart_offset,
    )?;
    for index in 0..SKEW_CYLINDER_BRANCH_PROOF_SEGMENTS {
        let cell = residual.certify_pcurve_cell(index).ok()?;
        let parameter = cell.parameter();
        if parameter.lo() == parameter.hi() {
            continue;
        }
        let raw_width = Interval::point(parameter.hi()) - Interval::point(parameter.lo());
        let width = Interval::new(raw_width.lo().max(0.0), raw_width.hi());
        let next = integrate_cell(
            cylinder,
            anchor,
            cell.pcurves()[source_slot],
            width,
            chart_offset,
        )?;
        total = total.add(next);
    }
    let upper = integrate_cell(
        cylinder,
        anchor,
        corridors[1].corridor().pcurves()[source_slot],
        upper_width,
        chart_offset,
    )?;
    total = total.add(upper);
    scale_integral(total, sign)
}

fn integrate_cell(
    cylinder: Cylinder,
    anchor: Point3,
    pcurve: SkewCylinderBranchPcurveEnclosure,
    width: Interval,
    chart_offset: Point2,
) -> Option<SpanIntegral> {
    integrate_enclosed_cell(
        cylinder,
        anchor,
        pcurve.source_uv(),
        pcurve.source_derivative(),
        width,
        chart_offset,
    )
}

fn integrate_enclosed_cell(
    cylinder: Cylinder,
    anchor: Point3,
    uv: [Interval; 2],
    derivative: [Interval; 2],
    width: Interval,
    chart_offset: Point2,
) -> Option<SpanIntegral> {
    if !finite_interval(width) || width.lo() < 0.0 || width.hi() <= 0.0 {
        return None;
    }
    let [mut u, mut v] = uv;
    let [du, _dv] = derivative;
    u = u + Interval::point(chart_offset.x);
    v = v + Interval::point(chart_offset.y);
    if ![u, v, du].into_iter().all(finite_interval) {
        return None;
    }

    let frame = cylinder.frame();
    let radius = Interval::point(cylinder.radius());
    let relative = relative_coordinates(frame.origin(), anchor);
    let x = point_vec(frame.x());
    let y = point_vec(frame.y());
    let axis = point_vec(frame.z());
    let cosine = trig_enclosure(u, false)?;
    let sine = trig_enclosure(u, true)?;
    let radial: [Interval; 3] =
        core::array::from_fn(|coordinate| radius * (x[coordinate] * cosine + y[coordinate] * sine));
    let base: [Interval; 3] =
        core::array::from_fn(|coordinate| relative[coordinate] + radial[coordinate]);
    let h = radius.square() + interval_dot(radial, relative);
    let v2 = v.square();
    let v3 = v2 * v;
    let line_measure = Interval::point(-1.0) * du * width;
    let signed_parameter_area = v * line_measure;
    let volume = h * v * line_measure * interval_ratio(1.0, 3.0);
    let moment = core::array::from_fn(|coordinate| {
        h * (base[coordinate] * v + axis[coordinate] * v2 * interval_ratio(1.0, 2.0))
            * line_measure
            * interval_ratio(1.0, 4.0)
    });
    let second_moment = core::array::from_fn(|component| {
        let (left, right) = super::inertia::SYMMETRIC_COMPONENTS[component];
        let linear = h * base[left] * base[right];
        let quadratic =
            h * (base[left] * axis[right] + base[right] * axis[left]) * interval_ratio(1.0, 2.0);
        let cubic = h * axis[left] * axis[right] * interval_ratio(1.0, 3.0);
        (linear * v + quadratic * v2 + cubic * v3) * line_measure * interval_ratio(1.0, 5.0)
    });
    let integral = SpanIntegral {
        signed_parameter_area,
        flux: Flux {
            volume,
            moment,
            second_moment,
        },
    };
    integral.finite().then_some(integral)
}

fn trig_enclosure(angle: Interval, sine: bool) -> Option<Interval> {
    if !finite_interval(angle) {
        return None;
    }
    let midpoint = 0.5 * angle.lo() + 0.5 * angle.hi();
    if !midpoint.is_finite() {
        return None;
    }
    let value = if sine {
        math::sin(midpoint)
    } else {
        math::cos(midpoint)
    };
    let radius = (angle.hi() - angle.lo()).abs().next_up();
    let enclosure =
        Interval::new(value.next_down(), value.next_up()) + Interval::new(-radius, radius);
    let result = Interval::new(enclosure.lo().max(-1.0), enclosure.hi().min(1.0));
    finite_interval(result).then_some(result)
}

fn scale_integral(integral: SpanIntegral, sign: f64) -> Option<SpanIntegral> {
    if sign != 1.0 && sign != -1.0 {
        return None;
    }
    let factor = Interval::point(sign);
    let scaled = SpanIntegral {
        signed_parameter_area: integral.signed_parameter_area * factor,
        flux: Flux {
            volume: integral.flux.volume * factor,
            moment: integral.flux.moment.map(|value| value * factor),
            second_moment: integral.flux.second_moment.map(|value| value * factor),
        },
    };
    scaled.finite().then_some(scaled)
}

fn gap(face: FaceId) -> BodyPropertiesRefusal {
    BodyPropertiesRefusal::UncertifiedAnalyticBoundary { face }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::Curve2dGeom;
    use kgeom::curve2d::Line2d;
    use kgeom::frame::Frame;
    use kgeom::vec::Vec2;

    fn contains_analytic_representative(outer: Interval, inner: Interval) -> bool {
        let representative = 0.5 * inner.lo() + 0.5 * inner.hi();
        outer.contains(representative)
    }

    fn contains_analytic_integral(outer: SpanIntegral, inner: SpanIntegral) -> bool {
        contains_analytic_representative(outer.signed_parameter_area, inner.signed_parameter_area)
            && contains_analytic_representative(outer.flux.volume, inner.flux.volume)
            && outer
                .flux
                .moment
                .into_iter()
                .zip(inner.flux.moment)
                .all(|(outer, inner)| contains_analytic_representative(outer, inner))
            && outer
                .flux
                .second_moment
                .into_iter()
                .zip(inner.flux.second_moment)
                .all(|(outer, inner)| contains_analytic_representative(outer, inner))
    }

    #[test]
    fn fixed_cell_flux_encloses_the_independent_exact_line_form_in_both_directions() {
        let cylinder = Cylinder::new(Frame::world(), 1.25).unwrap();
        let anchor = Point3::new(-0.25, 0.5, -0.75);
        let end = 0.125;
        let height = 0.7;
        let curve =
            Curve2dGeom::Line(Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap());
        let exact_forward = integrate_cylinder_span(
            cylinder,
            anchor,
            BoundedPcurveSpan::new(&curve, 0.0, end, Point2::default()),
        )
        .unwrap();
        let exact_reversed = integrate_cylinder_span(
            cylinder,
            anchor,
            BoundedPcurveSpan::new(&curve, end, 0.0, Point2::default()),
        )
        .unwrap();
        let enclosed = integrate_enclosed_cell(
            cylinder,
            anchor,
            [Interval::new(0.0, end), Interval::point(height)],
            [Interval::point(1.0), Interval::point(0.0)],
            Interval::point(end),
            Point2::default(),
        )
        .unwrap();
        assert!(
            contains_analytic_integral(enclosed, exact_forward),
            "cell {enclosed:?}, exact {exact_forward:?}"
        );
        assert!(contains_analytic_integral(
            scale_integral(enclosed, -1.0).unwrap(),
            exact_reversed
        ));
    }
}
