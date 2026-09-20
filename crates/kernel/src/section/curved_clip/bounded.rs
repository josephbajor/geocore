//! Clip closed circles against complete topology-owned bounded analytic loops.

use super::*;

pub(super) fn crossings(
    store: &Store,
    face: RawFaceId,
    loop_id: RawLoopId,
    circle: SectionUvCircle,
    carrier_range: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<(Vec<Crossing>, bool), ClosedConicClipGap>> {
    let ring = read(store.get(loop_id))?;
    let work = ktopo::bounded_trim::preparation_work(ring.fins().len()).ok_or(
        crate::error::Error::InconsistentTopology {
            source: kcore::error::Error::InvalidGeometry {
                reason: "bounded trim work overflow",
            },
        },
    )?;
    charge(scope, work)?;
    let Some(trim) = read(ktopo::bounded_trim::prepare(store, face, loop_id))? else {
        return Ok(Err(ClosedConicClipGap::UnsupportedTrim));
    };
    let source = match circle_source(circle, carrier_range) {
        Ok(source) => source,
        Err(gap) => return Ok(Err(gap)),
    };
    let seam = seam_point(&source);
    let Some(work) = trim.query_work() else {
        return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
    };
    charge(scope, work)?;
    let Some(inside) = trim.parity([seam.x, seam.y]) else {
        return Ok(Err(ClosedConicClipGap::ParameterSeamContact));
    };
    let mut crossings = Vec::new();
    for span in trim.spans() {
        if matches!(span.curve, Curve2dGeom::Line(_)) {
            let fin = read(store.get(span.fin))?;
            let edge = read(store.get(span.edge))?;
            let Some((lo, hi)) = edge.bounds else {
                return Ok(Err(ClosedConicClipGap::MalformedTrim));
            };
            let (Some(start), Some(end)) = (
                span.enclose(Interval::point(span.parameters[0])),
                span.enclose(Interval::point(span.parameters[1])),
            ) else {
                return Ok(Err(ClosedConicClipGap::ArithmeticGuard));
            };
            let segment = PlaneTrimSegment {
                face,
                loop_id,
                fin: span.fin,
                edge: span.edge,
                start: IntervalPoint2 {
                    x: start[0],
                    y: start[1],
                },
                end: IntervalPoint2 {
                    x: end[0],
                    y: end[1],
                },
                edge_parameters: if fin.sense.is_forward() {
                    [lo, hi]
                } else {
                    [hi, lo]
                },
            };
            match segment_crossings(&source, &segment, scope)? {
                Ok(found) => crossings.extend(found),
                Err(gap) => return Ok(Err(gap)),
            }
            continue;
        }
        if !matches!(span.curve, Curve2dGeom::Circle(_)) {
            return Ok(Err(ClosedConicClipGap::UnsupportedTrim));
        }
        match super::super::circle_disk_clip::bounded_arc_crossings(
            store,
            face,
            loop_id,
            span.fin,
            circle,
            carrier_range,
            scope,
        )? {
            Ok(sites) => crossings.extend(sites.into_iter().map(|site| Crossing { site })),
            Err(gap) => return Ok(Err(gap)),
        }
    }
    Ok(Ok((crossings, inside)))
}
