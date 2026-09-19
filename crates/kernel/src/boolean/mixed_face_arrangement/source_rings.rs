//! Preserve circular source holes disjoint from all newly arranged cuts.

use super::*;
use kcore::interval::Interval;
use ktopo::incidence_authority::{WholeFinIncidence, certify_whole_fin_incidence};

pub(super) fn polygon_loop(
    store: &Store,
    face: RawFaceId,
) -> Result<RawLoopId, MixedFaceArrangementError> {
    let face = store
        .get(face)
        .map_err(|_| MixedFaceArrangementError::MissingSourceFace)?;
    let mut polygon = None;
    for &loop_id in face.loops() {
        let ring = store
            .get(loop_id)
            .map_err(|_| MixedFaceArrangementError::MissingSourceLoop)?;
        if ring.fins().len() >= 3 {
            if polygon.replace(loop_id).is_some() {
                return Err(MixedFaceArrangementError::MultipleSourceLoops);
            }
        } else if ring.fins().len() != 1 {
            return Err(MixedFaceArrangementError::MultipleSourceLoops);
        }
    }
    polygon.ok_or(MixedFaceArrangementError::EmptySourceLoop)
}

pub(super) fn untouched_rings(
    store: &Store,
    face_id: RawFaceId,
    cuts: &[FaceCutEvidence],
) -> Result<Vec<RawLoopId>, MixedFaceArrangementError> {
    let fail = || MixedFaceArrangementError::MultipleSourceLoops;
    let polygon = polygon_loop(store, face_id)?;
    let face = store.get(face_id).map_err(|_| fail())?;
    let mut rings = Vec::new();
    for &loop_id in face.loops() {
        if loop_id == polygon {
            continue;
        }
        let ring = store.get(loop_id).map_err(|_| fail())?;
        let [fin_id] = ring.fins() else {
            return Err(fail());
        };
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let use_ = fin.pcurve().ok_or_else(fail)?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        let Curve2dGeom::Circle(circle) = store.pcurve(use_.curve()).map_err(|_| fail())? else {
            return Err(fail());
        };
        if fin.sense().times(use_.sense()) != face.sense().flipped()
            || edge.vertices() != [None, None]
            || edge.bounds().is_some()
            || edge.tolerance().is_some()
            || use_.seam().is_some()
            || !use_.chart().is_identity()
            || use_.range().width() != core::f64::consts::TAU
            || use_.edge_to_pcurve().scale().abs() != 1.0
            || use_.closure_winding() != Some([0, 0])
            || certify_whole_fin_incidence(
                store,
                face_id,
                loop_id,
                *fin_id,
                kcore::tolerance::LINEAR_RESOLUTION,
            ) != WholeFinIncidence::Certified
        {
            return Err(fail());
        }
        for cut in cuts {
            let CutEmbedding::WholeCircle {
                center,
                radius,
                x_direction,
                ..
            } = cut.embedding
            else {
                return Err(fail());
            };
            let x = Interval::point(center[0]) - Interval::point(circle.center().x);
            let y = Interval::point(center[1]) - Interval::point(circle.center().y);
            let effective_radius = |radius: f64, x: [f64; 2]| {
                (Interval::point(x[0]).square() + Interval::point(x[1]).square())
                    .sqrt()
                    .map(|gram| Interval::point(radius) * gram)
                    .ok_or_else(fail)
            };
            let radii = effective_radius(radius, x_direction)?
                + effective_radius(circle.radius(), [circle.x_dir().x, circle.x_dir().y])?;
            if (x.square() + y.square()).lo() <= radii.square().hi() {
                return Err(fail());
            }
        }
        rings.push(loop_id);
    }
    Ok(rings)
}
