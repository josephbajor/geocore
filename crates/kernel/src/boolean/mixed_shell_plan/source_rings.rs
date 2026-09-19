//! Retain unaffected source holes through the shared shell plan.

use super::super::mixed_cap_boundary::bind_annulus_source_ring;
use super::*;

pub(super) fn append_for_cell(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: &BTreeMap<MixedSourceFaceKey, MixedArrangementBinding<'_>>,
    binding: &MixedArrangementBinding<'_>,
    cell: usize,
    loops: &mut Vec<MixedShellLoopPlan>,
    cap_rings: &mut Vec<MixedCylinderCapRing>,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::AxialContactBoundaryMismatch;
    let MixedArrangementBinding::Planar {
        face,
        operand,
        lineage,
        ..
    } = binding
    else {
        return Err(fail());
    };
    let source = source_face_key(store, graph, face, *operand)?;
    for &(loop_id, owner) in &lineage.retained_rings {
        if owner != cell {
            continue;
        }
        let raw_loop = store.get(loop_id).map_err(|_| fail())?;
        let [fin_id] = raw_loop.fins() else {
            return Err(fail());
        };
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        let mate = edge
            .fins()
            .iter()
            .find(|id| **id != *fin_id)
            .ok_or_else(fail)?;
        let mate = store.get(*mate).map_err(|_| fail())?;
        let side_loop = store.get(mate.parent()).map_err(|_| fail())?;
        let periodic = bindings
            .values()
            .find_map(|binding| match binding {
                MixedArrangementBinding::Periodic {
                    face: side,
                    operand: side_operand,
                    arrangement,
                    embedding: Some(embedding),
                } if side.raw() == side_loop.face() && side_operand == operand => {
                    Some((side, arrangement, embedding))
                }
                _ => None,
            })
            .ok_or_else(fail)?;
        let ring = bind_annulus_source_ring(
            store,
            graph,
            periodic.0,
            *operand,
            mate.parent(),
            periodic.1,
            periodic.2,
        )
        .map_err(|_| fail())?;
        let proof = ProjectedEndpointFreeSourceCircle::certify(
            store,
            &ring,
            source,
            face,
            kcore::tolerance::LINEAR_RESOLUTION,
        )
        .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
        let seam = MixedShellVertexKey::ProofSeam {
            source: ring.side_source(),
            loop_key: ring.side_loop_key(),
        };
        loops.push(MixedShellLoopPlan {
            uses: vec![MixedShellEdgeUse {
                edge: MixedShellEdgeKey::PeriodicSource {
                    source: ring.side_source(),
                    loop_key: ring.side_loop_key(),
                },
                direction: ArrangementDirection::Forward,
                pcurve: MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof),
            }],
            vertices: vec![seam.clone(), seam],
        });
        cap_rings.push(ring);
    }
    Ok(())
}
