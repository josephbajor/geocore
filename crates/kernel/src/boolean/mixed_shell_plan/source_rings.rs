//! Retain unaffected source holes through the shared shell plan.

use super::super::mixed_cap_boundary::bind_annulus_source_ring;
use super::*;

pub(super) fn attach(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: &[MixedArrangementBinding<'_>],
    arrangement: &mut MixedShellArrangement<'_>,
) -> Result<(), MixedShellPlanError> {
    let fail = || MixedShellPlanError::AxialContactBoundaryMismatch;
    for binding in bindings {
        let MixedArrangementBinding::Planar {
            face,
            operand,
            lineage,
            arrangement: planar,
            ..
        } = binding
        else {
            continue;
        };
        let source = source_face_key(store, graph, face, *operand)?;
        for &(loop_id, cell) in &lineage.retained_rings {
            let owner = planar
                .cells()
                .iter()
                .find(|candidate| candidate.key() == cell)
                .ok_or_else(fail)?;
            let mut targets = arrangement.faces.iter_mut().filter(|target| {
                target.source == source
                    && target.loops.iter().any(|ring| {
                        ring.uses
                            .iter()
                            .any(|use_| match &use_.edge {
                                MixedShellEdgeKey::PlanarSource { span, .. } => owner.boundaries().iter()
                                    .any(|cycle| cycle.uses().iter().any(|owned| matches!(
                                        owned.edge(), ArrangementEdgeKey::Source(key) if key == span))),
                                _ => false,
                            })
                    })
            });
            let Some(target) = targets.next() else {
                continue;
            };
            if targets.next().is_some() {
                return Err(fail());
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
                .iter()
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
            target.loops.push(MixedShellLoopPlan {
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
            arrangement.cap_rings.push(ring);
        }
    }
    Ok(())
}
