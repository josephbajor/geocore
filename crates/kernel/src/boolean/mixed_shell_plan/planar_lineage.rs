//! Revalidate source-loop and root lineage before shell assembly.

use super::*;

pub(super) fn validate_planar_lineage(
    store: &Store,
    graph: &BodySectionGraph,
    face: &FaceId,
    operand: usize,
    arrangement: &MixedPlanarFaceArrangement,
    lineage: &MixedPlanarSourceLineage,
    source: MixedSourceFaceKey,
) -> Result<Vec<MixedBoundedSourceSpanPlan>, MixedShellPlanError> {
    let fail = || MixedShellPlanError::PlanarLineageMismatch(source);
    let raw_face = store.get(face.raw()).map_err(|_| fail())?;
    let loop_id = lineage.spans().first().ok_or_else(fail)?.loop_id();
    let mut covered = Vec::new();
    for span in lineage.spans() {
        if !covered.contains(&span.loop_id()) {
            covered.push(span.loop_id());
        }
    }
    for &(ring, cell) in &lineage.retained_rings {
        if covered.contains(&ring)
            || !arrangement
                .cells()
                .iter()
                .any(|candidate| candidate.key() == cell)
        {
            return Err(fail());
        }
        covered.push(ring);
    }
    if covered.len() != raw_face.loops().len()
        || raw_face.loops().iter().any(|ring| !covered.contains(ring))
    {
        return Err(fail());
    }
    let ordered_loops = std::iter::once(loop_id)
        .chain(raw_face.loops().iter().copied().filter(|id| *id != loop_id));
    let mut fins = Vec::new();
    for ring in ordered_loops {
        fins.extend(store.get(ring).map_err(|_| fail())?.fins().iter().copied());
    }
    let mut expected_vertices = Vec::new();
    for fin_id in &fins {
        let fin = store.get(*fin_id).map_err(|_| fail())?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        if edge.vertices() == [None, None] {
            continue;
        }
        let [Some(first), Some(second)] = edge.vertices() else {
            return Err(fail());
        };
        let pair = if fin.sense() == ktopo::entity::Sense::Forward {
            [first, second]
        } else {
            [second, first]
        };
        for vertex in pair {
            if !expected_vertices.contains(&vertex) {
                expected_vertices.push(vertex);
            }
        }
    }
    if lineage.source_vertices() != expected_vertices
        || lineage.spans().len() != arrangement.source_spans().len()
    {
        return Err(fail());
    }
    let mut seen = BTreeSet::new();
    let mut bounded = Vec::new();
    for span in arrangement.source_spans() {
        let candidates = lineage
            .spans()
            .iter()
            .filter(|candidate| candidate.key() == span.key())
            .collect::<Vec<_>>();
        let [candidate] = candidates.as_slice() else {
            return Err(fail());
        };
        if !seen.insert(span.key().clone()) {
            return Err(fail());
        }
        let fin_id = *fins.get(span.key().fin_loop_ordinal).ok_or_else(fail)?;
        let fin = store.get(fin_id).map_err(|_| fail())?;
        let edge = store.get(fin.edge()).map_err(|_| fail())?;
        if candidate.loop_id() != fin.parent()
            || candidate.fin() != fin_id
            || candidate.edge() != fin.edge()
        {
            return Err(fail());
        }
        for (vertex, evidence) in span.endpoints().into_iter().zip(candidate.range()) {
            match (vertex, evidence) {
                (
                    MixedArrangementVertex::SourceVertex(ordinal),
                    MixedSourceParameterEvidence::SourceVertex {
                        topology_ordinal,
                        vertex,
                        edge_parameter_bits,
                    },
                ) => {
                    let [Some(edge_start), Some(edge_end)] = edge.vertices() else {
                        return Err(fail());
                    };
                    let Some((lo, hi)) = edge.bounds() else {
                        return Err(fail());
                    };
                    let expected_parameter = if *vertex == edge_start {
                        lo
                    } else if *vertex == edge_end {
                        hi
                    } else {
                        return Err(fail());
                    };
                    if topology_ordinal != ordinal
                        || lineage.source_vertices().get(*ordinal) != Some(vertex)
                        || *edge_parameter_bits != expected_parameter.to_bits()
                    {
                        return Err(fail());
                    }
                }
                (
                    MixedArrangementVertex::SectionEndpoint(endpoint),
                    MixedSourceParameterEvidence::SectionRoot {
                        endpoint: claimed,
                        root_ordinal,
                        enclosure_bits,
                        period_shift,
                    },
                ) => {
                    let section = graph.curve_endpoints().get(*endpoint).ok_or_else(fail)?;
                    let SectionCurveEndpointTopology::Trim {
                        source_parameters, ..
                    } = section.topology()
                    else {
                        return Err(fail());
                    };
                    let parameter = source_parameters[operand].as_ref().ok_or_else(fail)?;
                    let enclosure = section.edge_parameters()[operand].ok_or_else(fail)?;
                    if (edge.vertices() != [None, None] && *period_shift != 0)
                        || claimed != endpoint
                        || parameter.edge().raw() != candidate.edge()
                        || parameter.root_ordinal() != *root_ordinal
                        || *enclosure_bits != [enclosure.lo().to_bits(), enclosure.hi().to_bits()]
                    {
                        return Err(fail());
                    }
                }
                _ => return Err(fail()),
            }
        }
        if edge.vertices() == [None, None] {
            let mut roots = Vec::new();
            for evidence in candidate.range() {
                let MixedSourceParameterEvidence::SectionRoot {
                    endpoint,
                    root_ordinal,
                    enclosure_bits,
                    period_shift,
                } = evidence
                else {
                    return Err(fail());
                };
                let SectionCurveEndpointTopology::Trim {
                    source_parameters, ..
                } = graph.curve_endpoints()[*endpoint].topology()
                else {
                    return Err(fail());
                };
                let parameter = source_parameters[operand].as_ref().ok_or_else(fail)?;
                roots.push(MixedBoundedSourceRoot {
                    endpoint: *endpoint,
                    root_ordinal: *root_ordinal,
                    parameter_bits: parameter.root_parameter().to_bits(),
                    enclosure_bits: *enclosure_bits,
                    period_shift: *period_shift,
                });
            }
            let roots: [MixedBoundedSourceRoot; 2] = roots.try_into().map_err(|_| fail())?;
            if intrinsic_circle_period_shifts(
                fin.sense(),
                roots.map(MixedBoundedSourceRoot::parameter),
            ) != Some(roots.map(MixedBoundedSourceRoot::period_shift))
            {
                return Err(fail());
            }
            bounded.push(MixedBoundedSourceSpanPlan {
                source,
                span: candidate.key().clone(),
                loop_id: candidate.loop_id(),
                fin: candidate.fin(),
                edge: candidate.edge(),
                roots,
            });
        }
    }
    Ok(bounded)
}
