//! Publication adapters for proof-bearing Plane/Cylinder branches.
//!
//! Discovery stays source-order deterministic; unsupported descriptors and
//! indeterminate orientation evidence continue to fail closed as section gaps.

use super::*;

/// Collect proof-bearing Plane/Cylinder circle and ruling-line carriers
/// independently of the planar trim/stitch admission path.
///
/// Face domains are conservative source-owned surface windows used only for
/// analytic branch discovery and paired trace proof. Exact membership is
/// decided afterward from topology-owned loops, fins, edges, and pcurves.
#[allow(clippy::too_many_arguments)]
pub(super) fn collect_plane_cylinder_branches(
    store: &Store,
    part_id: &PartId,
    faces_a: &[RawFaceId],
    faces_b: &[RawFaceId],
    envelopes_a: &[broad_phase::FaceEnvelope],
    envelopes_b: &[broad_phase::FaceEnvelope],
    linear: f64,
    examined: &mut u64,
    root_identity: &mut root_identity::RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    for (a_index, &raw_a) in faces_a.iter().enumerate() {
        let face_a = read(store.get(raw_a))?;
        let surface_a = read(store.surface(face_a.surface))?;
        for (b_index, &raw_b) in faces_b.iter().enumerate() {
            let face_b = read(store.get(raw_b))?;
            let surface_b = read(store.surface(face_b.surface))?;
            if !matches!(
                (surface_a, surface_b),
                (SurfaceGeom::Plane(_), SurfaceGeom::Cylinder(_))
                    | (SurfaceGeom::Cylinder(_), SurfaceGeom::Plane(_))
            ) {
                continue;
            }
            *examined += 1;
            scope
                .ledger_mut()
                .observe(SECTION_FACE_PAIRS, ResourceKind::Items, *examined)
                .map_err(Error::from)?;
            charge(scope, 1)?;
            if broad_phase::certifiably_disjoint(envelopes_a[a_index], envelopes_b[b_index], linear)
            {
                continue;
            }
            let facades = [
                FaceId::new(part_id.clone(), raw_a),
                FaceId::new(part_id.clone(), raw_b),
            ];
            let (Some(domain_a), Some(domain_b)) = (face_a.domain(), face_b.domain()) else {
                acc.gaps.push(SectionGap {
                    reason: GAP_PAIR_UNRESOLVED,
                    faces: facades.to_vec(),
                });
                continue;
            };
            let discovery_domains = circle_discovery::plane_cylinder_discovery_domains(
                [surface_a, surface_b],
                [[domain_a.u, domain_a.v], [domain_b.u, domain_b.v]],
            );
            let intersections = match intersect_bounded_graph_surfaces_in_scope(
                store.geometry(),
                face_a.surface,
                discovery_domains[0],
                face_b.surface,
                discovery_domains[1],
                scope,
            ) {
                Ok(intersections) => intersections,
                Err(error) => {
                    if let Some(error) = lift_limit_error(error.clone()) {
                        return Err(error);
                    }
                    if let Some(branches) = semantic_ruling::recover(
                        store,
                        [raw_a, raw_b],
                        &facades,
                        [surface_a, surface_b],
                        [face_a.sense, face_b.sense],
                        discovery_domains,
                        &error,
                        scope,
                    )? {
                        for branch in branches {
                            ruling_publish::append_branch(
                                store,
                                [raw_a, raw_b],
                                &facades,
                                branch,
                                root_identity,
                                scope,
                                acc,
                            )?;
                        }
                    } else {
                        acc.gaps.push(SectionGap {
                            reason: GAP_PAIR_UNRESOLVED,
                            faces: facades.to_vec(),
                        });
                    }
                    continue;
                }
            };
            if !intersections.raw.is_complete()
                || !intersections.raw.points.is_empty()
                || !intersections.raw.regions.is_empty()
            {
                acc.gaps.push(SectionGap {
                    reason: GAP_PAIR_UNRESOLVED,
                    faces: facades.to_vec(),
                });
                continue;
            }
            for edge in &intersections.branch_graph.edges {
                append_plane_cylinder_branch(
                    store,
                    [raw_a, raw_b],
                    &facades,
                    edge,
                    &intersections.branch_graph.vertices,
                    [surface_a, surface_b],
                    [face_a.sense, face_b.sense],
                    root_identity,
                    scope,
                    acc,
                )?;
            }
        }
    }
    Ok(())
}

/// Adapt and topology-clip one graph-certified Plane/Cylinder branch.
#[allow(clippy::too_many_arguments)]
fn append_plane_cylinder_branch(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    root_identity: &mut root_identity::RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    let branch = match adapt_plane_cylinder_branch(
        facades,
        edge,
        vertices,
        surfaces[0],
        senses[0],
        surfaces[1],
        senses[1],
    ) {
        PlaneCylinderBranchAdaptation::Adapted(branch) => *branch,
        PlaneCylinderBranchAdaptation::OrientationIndeterminate => {
            acc.gaps.push(SectionGap {
                reason: GAP_CARRIER_ORIENTATION,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
        PlaneCylinderBranchAdaptation::Unsupported => {
            acc.gaps.push(SectionGap {
                reason: GAP_PAIR_UNRESOLVED,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
    };
    if matches!(branch.carrier, SectionCarrier::Line { .. }) {
        return ruling_publish::append_branch(
            store,
            raw_faces,
            facades,
            branch,
            root_identity,
            scope,
            acc,
        );
    }
    let clipped = [
        curved_clip::clip_closed_conic_to_face(
            store,
            raw_faces[0],
            branch.pcurves[0],
            branch.range,
            scope,
        )?,
        curved_clip::clip_closed_conic_to_face(
            store,
            raw_faces[1],
            branch.pcurves[1],
            branch.range,
            scope,
        )?,
    ];
    let trim = merge_closed_trim_outcomes(&clipped[0], &clipped[1]);
    let branch_index = acc.branches.len();
    acc.branches.push(branch);
    match trim {
        ClosedTrimMerge::Empty => {}
        ClosedTrimMerge::Fragments(fragments) => {
            if let Err(reason) =
                append_closed_fragments(store, branch_index, &fragments, root_identity, scope, acc)?
            {
                acc.gaps.push(SectionGap {
                    reason,
                    faces: facades.to_vec(),
                });
            }
        }
        ClosedTrimMerge::UnsupportedIntersection => acc.gaps.push(SectionGap {
            reason: GAP_CURVED_TRIM_UNRESOLVED,
            faces: facades.to_vec(),
        }),
        ClosedTrimMerge::Gap(reason) => acc.gaps.push(SectionGap {
            reason,
            faces: facades.to_vec(),
        }),
    }
    Ok(())
}

enum PlaneCylinderBranchAdaptation {
    Adapted(Box<SectionBranch>),
    OrientationIndeterminate,
    Unsupported,
}

/// Adapt one verified graph branch into facade-owned carrier values.
fn adapt_plane_cylinder_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
) -> PlaneCylinderBranchAdaptation {
    if edge.kind != ContactKind::Transverse {
        return PlaneCylinderBranchAdaptation::Unsupported;
    }
    if edge.certificate.as_plane_cylinder_circle().is_some() {
        return adapt_plane_cylinder_circle_branch(
            faces, edge, vertices, surface_a, sense_a, surface_b, sense_b,
        );
    }
    if edge.certificate.as_plane_cylinder_ruling().is_some() {
        return adapt_plane_cylinder_ruling_branch(
            faces, edge, vertices, surface_a, sense_a, surface_b, sense_b,
        );
    }
    PlaneCylinderBranchAdaptation::Unsupported
}

#[allow(clippy::too_many_arguments)]
fn adapt_plane_cylinder_circle_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
) -> PlaneCylinderBranchAdaptation {
    let Some(certificate) = edge.certificate.as_plane_cylinder_circle() else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let CurveDescriptor::Circle(carrier) = edge.carrier else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    if edge.topology != IntersectionBranchTopology::Closed
        || edge.endpoint_vertices[0] != edge.endpoint_vertices[1]
    {
        return PlaneCylinderBranchAdaptation::Unsupported;
    }
    let Some(flipped) =
        canonical_plane_cylinder_circle_flip(surface_a, sense_a, surface_b, sense_b)
    else {
        return PlaneCylinderBranchAdaptation::OrientationIndeterminate;
    };
    let Some(pcurves) = adapt_branch_pcurves(edge, flipped) else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let Some(vertex) = vertices.get(edge.endpoint_vertices[0]).copied() else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::PeriodSeam { surfaces } = vertex.event else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    PlaneCylinderBranchAdaptation::Adapted(Box::new(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::Circle {
            center: carrier.frame().origin(),
            normal: if flipped {
                -carrier.frame().z()
            } else {
                carrier.frame().z()
            },
            x_direction: carrier.frame().x(),
            radius: carrier.radius(),
        },
        range: edge.carrier_range,
        topology: SectionBranchTopology::Closed,
        pcurves,
        fragment_sites: vec![SectionFragmentSite {
            point: vertex.point,
            surface_parameters: vertex.surface_parameters,
            surface_window_boundaries: surfaces,
        }],
        endpoint_sites: [0, 0],
        evidence: SectionBranchEvidence {
            residual_bounds: certificate.residual_bounds(),
            tolerance: certificate.tolerance(),
        },
        ruling_recertification: None,
        ruling_parameter_flipped: false,
    }))
}

#[allow(clippy::too_many_arguments)]
fn adapt_plane_cylinder_ruling_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
) -> PlaneCylinderBranchAdaptation {
    let Some(certificate) = edge.certificate.as_plane_cylinder_ruling() else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let CurveDescriptor::Line(carrier) = edge.carrier else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    if edge.topology != IntersectionBranchTopology::Open
        || edge.endpoint_vertices[0] == edge.endpoint_vertices[1]
    {
        return PlaneCylinderBranchAdaptation::Unsupported;
    }
    let Some(flipped) = canonical_plane_cylinder_ruling_flip(
        surface_a,
        sense_a,
        surface_b,
        sense_b,
        carrier.origin(),
        carrier.dir(),
    ) else {
        return PlaneCylinderBranchAdaptation::OrientationIndeterminate;
    };
    let Some(pcurves) = adapt_branch_pcurves(edge, flipped) else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let Some(low_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(flipped)])
        .copied()
    else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let Some(high_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(!flipped)])
        .copied()
    else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: low_surfaces,
    } = low_vertex.event
    else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: high_surfaces,
    } = high_vertex.event
    else {
        return PlaneCylinderBranchAdaptation::Unsupported;
    };
    let range = if flipped {
        ParamRange {
            lo: -edge.carrier_range.hi,
            hi: -edge.carrier_range.lo,
        }
    } else {
        edge.carrier_range
    };
    PlaneCylinderBranchAdaptation::Adapted(Box::new(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::Line {
            origin: carrier.origin(),
            direction: if flipped {
                -carrier.dir()
            } else {
                carrier.dir()
            },
        },
        range,
        topology: SectionBranchTopology::Open,
        pcurves,
        fragment_sites: vec![
            SectionFragmentSite {
                point: low_vertex.point,
                surface_parameters: low_vertex.surface_parameters,
                surface_window_boundaries: low_surfaces,
            },
            SectionFragmentSite {
                point: high_vertex.point,
                surface_parameters: high_vertex.surface_parameters,
                surface_window_boundaries: high_surfaces,
            },
        ],
        endpoint_sites: [0, 1],
        evidence: SectionBranchEvidence {
            residual_bounds: certificate.residual_bounds(),
            tolerance: certificate.tolerance(),
        },
        ruling_recertification: Some(RulingRecertification::Graph(certificate)),
        ruling_parameter_flipped: flipped,
    }))
}

fn adapt_branch_pcurves(
    edge: &IntersectionBranchEdge,
    flipped: bool,
) -> Option<[SectionUvCurve; 2]> {
    let pcurves = [
        adapt_branch_pcurve(&edge.pcurves[0], edge.parameter_maps[0], flipped)?,
        adapt_branch_pcurve(&edge.pcurves[1], edge.parameter_maps[1], flipped)?,
    ];
    Some(pcurves)
}

/// Compose graph-owned pcurve geometry with its carrier map into facade-owned
/// exact values. Unsupported descriptor families fail closed.
fn adapt_branch_pcurve(
    descriptor: &kgraph::Curve2dDescriptor,
    map: AffineParamMap1d,
    flipped: bool,
) -> Option<SectionUvCurve> {
    if let Some(line) = descriptor.as_line() {
        return Some(SectionUvCurve::Line(compose_uv_line(
            line.origin(),
            line.dir(),
            map,
            flipped,
        )));
    }
    let circle = descriptor.as_circle()?;
    Some(SectionUvCurve::Circle(SectionUvCircle {
        center: circle.center(),
        radius: circle.radius(),
        x_direction: circle.x_dir(),
        parameter_scale: if flipped { -map.scale() } else { map.scale() },
        parameter_offset: map.offset(),
    }))
}
