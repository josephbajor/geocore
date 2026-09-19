//! Plan shared source and derived rings from certified axial intervals.

use super::*;

#[derive(Debug, Clone, Copy)]
pub(crate) struct InternalTangencyArrangementGeometry<'a> {
    pub(crate) contained_operand: usize,
    pub(crate) axial_parameters: [[f64; 2]; 2],
    pub(crate) preorder: &'a super::super::axial_interval_sweep::CertifiedAxialEndpointPreorder,
}

fn valid_internal_tangency_geometry(geometry: InternalTangencyArrangementGeometry<'_>) -> bool {
    geometry.contained_operand < 2
        && geometry
            .axial_parameters
            .iter()
            .all(|boundaries| boundaries.iter().all(|parameter| parameter.is_finite()))
}

#[derive(Debug, Clone, Copy)]
struct InternalTangencyBoundary {
    operand: usize,
    boundary: usize,
    axial_parameter: f64,
}

#[derive(Debug, Clone, Copy)]
struct InternalTangencyBand {
    operand: usize,
    low: InternalTangencyBoundary,
    high: InternalTangencyBoundary,
    split: Option<AnalyticFaceSplitPiece>,
}

pub(crate) fn arrange_internal_tangency_bands_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    geometry: InternalTangencyArrangementGeometry<'_>,
    cylinders: [&super::super::curved_source::CertifiedCylinderSource; 2],
    interval: &super::super::axial_interval_sweep::AxialIntervalPlan,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellArrangement<'a>, MixedShellPlanError> {
    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    if !valid_internal_tangency_geometry(geometry) || interval.spans().len() > 2 {
        return Err(fail());
    }
    let mut arrangement = arrange_selected_mixed_shell(
        store,
        graph,
        bindings,
        selected.into_iter().map(selected_cell),
        false,
    )?;
    let faces = &mut arrangement.faces;
    let rings = &mut arrangement.cap_rings;
    let derived = &mut arrangement.derived_rings;
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let contained = geometry.contained_operand;
    let source = cylinders.get(contained).ok_or_else(fail)?;
    faces.clear();
    rings.clear();
    for (span_index, span) in interval.spans().iter().enumerate() {
        let endpoints = [span.low(), span.high()]
            .map(|contributors| bind_internal_boundary_class(geometry, contributors))
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        let [low, high] = endpoints.as_slice() else {
            return Err(fail());
        };
        let centers = [
            internal_axis_endpoint(cylinders, source, contained, low)?,
            internal_axis_endpoint(cylinders, source, contained, high)?,
        ];
        if centers[0].1 == centers[1].1 {
            return Err(fail());
        }
        let mut ring_indices = [0_usize; 2];
        for end in 0..2 {
            let circle = Circle::new(
                source.cylinder().frame().with_origin(centers[end].0),
                source.cylinder().radius(),
            )
            .map_err(|_| fail())?;
            let lineage =
                internal_ring_lineage(cylinders, source.side_face(), contained, &endpoints[end])?;
            ring_indices[end] = derived.len();
            derived.push(MixedDerivedRingPlan::endpoint_free(circle, lineage));
        }
        let side_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == contained)
            .ok_or_else(fail)?;
        let mut side = source_face(
            &source_faces,
            side_ring.side_source(),
            side_ring.side_face(),
        )?;
        let desired = if centers[0].1 < centers[1].1 {
            [ArrangementDirection::Forward, ArrangementDirection::Reverse]
        } else {
            [ArrangementDirection::Reverse, ArrangementDirection::Forward]
        };
        let directions: [ArrangementDirection; 2] = core::array::from_fn(|end| {
            if derived_ring_cylinder_scale(
                derived[ring_indices[end]].circle(),
                *source.cylinder().frame(),
            ) > 0.0
            {
                desired[end]
            } else {
                opposite(desired[end])
            }
        });
        side.loops = (0..2)
            .map(|end| {
                derived_ring_loop(
                    ring_indices[end],
                    directions[end],
                    Some(centers[end].1),
                    derived,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        side.split_lineage = (interval.spans().len() == 2).then_some(if span_index == 0 {
            AnalyticFaceSplitPiece::First
        } else {
            AnalyticFaceSplitPiece::Second
        });
        faces.push(side);
        for end in 0..2 {
            let boundary = endpoints[end]
                .iter()
                .find(|boundary| boundary.operand == contained)
                .or_else(|| endpoints[end].first())
                .ok_or_else(fail)?;
            let source_ring = source_rings
                .iter()
                .find(|ring| {
                    ring.operand() == boundary.operand && ring.boundary() == boundary.boundary
                })
                .ok_or_else(fail)?;
            let mut cap = source_face(
                &source_faces,
                source_ring.cap_source(),
                source_ring.cap_face(),
            )?;
            let sibling = InternalTangencyBoundary {
                operand: boundary.operand,
                boundary: 1 - boundary.boundary,
                axial_parameter: geometry.axial_parameters[boundary.operand][1 - boundary.boundary],
            };
            let source_low = compare_internal_boundaries(geometry, *boundary, sibling)
                == core::cmp::Ordering::Less;
            if source_low != (end == 0) {
                cap.selected_orientation = reverse_selected_orientation(cap.selected_orientation);
            }
            cap.loops = vec![derived_ring_loop(
                ring_indices[end],
                opposite(directions[end]),
                None,
                derived,
            )?];
            faces.push(cap);
        }
    }
    Ok(arrangement)
}

pub(crate) fn arrange_internal_tangency_union_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    geometry: InternalTangencyArrangementGeometry<'_>,
    cylinders: [&super::super::curved_source::CertifiedCylinderSource; 2],
    tails: &[super::super::axial_interval_sweep::PlannedAxialSpan],
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
) -> Result<MixedShellArrangement<'a>, MixedShellPlanError> {
    use core::cmp::Ordering;

    use super::super::axial_interval_sweep::AxialIntervalOperand;

    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    if !valid_internal_tangency_geometry(geometry) || !(1..=2).contains(&tails.len()) {
        return Err(fail());
    }
    let mut arrangement = arrange_selected_mixed_shell(
        store,
        graph,
        bindings,
        selected.into_iter().map(selected_cell),
        false,
    )?;
    let faces = &mut arrangement.faces;
    let rings = &mut arrangement.cap_rings;
    let derived = &mut arrangement.derived_rings;
    let contained = geometry.contained_operand;
    let containing = 1 - geometry.contained_operand;
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let [outer_low, outer_high] = ordered_internal_boundaries(geometry, containing)?;
    let mut bands = Vec::with_capacity(tails.len() + 1);
    for (tail_index, tail) in tails.iter().enumerate() {
        let operand = if contained == 0 {
            AxialIntervalOperand::Left
        } else {
            AxialIntervalOperand::Right
        };
        if !tail.side_operands().contains(operand) {
            return Err(fail());
        }
        let low = single_internal_boundary(bind_internal_boundary_class(geometry, tail.low())?)?;
        let high = single_internal_boundary(bind_internal_boundary_class(geometry, tail.high())?)?;
        bands.push(InternalTangencyBand {
            operand: contained,
            low,
            high,
            split: (tails.len() == 2).then_some(if tail_index == 0 {
                AnalyticFaceSplitPiece::First
            } else {
                AnalyticFaceSplitPiece::Second
            }),
        });
    }
    bands.push(InternalTangencyBand {
        operand: containing,
        low: outer_low,
        high: outer_high,
        split: None,
    });
    bands.sort_by(|first, second| compare_internal_boundaries(geometry, first.low, second.low));
    if bands.windows(2).any(|pair| {
        pair[0].operand == pair[1].operand
            || !same_internal_boundary(pair[0].high, pair[1].low)
            || pair[0].high.operand != containing
    }) {
        return Err(fail());
    }

    let outer = cylinders[containing];
    let inner = cylinders[contained];
    let outer_source_frame = *outer.cylinder().frame();
    let outer_low_center = internal_boundary_center(cylinders, outer_low)?;
    let trial_frame = Frame::new(
        outer_low_center,
        outer_source_frame.z(),
        outer_source_frame.x(),
    )
    .map_err(|_| fail())?;
    let (_, trial_height) = exact_internal_axial_projection(
        trial_frame,
        internal_boundary_center(cylinders, outer_high)?,
    )
    .ok_or_else(fail)?;
    let axis = match trial_height.total_cmp(&0.0) {
        Ordering::Greater => outer_source_frame.z(),
        Ordering::Less => -outer_source_frame.z(),
        Ordering::Equal => return Err(fail()),
    };
    let outer_origin = outer_low_center;
    let inner_origin = exact_internal_axial_projection(*inner.cylinder().frame(), outer_origin)
        .map(|projection| projection.0)
        .ok_or_else(fail)?;
    let radial = inner_origin - outer_origin;
    let outer_frame = Frame::new(outer_origin, axis, radial).map_err(|_| fail())?;
    let inner_frame = outer_frame.with_origin(inner_origin);
    if outer.cylinder().radius() <= inner.cylinder().radius() {
        return Err(fail());
    }

    let prepared = bands
        .iter()
        .map(|band| {
            let source = cylinders[band.operand];
            let frame = if band.operand == containing {
                outer_frame
            } else {
                inner_frame
            };
            let radius = source.cylinder().radius();
            let low_center = internal_boundary_center(cylinders, band.low)?;
            let high_center = internal_boundary_center(cylinders, band.high)?;
            let low = exact_internal_axial_projection(frame, low_center).ok_or_else(fail)?;
            let high = exact_internal_axial_projection(frame, high_center).ok_or_else(fail)?;
            (low.1 < high.1)
                .then_some((*band, frame, radius, low.0, high.0))
                .ok_or_else(fail)
        })
        .collect::<Result<Vec<_>, _>>()?;

    derived.clear();
    let far_boundaries = [prepared[0].0.low, prepared.last().ok_or_else(fail)?.0.high];
    let far_centers = [prepared[0].3, prepared.last().ok_or_else(fail)?.4];
    let far_frames = [prepared[0].1, prepared.last().ok_or_else(fail)?.1];
    let far_radii = [prepared[0].2, prepared.last().ok_or_else(fail)?.2];
    let mut far_ring_indices = [0_usize; 2];
    for end in 0..2 {
        far_ring_indices[end] = derived.len();
        derived.push(MixedDerivedRingPlan::endpoint_free(
            Circle::new(
                far_frames[end].with_origin(far_centers[end]),
                far_radii[end],
            )
            .map_err(|_| fail())?,
            MixedDerivedRingLineage::Source(
                cylinders[far_boundaries[end].operand].boundaries()[far_boundaries[end].boundary]
                    .edge(),
            ),
        ));
    }

    struct Contact {
        vertex: usize,
        outer_ring: usize,
        inner_ring: usize,
        boundary: InternalTangencyBoundary,
        arranged_boundaries: Vec<(usize, ArrangementDirection)>,
    }
    let mut contacts = Vec::with_capacity(tails.len());
    for contact_index in 0..tails.len() {
        let left = prepared[contact_index];
        let right = prepared[contact_index + 1];
        let boundary = left.0.high;
        if boundary.operand != containing || !same_internal_boundary(boundary, right.0.low) {
            return Err(fail());
        }
        let outer_center = if left.0.operand == containing {
            left.4
        } else {
            right.3
        };
        let inner_center = if left.0.operand == contained {
            left.4
        } else {
            right.3
        };
        let outer_circle = Circle::new(
            outer_frame.with_origin(outer_center),
            outer.cylinder().radius(),
        )
        .map_err(|_| fail())?;
        let inner_circle = Circle::new(
            inner_frame.with_origin(inner_center),
            inner.cylinder().radius(),
        )
        .map_err(|_| fail())?;
        let outer_ring = derived.len();
        let inner_ring = outer_ring + 1;
        let tangency = arrange_mixed_periodic_tangency_cell(
            [
                PeriodicTangencyRingKey::new(outer_ring),
                PeriodicTangencyRingKey::new(inner_ring),
            ],
            PeriodicTangencyVertexKey::new(contact_index),
        )
        .map_err(|_| fail())?;
        let [cell] = tangency.cells() else {
            return Err(fail());
        };
        let arranged_boundaries = cell
            .boundaries()
            .iter()
            .map(|cycle| {
                let use_ = cycle.uses().first().ok_or_else(fail)?;
                let ArrangementEdgeKey::Source(ring) = use_.edge() else {
                    return Err(fail());
                };
                Ok((ring.ring(), use_.direction()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let point = outer_circle.eval(0.0);
        derived.push(MixedDerivedRingPlan::tangent(
            outer_circle,
            contact_index,
            point,
            MixedDerivedRingLineage::Source(
                cylinders[containing].boundaries()[boundary.boundary].edge(),
            ),
        ));
        derived.push(MixedDerivedRingPlan::tangent(
            inner_circle,
            contact_index,
            point,
            MixedDerivedRingLineage::Derived([
                inner.side_face(),
                cylinders[containing].boundaries()[boundary.boundary].cap_face(),
            ]),
        ));
        contacts.push(Contact {
            vertex: contact_index,
            outer_ring,
            inner_ring,
            boundary,
            arranged_boundaries,
        });
    }

    faces.clear();
    rings.clear();
    let mut side_directions = Vec::with_capacity(prepared.len());
    for (band_index, (band, _, _, low_center, high_center)) in prepared.iter().enumerate() {
        let source = cylinders[band.operand];
        let native = *source.cylinder().frame();
        let low_parameter = exact_internal_axial_projection(native, *low_center)
            .map(|projection| projection.1)
            .ok_or_else(fail)?;
        let high_parameter = exact_internal_axial_projection(native, *high_center)
            .map(|projection| projection.1)
            .ok_or_else(fail)?;
        let desired = if low_parameter < high_parameter {
            [ArrangementDirection::Forward, ArrangementDirection::Reverse]
        } else if low_parameter > high_parameter {
            [ArrangementDirection::Reverse, ArrangementDirection::Forward]
        } else {
            return Err(fail());
        };
        let ring_indices = [
            if band_index == 0 {
                far_ring_indices[0]
            } else if band.operand == contained {
                contacts[band_index - 1].inner_ring
            } else {
                contacts[band_index - 1].outer_ring
            },
            if band_index + 1 == prepared.len() {
                far_ring_indices[1]
            } else if band.operand == contained {
                contacts[band_index].inner_ring
            } else {
                contacts[band_index].outer_ring
            },
        ];
        let directions: [ArrangementDirection; 2] = core::array::from_fn(|end| {
            if derived_ring_cylinder_scale(
                derived[ring_indices[end]].circle(),
                *source.cylinder().frame(),
            ) > 0.0
            {
                desired[end]
            } else {
                opposite(desired[end])
            }
        });
        let source_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == band.operand)
            .ok_or_else(fail)?;
        let mut face = source_face(
            &source_faces,
            source_ring.side_source(),
            source_ring.side_face(),
        )?;
        face.loops = vec![
            derived_ring_loop(ring_indices[0], directions[0], Some(low_parameter), derived)?,
            derived_ring_loop(
                ring_indices[1],
                directions[1],
                Some(high_parameter),
                derived,
            )?,
        ];
        face.split_lineage = band.split;
        faces.push(face);
        side_directions.push(directions);
    }

    for (end, band_index) in [0, prepared.len() - 1].into_iter().enumerate() {
        let boundary = far_boundaries[end];
        let source_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == boundary.operand && ring.boundary() == boundary.boundary)
            .ok_or_else(fail)?;
        let mut cap = source_face(
            &source_faces,
            source_ring.cap_source(),
            source_ring.cap_face(),
        )?;
        cap.loops = vec![derived_ring_loop(
            far_ring_indices[end],
            opposite(side_directions[band_index][end]),
            None,
            derived,
        )?];
        faces.push(cap);
    }
    for (index, contact) in contacts.iter().enumerate() {
        let left = prepared[index].0;
        let outer_side = if left.operand == containing {
            side_directions[index][1]
        } else {
            side_directions[index + 1][0]
        };
        let inner_side = if left.operand == contained {
            side_directions[index][1]
        } else {
            side_directions[index + 1][0]
        };
        if outer_side == inner_side {
            return Err(fail());
        }
        let source_ring = source_rings
            .iter()
            .find(|ring| {
                ring.operand() == containing && ring.boundary() == contact.boundary.boundary
            })
            .ok_or_else(fail)?;
        let mut shoulder = source_face(
            &source_faces,
            source_ring.cap_source(),
            source_ring.cap_face(),
        )?;
        let uses = contact
            .arranged_boundaries
            .iter()
            .map(|(ring, direction)| {
                let side_direction = if *ring == contact.outer_ring {
                    outer_side
                } else {
                    inner_side
                };
                derived_ring_use(
                    *ring,
                    compose_direction(*direction, opposite(side_direction)),
                    None,
                )
            })
            .collect::<Vec<_>>();
        let vertex = MixedShellVertexKey::Tangency(contact.vertex);
        shoulder.loops = vec![MixedShellLoopPlan {
            vertices: vec![vertex; uses.len() + 1],
            uses,
        }];
        faces.push(shoulder);
    }
    Ok(arrangement)
}

fn source_face(
    faces: &[MixedShellFacePlan],
    source: MixedSourceFaceKey,
    face: &FaceId,
) -> Result<MixedShellFacePlan, MixedShellPlanError> {
    let matching = faces
        .iter()
        .filter(|candidate| candidate.source == source && candidate.source_face == *face)
        .cloned()
        .collect::<Vec<_>>();
    let [face] = matching.as_slice() else {
        return Err(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    };
    let mut face = face.clone();
    face.merge_sources = None;
    face.split_lineage = None;
    Ok(face)
}

fn derived_ring_use(
    ring: usize,
    direction: ArrangementDirection,
    cylinder_parameter: Option<f64>,
) -> MixedShellEdgeUse {
    MixedShellEdgeUse {
        edge: MixedShellEdgeKey::DerivedRing(ring),
        direction,
        pcurve: MixedPcurveLineage::DerivedRing {
            cylinder_parameter_bits: cylinder_parameter.map(f64::to_bits),
        },
    }
}

fn derived_ring_cylinder_scale(circle: Circle, cylinder: Frame) -> f64 {
    let local_x = [
        circle.frame().x().dot(cylinder.x()),
        circle.frame().x().dot(cylinder.y()),
    ];
    let local_y = [
        circle.frame().y().dot(cylinder.x()),
        circle.frame().y().dot(cylinder.y()),
    ];
    if local_x[0] * local_y[1] - local_x[1] * local_y[0] > 0.0 {
        1.0
    } else {
        -1.0
    }
}

const fn reverse_selected_orientation(orientation: SelectedOrientation) -> SelectedOrientation {
    match orientation {
        SelectedOrientation::Preserved => SelectedOrientation::Reversed,
        SelectedOrientation::Reversed => SelectedOrientation::Preserved,
    }
}

fn derived_ring_loop(
    ring: usize,
    direction: ArrangementDirection,
    cylinder_parameter: Option<f64>,
    rings: &[MixedDerivedRingPlan],
) -> Result<MixedShellLoopPlan, MixedShellPlanError> {
    let planned = rings
        .get(ring)
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    let vertex = if let Some((vertex, _)) = planned.tangency() {
        MixedShellVertexKey::Tangency(vertex)
    } else {
        MixedShellVertexKey::DerivedRingSeam(ring)
    };
    Ok(MixedShellLoopPlan {
        uses: vec![derived_ring_use(ring, direction, cylinder_parameter)],
        vertices: vec![vertex.clone(), vertex],
    })
}

fn bind_internal_boundary_class(
    geometry: InternalTangencyArrangementGeometry<'_>,
    contributors: super::super::axial_interval_sweep::AxialEndpointContributors,
) -> Result<Vec<InternalTangencyBoundary>, MixedShellPlanError> {
    use super::super::axial_interval_sweep::{AuthoredAxialEndpoint, AxialIntervalOperand};

    let fail = || MixedShellPlanError::InternalTangencyBoundaryMismatch;
    let mut boundaries = Vec::with_capacity(2);
    for contributor in contributors.iter() {
        let operand = match contributor.operand() {
            AxialIntervalOperand::Left => 0,
            AxialIntervalOperand::Right => 1,
        };
        let boundary = match contributor.endpoint() {
            AuthoredAxialEndpoint::Start => 0,
            AuthoredAxialEndpoint::End => 1,
        };
        if boundaries
            .iter()
            .any(|candidate: &InternalTangencyBoundary| candidate.operand == operand)
        {
            return Err(fail());
        }
        boundaries.push(InternalTangencyBoundary {
            operand,
            boundary,
            axial_parameter: geometry.axial_parameters[operand][boundary],
        });
    }
    (!boundaries.is_empty() && boundaries.len() <= 2)
        .then_some(boundaries)
        .ok_or_else(fail)
}

fn internal_axis_endpoint(
    cylinders: [&super::super::curved_source::CertifiedCylinderSource; 2],
    source: &super::super::curved_source::CertifiedCylinderSource,
    operand: usize,
    boundaries: &[InternalTangencyBoundary],
) -> Result<(Point3, f64), MixedShellPlanError> {
    if let Some(boundary) = boundaries
        .iter()
        .find(|boundary| boundary.operand == operand)
    {
        return Ok((
            source.boundaries()[boundary.boundary].center(),
            boundary.axial_parameter,
        ));
    }
    let boundary = boundaries
        .first()
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    exact_internal_axial_projection(
        *source.cylinder().frame(),
        internal_boundary_center(cylinders, *boundary)?,
    )
    .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)
}

fn internal_boundary_center(
    cylinders: [&super::super::curved_source::CertifiedCylinderSource; 2],
    boundary: InternalTangencyBoundary,
) -> Result<Point3, MixedShellPlanError> {
    cylinders
        .get(boundary.operand)
        .and_then(|source| source.boundaries().get(boundary.boundary))
        .map(|boundary| boundary.center())
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)
}

fn internal_ring_lineage(
    cylinders: [&super::super::curved_source::CertifiedCylinderSource; 2],
    contained_side: RawFaceId,
    contained: usize,
    boundaries: &[InternalTangencyBoundary],
) -> Result<MixedDerivedRingLineage, MixedShellPlanError> {
    if let Some(boundary) = boundaries
        .iter()
        .find(|boundary| boundary.operand == contained)
    {
        return cylinders
            .get(contained)
            .and_then(|source| source.boundaries().get(boundary.boundary))
            .map(|boundary| MixedDerivedRingLineage::Source(boundary.edge()))
            .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    }
    let cutting = boundaries
        .first()
        .and_then(|boundary| {
            cylinders
                .get(boundary.operand)
                .and_then(|source| source.boundaries().get(boundary.boundary))
        })
        .ok_or(MixedShellPlanError::InternalTangencyBoundaryMismatch)?;
    Ok(MixedDerivedRingLineage::Derived([
        contained_side,
        cutting.cap_face(),
    ]))
}

fn ordered_internal_boundaries(
    geometry: InternalTangencyArrangementGeometry<'_>,
    operand: usize,
) -> Result<[InternalTangencyBoundary; 2], MixedShellPlanError> {
    let boundaries = [0, 1].map(|boundary| InternalTangencyBoundary {
        operand,
        boundary,
        axial_parameter: geometry.axial_parameters[operand][boundary],
    });
    match compare_internal_boundaries(geometry, boundaries[0], boundaries[1]) {
        core::cmp::Ordering::Less => Ok(boundaries),
        core::cmp::Ordering::Greater => Ok([boundaries[1], boundaries[0]]),
        core::cmp::Ordering::Equal => Err(MixedShellPlanError::InternalTangencyBoundaryMismatch),
    }
}

fn single_internal_boundary(
    boundaries: Vec<InternalTangencyBoundary>,
) -> Result<InternalTangencyBoundary, MixedShellPlanError> {
    let [boundary] = boundaries.as_slice() else {
        return Err(MixedShellPlanError::InternalTangencyBoundaryMismatch);
    };
    Ok(*boundary)
}

fn compare_internal_boundaries(
    geometry: InternalTangencyArrangementGeometry<'_>,
    first: InternalTangencyBoundary,
    second: InternalTangencyBoundary,
) -> core::cmp::Ordering {
    use super::super::axial_interval_sweep::{
        AuthoredAxialEndpoint, AxialEndpointContributor, AxialIntervalOperand,
    };

    let contributor = |boundary: InternalTangencyBoundary| {
        AxialEndpointContributor::new(
            if boundary.operand == 0 {
                AxialIntervalOperand::Left
            } else {
                AxialIntervalOperand::Right
            },
            if boundary.boundary == 0 {
                AuthoredAxialEndpoint::Start
            } else {
                AuthoredAxialEndpoint::End
            },
        )
    };
    geometry
        .preorder
        .compare(contributor(first), contributor(second))
}

fn same_internal_boundary(
    first: InternalTangencyBoundary,
    second: InternalTangencyBoundary,
) -> bool {
    first.operand == second.operand && first.boundary == second.boundary
}

fn exact_internal_axial_projection(frame: Frame, point: Point3) -> Option<(Point3, f64)> {
    let origin = frame.origin();
    let axis = frame.z();
    let delta = point - origin;
    let axis_components = axis.to_array();
    let delta_components = delta.to_array();
    let candidates = [
        delta.dot(axis),
        delta_components[0] / axis_components[0],
        delta_components[1] / axis_components[1],
        delta_components[2] / axis_components[2],
    ];
    candidates.into_iter().find_map(|parameter| {
        if !parameter.is_finite() {
            return None;
        }
        let center = origin + axis * parameter;
        (axis_parameter_identity_is_exact(center, origin, axis, parameter)
            && affine_dot3(axis.to_array(), center.to_array(), point.to_array(), 0.0)
                .is_some_and(|orientation| orientation.sign() == Orientation::Zero))
        .then_some((center, parameter))
    })
}

fn axis_parameter_identity_is_exact(
    point: Point3,
    origin: Point3,
    axis: kgeom::vec::Vec3,
    parameter: f64,
) -> bool {
    let point = point.to_array();
    let origin = origin.to_array();
    let axis = axis.to_array();
    (0..3).all(|component| {
        affine_dot3(
            [1.0, axis[component], -1.0],
            [origin[component], parameter, point[component]],
            [0.0; 3],
            0.0,
        )
        .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

pub(crate) fn arrange_common_support_spans_mixed_shell<'a>(
    store: &Store,
    graph: &BodySectionGraph,
    bindings: impl IntoIterator<Item = MixedArrangementBinding<'a>>,
    selected: impl IntoIterator<Item = SelectedBoundaryFragment<MixedShellCellKey, ()>>,
    interval: &super::super::axial_interval_sweep::AxialIntervalPlan,
    preorder: &super::super::axial_interval_sweep::CertifiedAxialEndpointPreorder,
    tolerance: f64,
) -> Result<MixedShellArrangement<'a>, MixedShellPlanError> {
    use core::cmp::Ordering;

    use super::super::axial_interval_sweep::{
        AuthoredAxialEndpoint, AxialEndpointContributor, AxialIntervalOperand,
    };

    let fail = || MixedShellPlanError::CommonSupportBoundaryMismatch;
    let mut arrangement = arrange_selected_mixed_shell(
        store,
        graph,
        bindings,
        selected.into_iter().map(selected_cell),
        false,
    )?;
    let faces = &mut arrangement.faces;
    let rings = &mut arrangement.cap_rings;
    let source_faces = faces.clone();
    let source_rings = rings.clone();
    let split_operand = match interval.spans() {
        [first, second] => {
            let first = sole_interval_operand(first.side_operands());
            (first.is_some() && first == sole_interval_operand(second.side_operands()))
                .then_some(first)
                .flatten()
        }
        _ => None,
    };
    faces.clear();
    rings.clear();
    for (span_index, span) in interval.spans().iter().enumerate() {
        let target_operand = if span.side_operands().contains(AxialIntervalOperand::Left) {
            0
        } else if span.side_operands().contains(AxialIntervalOperand::Right) {
            1
        } else {
            return Err(fail());
        };
        let target_ring = source_rings
            .iter()
            .find(|ring| ring.operand() == target_operand)
            .ok_or_else(fail)?;
        let target = source_faces
            .iter()
            .find(|face| {
                face.source == target_ring.side_source()
                    && face.source_face == *target_ring.side_face()
            })
            .ok_or_else(fail)?;
        let mut boundary_faces = Vec::with_capacity(2);
        let mut boundary_loops = Vec::with_capacity(2);
        for (output_end, contributors) in [span.low(), span.high()].into_iter().enumerate() {
            let primary_contributor = contributors.iter().next().ok_or_else(fail)?;
            let mut endpoint_rings = contributors
                .iter()
                .map(|contributor| {
                    let operand = match contributor.operand() {
                        AxialIntervalOperand::Left => 0,
                        AxialIntervalOperand::Right => 1,
                    };
                    let boundary = match contributor.endpoint() {
                        AuthoredAxialEndpoint::Start => 0,
                        AuthoredAxialEndpoint::End => 1,
                    };
                    source_rings
                        .iter()
                        .find(|ring| ring.operand() == operand && ring.boundary() == boundary)
                        .cloned()
                        .ok_or_else(fail)
                })
                .collect::<Result<Vec<_>, _>>()?;
            let [primary, rest @ ..] = endpoint_rings.as_mut_slice() else {
                return Err(fail());
            };
            if rest.len() > 1
                || rest
                    .first()
                    .is_some_and(|peer| peer.edge() == primary.edge())
            {
                return Err(fail());
            }
            let owner = source_faces
                .iter()
                .find(|face| {
                    face.source == primary.side_source() && face.source_face == *primary.side_face()
                })
                .ok_or_else(fail)?;
            let matching = owner
                .loops
                .iter()
                .filter(|loop_| {
                    loop_.uses.len() == 1
                        && loop_.uses[0].edge
                            == (MixedShellEdgeKey::PeriodicSource {
                                source: primary.side_source(),
                                loop_key: primary.side_loop_key(),
                            })
                })
                .cloned()
                .collect::<Vec<_>>();
            let [mut loop_] = matching.try_into().map_err(|_| fail())?;
            if primary.side_source() != target.source {
                let proof = ProjectedEndpointFreeSourceCircle::certify(
                    store,
                    primary,
                    target.source,
                    &target.source_face,
                    tolerance,
                )
                .map_err(MixedShellPlanError::ProjectedSourceCircle)?;
                loop_.uses[0].pcurve = MixedPcurveLineage::ProjectedEndpointFreeSourceCircle(proof);
            }
            let mut cap = source_faces
                .iter()
                .find(|face| {
                    face.source == primary.cap_source() && face.source_face == *primary.cap_face()
                })
                .cloned()
                .ok_or_else(fail)?;
            let other = AxialEndpointContributor::new(
                primary_contributor.operand(),
                match primary_contributor.endpoint() {
                    AuthoredAxialEndpoint::Start => AuthoredAxialEndpoint::End,
                    AuthoredAxialEndpoint::End => AuthoredAxialEndpoint::Start,
                },
            );
            let source_low = match preorder.compare(primary_contributor, other) {
                Ordering::Less => true,
                Ordering::Greater => false,
                Ordering::Equal => return Err(fail()),
            };
            if source_low != (output_end == 0) {
                cap.selected_orientation = match cap.selected_orientation {
                    SelectedOrientation::Preserved => SelectedOrientation::Reversed,
                    SelectedOrientation::Reversed => SelectedOrientation::Preserved,
                };
                loop_.uses[0].direction = opposite(loop_.uses[0].direction);
            }
            if let [peer] = rest {
                *primary = primary.clone().with_merge_edge_source(peer.edge());
                cap.merge_sources = Some([primary.cap_face().clone(), peer.cap_face().clone()]);
            }
            boundary_loops.push(loop_);
            boundary_faces.push(cap);
            rings.push(primary.clone());
        }
        let both = span.side_operands().contains(AxialIntervalOperand::Left)
            && span.side_operands().contains(AxialIntervalOperand::Right);
        let source_side_faces = [0, 1]
            .map(|operand| {
                source_rings
                    .iter()
                    .find(|ring| ring.operand() == operand)
                    .map(|ring| ring.side_face().clone())
                    .ok_or_else(fail)
            })
            .into_iter()
            .collect::<Result<Vec<_>, _>>()?;
        let [first_side_face, second_side_face] =
            source_side_faces.try_into().map_err(|_| fail())?;
        faces.push(MixedShellFacePlan {
            source: target.source,
            source_face: target.source_face.clone(),
            selected_orientation: target.selected_orientation,
            loops: boundary_loops,
            merge_sources: both.then_some([first_side_face, second_side_face]),
            split_lineage: (split_operand
                == Some(if target_operand == 0 {
                    AxialIntervalOperand::Left
                } else {
                    AxialIntervalOperand::Right
                }))
            .then_some(if span_index == 0 {
                AnalyticFaceSplitPiece::First
            } else {
                AnalyticFaceSplitPiece::Second
            }),
        });
        faces.extend(boundary_faces);
    }
    Ok(arrangement)
}

fn sole_interval_operand(
    operands: super::super::axial_interval_sweep::AxialOperandContributors,
) -> Option<super::super::axial_interval_sweep::AxialIntervalOperand> {
    let mut operands = operands.iter();
    let first = operands.next()?;
    operands.next().is_none().then_some(first)
}
