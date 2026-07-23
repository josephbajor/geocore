//! Section publication for graph-certified Cylinder/Cylinder branches.
//!
//! Rulings retain topology-owned clipping and root identity. A certified skew
//! sheet publishes directly as one whole closed fragment only after graph
//! source-window containment and dispatcher-owned whole-band topology proof;
//! no unsupported nonlinear clipping result is invented.
//!
//! Orientation uses metric projections against the stored cylinder axes. A
//! [`Frame`] axis is semantically unit length but its stored components are
//! rounded, so the radial theorem retains the outward `axis · axis`
//! denominator instead of silently replacing it by one. Carrier and normal
//! magnitudes remain irrelevant: only a strict interval sign is published.

use kcore::predicates::{Orientation, orient3d};
use ktopo::geom::Curve2dGeom;

use super::*;

pub(super) enum CylinderCylinderBranchAdaptation {
    Adapted(Box<SectionBranch>),
    OrientationIndeterminate,
    Unsupported,
}

/// Adapt and topology-clip one graph-certified Cylinder/Cylinder ruling.
#[allow(clippy::too_many_arguments)]
pub(super) fn append_branch(
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
    let branch = match adapt_branch(facades, edge, vertices, surfaces, senses) {
        CylinderCylinderBranchAdaptation::Adapted(branch) => *branch,
        CylinderCylinderBranchAdaptation::OrientationIndeterminate => {
            acc.gaps.push(SectionGap {
                reason: GAP_CARRIER_ORIENTATION,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
        CylinderCylinderBranchAdaptation::Unsupported => {
            acc.gaps.push(SectionGap {
                reason: GAP_PAIR_UNRESOLVED,
                faces: facades.to_vec(),
            });
            return Ok(());
        }
    };
    if matches!(branch.carrier, SectionCarrier::SkewCylinderBranch(_)) {
        if !append_whole_closed_branch(branch, acc) {
            acc.gaps.push(SectionGap {
                reason: GAP_CLOSED_STITCH,
                faces: facades.to_vec(),
            });
        }
        return Ok(());
    }
    let [
        SectionUvCurve::Line(first_trace),
        SectionUvCurve::Line(second_trace),
    ] = branch.pcurves
    else {
        acc.gaps.push(SectionGap {
            reason: GAP_PAIR_UNRESOLVED,
            faces: facades.to_vec(),
        });
        return Ok(());
    };
    ruling_publish::append_branch_with_endpoint_proof(
        store,
        raw_faces,
        facades,
        branch,
        |left, right, scope| {
            certify_coincident_cap_ring_endpoints(
                store,
                raw_faces,
                [first_trace, second_trace],
                left,
                right,
                scope,
            )
        },
        root_identity,
        scope,
        acc,
    )
}

/// Publish one source-window-contained sheet as a whole closed fragment.
///
/// Source adaptation is completed before any accumulator mutation so branch,
/// stitch input, and facade evidence remain index-aligned.
fn append_whole_closed_branch(branch: SectionBranch, acc: &mut SectionAccumulator) -> bool {
    let branch_index = acc.branches.len();
    let Some(source) =
        closed_stitch::ClosedBranchSource::from_section_branch(branch_index, &branch)
    else {
        return false;
    };
    let fragment = closed_stitch::ClosedCurveFragment {
        source: source.fragment(0),
        orientation: closed_stitch::ClosedFragmentOrientation::AlongCarrier,
        span: closed_stitch::ClosedFragmentSpan::Whole,
    };
    let evidence = ClosedFragmentEvidence {
        branch: branch_index,
        ordinal: 0,
        span: ClosedFragmentEvidenceSpan::Whole,
    };
    acc.branches.push(branch);
    acc.closed_fragments.push(fragment);
    acc.closed_fragment_evidence.push(evidence);
    true
}

/// Prove cap-ring pairs with one exact root in the graph-certified semantic
/// carrier parameterization.
///
/// Graph promotion has already bound both pcurves to one paired whole-range
/// residual certificate. This stage adds topology-owned horizontal ring
/// coefficients and uses an exact determinant; it never infers identity from
/// overlapping carrier intervals or nearby model-space points.
fn certify_coincident_cap_ring_endpoints(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    traces: [SectionUvLine; 2],
    left: &[ruling_clip::RulingClipSpan],
    right: &[ruling_clip::RulingClipSpan],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<ruling_publish::RulingEndpointCoincidenceProof>> {
    let candidate_pairs = (left.len() as u64)
        .saturating_mul(right.len() as u64)
        .saturating_mul(4);
    scope
        .ledger_mut()
        .charge(SECTION_WORK, candidate_pairs)
        .map_err(Error::from)?;
    let mut pairs = Vec::with_capacity(2);
    for left_site in left.iter().flat_map(|span| [span.start, span.end]) {
        for right_site in right.iter().flat_map(|span| [span.start, span.end]) {
            if cap_ring_sites_are_coincident(store, raw_faces, traces, left_site, right_site)? {
                let pair = [left_site.edge, right_site.edge];
                if !pairs.contains(&pair) {
                    pairs.push(pair);
                }
            }
        }
    }
    Ok(ruling_publish::RulingEndpointCoincidenceProof::from_exact_cap_ring_pairs(&pairs))
}

fn cap_ring_sites_are_coincident(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    traces: [SectionUvLine; 2],
    left: ruling_clip::RulingTrimSite,
    right: ruling_clip::RulingTrimSite,
) -> Result<bool> {
    if [left.face, right.face] != raw_faces {
        return Ok(false);
    }
    let (Some(left_height), Some(right_height)) =
        (ring_height(store, left)?, ring_height(store, right)?)
    else {
        return Ok(false);
    };
    let [left_trace, right_trace] = traces;
    let coefficients = [
        left_trace.direction().y,
        left_trace.origin().y,
        left_height,
        right_trace.direction().y,
        right_trace.origin().y,
        right_height,
    ];
    Ok(coefficients.into_iter().all(f64::is_finite)
        && left_trace.direction().y != 0.0
        && right_trace.direction().y != 0.0
        // The roots `(h - origin) / direction` are equal exactly iff this
        // determinant is zero; orient3d evaluates it without rounded
        // intermediate subtractions.
        && orient3d(
            [left_trace.direction().y, left_trace.origin().y, left_height],
            [
                right_trace.direction().y,
                right_trace.origin().y,
                right_height,
            ],
            [0.0, 1.0, 1.0],
            [0.0; 3],
        ) == Orientation::Zero)
}

fn ring_height(store: &Store, site: ruling_clip::RulingTrimSite) -> Result<Option<f64>> {
    let loop_ = store
        .get(site.loop_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let fin = store
        .get(site.fin)
        .map_err(|source| Error::InconsistentTopology { source })?;
    if loop_.face() != site.face || fin.parent() != site.loop_id || fin.edge() != site.edge {
        return Ok(None);
    }
    let Some(use_) = fin.pcurve() else {
        return Ok(None);
    };
    Ok(
        match store
            .pcurve(use_.curve())
            .map_err(|source| Error::InconsistentTopology { source })?
        {
            Curve2dGeom::Line(boundary) if boundary.dir().y == 0.0 => Some(boundary.origin().y),
            _ => None,
        },
    )
}

fn adapt_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
) -> CylinderCylinderBranchAdaptation {
    if edge.kind != ContactKind::Transverse {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    if let Some(certificate) = edge.certificate.as_skew_cylinder_two_sheet() {
        return adapt_skew_two_sheet_branch(faces, edge, vertices, surfaces, senses, certificate);
    }
    let Some(certificate) = edge.certificate.as_cylinder_cylinder_ruling() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let CurveDescriptor::Line(carrier) = edge.carrier else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    if edge.topology != IntersectionBranchTopology::Open
        || edge.endpoint_vertices[0] == edge.endpoint_vertices[1]
    {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(flipped) = canonical_flip(
        surfaces[0],
        senses[0],
        surfaces[1],
        senses[1],
        carrier.origin(),
        carrier.dir(),
    ) else {
        return CylinderCylinderBranchAdaptation::OrientationIndeterminate;
    };
    let Some(pcurves) = branch_publish::adapt_branch_pcurves(edge, flipped) else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    if !matches!(pcurves, [SectionUvCurve::Line(_), SectionUvCurve::Line(_)]) {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(low_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(flipped)])
        .copied()
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let Some(high_vertex) = vertices
        .get(edge.endpoint_vertices[usize::from(!flipped)])
        .copied()
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: low_surfaces,
    } = low_vertex.event
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::BoundaryEndpoint {
        surfaces: high_surfaces,
    } = high_vertex.event
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let range = if flipped {
        ParamRange {
            lo: -edge.carrier_range.hi,
            hi: -edge.carrier_range.lo,
        }
    } else {
        edge.carrier_range
    };
    CylinderCylinderBranchAdaptation::Adapted(Box::new(SectionBranch {
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
        ruling_recertification: Some(RulingRecertification::CylinderCylinderGraph(certificate)),
        ruling_parameter_flipped: flipped,
    }))
}

pub(super) fn adapt_skew_two_sheet_branch(
    faces: &[FaceId; 2],
    edge: &IntersectionBranchEdge,
    vertices: &[kops::intersect::IntersectionBranchVertex],
    surfaces: [&SurfaceGeom; 2],
    senses: [Sense; 2],
    certificate: kgraph::PairedSkewCylinderBranchResidualCertificate,
) -> CylinderCylinderBranchAdaptation {
    let Some(carrier) = edge.carrier.as_skew_cylinder_branch().copied() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let Some(first_pcurve) = edge.pcurves[0].as_skew_cylinder_branch().copied() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let Some(second_pcurve) = edge.pcurves[1].as_skew_cylinder_branch().copied() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let source_pcurves = [first_pcurve, second_pcurve];
    let traces = certificate.traces();
    if edge.topology != IntersectionBranchTopology::Closed
        || edge.endpoint_vertices[0] != edge.endpoint_vertices[1]
        || edge.carrier_range != certificate.carrier_range()
        || carrier != certificate.carrier()
        || source_pcurves != traces.map(|trace| trace.pcurve())
        || edge.parameter_maps != certificate.parameter_maps()
    {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    let Some(vertex) = vertices.get(edge.endpoint_vertices[0]).copied() else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let IntersectionBranchVertexEvent::PeriodSeam {
        surfaces: seam_boundaries,
    } = vertex.event
    else {
        return CylinderCylinderBranchAdaptation::Unsupported;
    };
    let range = edge.carrier_range;
    let graph_carrier = SectionSkewCylinderBranchCarrier::new(carrier, range, false);
    let graph_seam = graph_carrier.eval_derivs(range.lo, 1);
    let Some(flipped) = finite_point_and_tangent(graph_seam.d[0], graph_seam.d[1])
        .then(|| {
            canonical_flip(
                surfaces[0],
                senses[0],
                surfaces[1],
                senses[1],
                graph_seam.d[0],
                graph_seam.d[1],
            )
        })
        .flatten()
    else {
        return CylinderCylinderBranchAdaptation::OrientationIndeterminate;
    };

    let carrier = SectionSkewCylinderBranchCarrier::new(carrier, range, flipped);
    let pcurves =
        source_pcurves.map(|pcurve| SectionSkewCylinderBranchPcurve::new(pcurve, range, flipped));
    let seam = carrier.eval_derivs(range.lo, 1);
    if !finite_point_and_tangent(seam.d[0], seam.d[1])
        || canonical_flip(
            surfaces[0],
            senses[0],
            surfaces[1],
            senses[1],
            seam.d[0],
            seam.d[1],
        ) != Some(false)
    {
        return CylinderCylinderBranchAdaptation::OrientationIndeterminate;
    }
    let seam_uv = pcurves.map(|pcurve| pcurve.eval(range.lo));
    if !seam_uv
        .iter()
        .flat_map(|uv| [uv.x, uv.y])
        .all(f64::is_finite)
    {
        return CylinderCylinderBranchAdaptation::Unsupported;
    }
    CylinderCylinderBranchAdaptation::Adapted(Box::new(SectionBranch {
        faces: faces.clone(),
        carrier: SectionCarrier::SkewCylinderBranch(carrier),
        range,
        topology: SectionBranchTopology::Closed,
        pcurves: pcurves.map(SectionUvCurve::SkewCylinderBranch),
        fragment_sites: vec![SectionFragmentSite {
            point: seam.d[0],
            surface_parameters: seam_uv.map(|uv| [uv.x, uv.y]),
            surface_window_boundaries: seam_boundaries,
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

fn finite_point_and_tangent(point: Point3, tangent: Vec3) -> bool {
    point
        .to_array()
        .into_iter()
        .chain(tangent.to_array())
        .all(f64::is_finite)
}

/// Decide whether a graph carrier must be reversed to follow Section's
/// `n_A x n_B` convention.
///
/// Both cylinder normals are evaluated as outward-rounded, unnormalized
/// radial vectors at the carrier origin. A strict sign is required; tangent
/// or arithmetically ambiguous candidates remain structured gaps.
///
/// For a skew sheet, the paired certificate's strict-positive radicand and
/// transverse closed-sheet invariant make this continuous orientation sign
/// nonzero over the complete cycle. Its sign is therefore constant, so one
/// strictly certified seam sign orients the whole loop.
pub(super) fn canonical_flip(
    surface_a: &SurfaceGeom,
    sense_a: Sense,
    surface_b: &SurfaceGeom,
    sense_b: Sense,
    origin: Point3,
    direction: Vec3,
) -> Option<bool> {
    let (SurfaceGeom::Cylinder(cylinder_a), SurfaceGeom::Cylinder(cylinder_b)) =
        (surface_a, surface_b)
    else {
        return None;
    };
    let normal_a = cylinder_normal(cylinder_a, sense_a, origin)?;
    let normal_b = cylinder_normal(cylinder_b, sense_b, origin)?;
    certified_carrier_sign_intervals(
        direction.to_array().map(Interval::point),
        normal_a,
        normal_b,
    )
    .map(|positive| !positive)
}

fn cylinder_normal(
    cylinder: &kgeom::surface::Cylinder,
    sense: Sense,
    point: Point3,
) -> Option<[Interval; 3]> {
    let axis = cylinder.frame().z().to_array().map(Interval::point);
    let axis_origin = cylinder.frame().origin().to_array();
    let point = point.to_array();
    let offset: [Interval; 3] = core::array::from_fn(|index| {
        Interval::point(point[index]) - Interval::point(axis_origin[index])
    });
    metric_radial(offset, axis, sense)
}

/// Outward metric projection onto the plane perpendicular to `axis`.
///
/// The division is essential for a general stored vector. In the certified
/// parallel-ruling family, omitting it adds only an axis-parallel component
/// whose exact contribution to `direction · (n_a × n_b)` vanishes. Retaining
/// it here removes that family-specific dependency and avoids needlessly wide
/// normal enclosures for rounded nonunit frame axes.
fn metric_radial(
    offset: [Interval; 3],
    axis: [Interval; 3],
    sense: Sense,
) -> Option<[Interval; 3]> {
    let axial_numerator = interval_dot(offset, axis);
    let axis_squared = interval_dot(axis, axis);
    let axial = axial_numerator.checked_div(axis_squared)?;
    if !finite(axial) {
        return None;
    }
    let sign = Interval::point(if sense.is_forward() { 1.0 } else { -1.0 });
    let radial = core::array::from_fn(|index| sign * (offset[index] - axis[index] * axial));
    radial.into_iter().all(finite).then_some(radial)
}

fn interval_dot(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    (0..3).fold(Interval::point(0.0), |sum, index| sum + a[index] * b[index])
}

fn finite(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

#[cfg(test)]
mod tests {
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, OperationReport, OperationScope,
        ResourceKind, SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::surface::Cylinder;

    use super::*;
    use crate::{
        BodyId, CylinderRequest, Kernel, OperationOutcome, OperationSettings, PartId,
        SectionBodiesRequest, SectionCompletion, Session,
    };

    #[derive(Debug, Clone, Copy)]
    enum Placement {
        World,
        Oblique,
    }

    struct PublicFixture {
        session: Session,
        part: PartId,
        first: BodyId,
        second: BodyId,
    }

    fn placement_frame(placement: Placement) -> Frame {
        match placement {
            Placement::World => Frame::world(),
            Placement::Oblique => Frame::new(
                Point3::new(2.5, -1.75, 0.625),
                Vec3::new(0.48, 0.64, 0.6),
                Vec3::new(0.8, -0.6, 0.0),
            )
            .unwrap(),
        }
    }

    fn public_fixture(
        placement: Placement,
        first: (f64, f64),
        second: (f64, f64),
        reverse_second_axis: bool,
    ) -> PublicFixture {
        public_fixture_geometry(
            placement,
            first,
            second,
            reverse_second_axis,
            1.0,
            [1.0, 1.0],
        )
    }

    fn public_fixture_geometry(
        placement: Placement,
        first: (f64, f64),
        second: (f64, f64),
        reverse_second_axis: bool,
        radial_offset: f64,
        radii: [f64; 2],
    ) -> PublicFixture {
        let frame = placement_frame(placement);
        let first_frame = frame.with_origin(frame.point_at(-0.5 * radial_offset, 0.0, first.0));
        let second_frame = if reverse_second_axis {
            Frame::new(
                frame.point_at(0.5 * radial_offset, 0.0, second.0 + second.1),
                -frame.z(),
                frame.x(),
            )
            .unwrap()
        } else {
            frame.with_origin(frame.point_at(0.5 * radial_offset, 0.0, second.0))
        };
        let mut session = Kernel::new().create_session();
        let part = session.create_part();
        let (first, second) = {
            let mut edit = session.edit_part(part.clone()).unwrap();
            let first = edit
                .create_cylinder(CylinderRequest::new(first_frame, radii[0], first.1))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            let second = edit
                .create_cylinder(CylinderRequest::new(second_frame, radii[1], second.1))
                .unwrap()
                .into_result()
                .unwrap()
                .body();
            (first, second)
        };
        PublicFixture {
            session,
            part,
            first,
            second,
        }
    }

    fn section(fixture: &PublicFixture, swapped: bool) -> (BodySectionGraph, OperationReport) {
        let (first, second) = if swapped {
            (fixture.second.clone(), fixture.first.clone())
        } else {
            (fixture.first.clone(), fixture.second.clone())
        };
        let outcome = fixture
            .session
            .part(fixture.part.clone())
            .unwrap()
            .section_bodies(SectionBodiesRequest::new(first, second))
            .unwrap();
        let (result, report) = outcome.into_parts();
        (result.unwrap(), report)
    }

    fn section_with_work_limit(
        fixture: &PublicFixture,
        allowed: u64,
    ) -> OperationOutcome<BodySectionGraph> {
        let budget = BudgetPlan::new([LimitSpec::new(
            SECTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        fixture
            .session
            .part(fixture.part.clone())
            .unwrap()
            .section_bodies(
                SectionBodiesRequest::new(fixture.first.clone(), fixture.second.clone())
                    .with_settings(OperationSettings::new().with_budget_overrides(budget)),
            )
            .unwrap()
    }

    fn section_work(report: &OperationReport) -> u64 {
        report
            .usage()
            .iter()
            .find(|snapshot| {
                snapshot.stage == SECTION_WORK && snapshot.resource == ResourceKind::Work
            })
            .expect("section query must retain its exact work usage")
            .consumed
    }

    fn assert_all_cap_side_pairs_are_coincident(fixture: &PublicFixture, graph: &BodySectionGraph) {
        let part = fixture.session.part(fixture.part.clone()).unwrap();
        let faces = |body: &BodyId| {
            let mut side = None;
            let mut caps = Vec::new();
            for face in part.state.store.faces_of_body(body.raw()).unwrap() {
                match part
                    .state
                    .store
                    .surface(part.state.store.get(face).unwrap().surface)
                    .unwrap()
                {
                    SurfaceGeom::Cylinder(_) => assert!(side.replace(face).is_none()),
                    SurfaceGeom::Plane(_) => caps.push(face),
                    _ => panic!("cylinder fixture retained an unexpected surface"),
                }
            }
            (side.unwrap(), caps)
        };
        let (first_side, first_caps) = faces(&fixture.first);
        let (second_side, second_caps) = faces(&fixture.second);
        assert_eq!(first_caps.len(), 2);
        assert_eq!(second_caps.len(), 2);
        let expected = first_caps
            .into_iter()
            .map(|cap| [cap, second_side])
            .chain(second_caps.into_iter().map(|cap| [first_side, cap]))
            .collect::<Vec<_>>();
        let mut actual = graph
            .gaps()
            .iter()
            .filter(|gap| {
                gap.reason() == curved_clip::ClosedConicClipGap::CoincidentBoundary.reason()
            })
            .map(|gap| gap.faces().iter().map(FaceId::raw).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        for pair in expected {
            let index = actual
                .iter()
                .position(|candidate| {
                    candidate.len() == 2
                        && (candidate[0] == pair[0] && candidate[1] == pair[1]
                            || candidate[0] == pair[1] && candidate[1] == pair[0])
                })
                .expect("every cap/other-side pair must retain one coincidence gap");
            actual.remove(index);
        }
        assert!(actual.is_empty());
    }

    fn assert_dual_source_rulings(
        graph: &BodySectionGraph,
        expected_span_counts: [usize; 3],
        expected_dual_ends: usize,
        expected_conic_gaps: usize,
        expected_coincident_face_gaps: usize,
    ) {
        assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
        assert_eq!(
            graph.curve_fragments().len(),
            expected_span_counts.into_iter().sum::<usize>()
        );
        assert_eq!(graph.curve_endpoints().len(), 4);
        assert!(graph.curve_components().is_empty());
        let [first_embedding, second_embedding] = graph.periodic_face_embeddings() else {
            panic!("both source cylinders require periodic embedding evidence")
        };
        let mut embedding_operands = [usize::MAX; 2];
        let mut embedding_faces = Vec::with_capacity(2);
        for (ordinal, embedding) in [first_embedding, second_embedding].into_iter().enumerate() {
            let SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
                operand,
                face,
                gap: SectionPeriodicEmbeddingGap::UnstitchedFragmentPath { fragment },
            } = embedding
            else {
                panic!(
                    "visible degree-3 cap/ruling incidence must retain the exact unstitched-path gap: {embedding:#?}"
                )
            };
            let source = graph
                .curve_fragments()
                .get(*fragment)
                .and_then(|fragment| graph.branches().get(fragment.branch()))
                .expect("periodic gap must name a live source fragment");
            assert_eq!(&source.faces()[*operand], face);
            embedding_operands[ordinal] = *operand;
            embedding_faces.push(face.clone());
        }
        embedding_operands.sort_unstable();
        assert_eq!(embedding_operands, [0, 1]);
        assert_ne!(embedding_faces[0], embedding_faces[1]);
        assert_gap_counts(graph, expected_conic_gaps, expected_coincident_face_gaps);
        let mut span_counts = [0; 3];
        let mut dual_arc_count = 0;
        let mut ruling_count = 0;
        let mut dual_ends = 0;
        for fragment in graph.curve_fragments() {
            let endpoints = match fragment.span() {
                SectionCurveFragmentSpan::Whole => {
                    span_counts[0] += 1;
                    continue;
                }
                SectionCurveFragmentSpan::Arc { endpoints, .. } => {
                    span_counts[1] += 1;
                    let mut dual = true;
                    for end in endpoints.iter() {
                        let SectionCurveEndpointTopology::Trim {
                            sites,
                            source_parameters,
                        } = graph.curve_endpoints()[end.endpoint()].topology()
                        else {
                            panic!("coincident ring arc lost physical root topology")
                        };
                        let present = sites
                            .iter()
                            .zip(source_parameters)
                            .filter(|(site, root)| {
                                matches!(site, SectionSite::EdgeInterior(edge) if root.as_ref().is_some_and(|root| root.edge() == edge.clone()))
                            })
                            .count();
                        assert!(matches!(present, 1 | 2));
                        dual &= present == 2;
                    }
                    dual_arc_count += usize::from(dual);
                    continue;
                }
                SectionCurveFragmentSpan::LineSegment { endpoints } => {
                    span_counts[2] += 1;
                    endpoints
                }
                SectionCurveFragmentSpan::BoundedProcedural { .. } => {
                    panic!(
                        "parallel-cylinder fixture unexpectedly published a bounded procedural fragment"
                    )
                }
            };
            ruling_count += 1;
            let branch = &graph.branches()[fragment.branch()];
            for end in endpoints.iter() {
                let present = end.trims().iter().filter(|trim| trim.is_some()).count();
                dual_ends += usize::from(present == 2);
                assert!(matches!(present, 1 | 2));
                let public = &graph.curve_endpoints()[end.endpoint()];
                let SectionCurveEndpointTopology::Trim {
                    sites,
                    source_parameters,
                } = public.topology()
                else {
                    panic!("ruling endpoint must retain physical trim topology")
                };
                for operand in 0..2 {
                    match &end.trims()[operand] {
                        Some(trim) => {
                            assert_eq!(trim.operand(), operand);
                            assert_eq!(trim.face(), branch.faces()[operand]);
                            assert_eq!(
                                sites[operand],
                                SectionSite::EdgeInterior(trim.source_parameter().edge())
                            );
                            assert_eq!(
                                source_parameters[operand].as_ref(),
                                Some(trim.source_parameter())
                            );
                            assert!(public.edge_parameters()[operand].is_some());
                        }
                        None => {
                            assert_eq!(
                                sites[operand],
                                SectionSite::FaceInterior(branch.faces()[operand].clone())
                            );
                            assert!(source_parameters[operand].is_none());
                            assert!(public.edge_parameters()[operand].is_none());
                        }
                    }
                }
                if present == 2 {
                    let [Some(first), Some(second)] = source_parameters else {
                        unreachable!()
                    };
                    assert_ne!(first.edge(), second.edge());
                }
            }
        }
        assert_eq!(span_counts, expected_span_counts);
        assert_eq!(dual_arc_count, expected_conic_gaps);
        assert_eq!(ruling_count, 2);
        assert_eq!(dual_ends, expected_dual_ends);
    }

    fn assert_gap_counts(
        graph: &BodySectionGraph,
        expected_conic_gaps: usize,
        expected_coincident_face_gaps: usize,
    ) {
        let gap_count = |reason| {
            graph
                .gaps()
                .iter()
                .filter(|gap| gap.reason() == reason)
                .count()
        };
        assert_eq!(
            gap_count(curved_clip::ClosedConicClipGap::CoincidentBoundary.reason()),
            expected_conic_gaps
        );
        assert_eq!(
            gap_count(GAP_COINCIDENT_FACE_PAIR),
            expected_coincident_face_gaps
        );
        assert_eq!(gap_count(GAP_MIXED_FRAGMENT_STITCH), 1);
        assert_eq!(
            graph.gaps().len(),
            expected_conic_gaps + expected_coincident_face_gaps + 1
        );
        assert_eq!(
            gap_count(ruling_clip::RulingClipGap::UnorderedCrossings.reason()),
            0
        );
    }

    fn point_intervals(values: [f64; 3]) -> [Interval; 3] {
        values.map(Interval::point)
    }

    #[test]
    fn metric_radial_divides_by_the_outward_axis_gram_term() {
        let radial = metric_radial(
            point_intervals([3.0, 4.0, 5.0]),
            point_intervals([2.0, 0.0, 0.0]),
            Sense::Forward,
        )
        .unwrap();
        assert!(radial[0].contains(0.0));
        assert!(radial[1].contains(4.0));
        assert!(radial[2].contains(5.0));
        assert!(interval_dot(radial, point_intervals([2.0, 0.0, 0.0])).contains_zero());
        assert!(
            metric_radial(
                point_intervals([1.0, 2.0, 3.0]),
                point_intervals([0.0, 0.0, 0.0]),
                Sense::Forward,
            )
            .is_none()
        );
    }

    #[test]
    fn canonical_orientation_reverses_under_operand_swap_and_one_face_reversal() {
        let first = SurfaceGeom::Cylinder(Cylinder::new(Frame::world(), 1.0).unwrap());
        let second = SurfaceGeom::Cylinder(
            Cylinder::new(Frame::world().with_origin(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap(),
        );
        let origin = Point3::new(0.5, 3.0_f64.sqrt() * 0.5, 0.0);
        let direction = Vec3::new(0.0, 0.0, 1.0);
        let forward = canonical_flip(
            &first,
            Sense::Forward,
            &second,
            Sense::Forward,
            origin,
            direction,
        )
        .unwrap();
        assert_eq!(
            canonical_flip(
                &second,
                Sense::Forward,
                &first,
                Sense::Forward,
                origin,
                direction,
            ),
            Some(!forward)
        );
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Reversed,
                &second,
                Sense::Forward,
                origin,
                direction,
            ),
            Some(!forward)
        );
    }

    #[test]
    fn rounded_oblique_axes_preserve_orientation_under_axial_translation_and_swap() {
        let frame = Frame::new(
            Point3::new(2.0, -3.0, 5.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(1.0, -1.0, 0.0),
        )
        .unwrap();
        assert_ne!(
            frame.z().dot(frame.z()),
            1.0,
            "fixture must exercise a rounded nonunit stored axis"
        );
        let second_frame = frame.with_origin(frame.point_at(1.0, 0.0, 0.0));
        let first = SurfaceGeom::Cylinder(Cylinder::new(frame, 1.0).unwrap());
        let second = SurfaceGeom::Cylinder(Cylinder::new(second_frame, 1.0).unwrap());
        let local_y = 3.0_f64.sqrt() * 0.5;
        let near = frame.point_at(0.5, local_y, 0.0);
        let translated = frame.point_at(0.5, local_y, 1.0e6);
        let direction = frame.z();

        let expected = canonical_flip(
            &first,
            Sense::Forward,
            &second,
            Sense::Forward,
            near,
            direction,
        )
        .unwrap();
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Forward,
                &second,
                Sense::Forward,
                translated,
                direction,
            ),
            Some(expected)
        );
        assert_eq!(
            canonical_flip(
                &second,
                Sense::Forward,
                &first,
                Sense::Forward,
                translated,
                direction,
            ),
            Some(!expected)
        );
        assert_eq!(
            canonical_flip(
                &first,
                Sense::Forward,
                &second,
                Sense::Forward,
                translated,
                direction * 7.0,
            ),
            Some(expected)
        );
    }

    #[test]
    fn equal_interval_rulings_publish_dual_sources_for_world_exact_oblique_and_axis_parity() {
        for placement in [Placement::World, Placement::Oblique] {
            for reverse_second_axis in [false, true] {
                for axial_origin in [-1.0, 0.0, 1.0] {
                    let fixture = public_fixture(
                        placement,
                        (axial_origin, 2.0),
                        (axial_origin, 2.0),
                        reverse_second_axis,
                    );
                    let (forward, forward_report) = section(&fixture, false);
                    let (replay, replay_report) = section(&fixture, false);
                    let (swapped, swapped_report) = section(&fixture, true);
                    assert_eq!(forward, replay);
                    assert_eq!(forward_report, replay_report);
                    assert_eq!(forward_report.usage(), swapped_report.usage());
                    assert_dual_source_rulings(&forward, [0, 4, 2], 4, 4, 2);
                    assert_dual_source_rulings(&swapped, [0, 4, 2], 4, 4, 2);
                    assert_all_cap_side_pairs_are_coincident(&fixture, &forward);
                    assert_all_cap_side_pairs_are_coincident(&fixture, &swapped);
                }
            }
        }
    }

    #[test]
    fn one_shared_end_rulings_retain_only_the_exact_dual_source_end() {
        for placement in [Placement::World, Placement::Oblique] {
            for reverse_second_axis in [false, true] {
                for (first, second) in [((0.0, 3.0), (0.0, 2.0)), ((-2.0, 4.0), (-1.0, 3.0))] {
                    let fixture = public_fixture(placement, first, second, reverse_second_axis);
                    let (forward, forward_report) = section(&fixture, false);
                    let (replay, replay_report) = section(&fixture, false);
                    let (swapped, swapped_report) = section(&fixture, true);
                    assert_eq!(forward, replay);
                    assert_eq!(forward_report, replay_report);
                    assert_eq!(forward_report.usage(), swapped_report.usage());
                    assert_dual_source_rulings(&forward, [0, 3, 2], 2, 2, 1);
                    assert_dual_source_rulings(&swapped, [0, 3, 2], 2, 2, 1);
                }
            }
        }
    }

    #[test]
    fn exact_shared_endpoint_work_accepts_n_and_refuses_n_minus_one() {
        for (fixture, expected_work) in [
            (
                public_fixture(Placement::World, (0.0, 2.0), (0.0, 2.0), false),
                131_283,
            ),
            (
                public_fixture(Placement::World, (0.0, 3.0), (0.0, 2.0), false),
                98_473,
            ),
            (
                public_fixture(Placement::Oblique, (-2.0, 4.0), (-1.0, 3.0), false),
                98_473,
            ),
        ] {
            let (expected, report) = section(&fixture, false);
            let exact_work = section_work(&report);
            assert_eq!(exact_work, expected_work);
            let admitted = section_with_work_limit(&fixture, exact_work);
            assert_eq!(admitted.result().unwrap(), &expected);

            let refused = section_with_work_limit(&fixture, exact_work - 1);
            let snapshot = refused
                .result()
                .unwrap_err()
                .limit()
                .expect("section refusal must retain exact limit evidence");
            assert_eq!(snapshot.stage, SECTION_WORK);
            assert_eq!(snapshot.resource, ResourceKind::Work);
            assert_eq!(snapshot.consumed, exact_work);
            assert_eq!(snapshot.allowed, exact_work - 1);
        }
    }

    #[test]
    fn nearby_overlapping_endpoint_enclosures_remain_unordered_without_exact_proof() {
        let near = f64::from_bits(1);
        let mut store = Store::new();
        let first = ktopo::make::cylinder(&mut store, &Frame::world(), 1.0, 2.0).unwrap();
        let second_frame = Frame::world().with_origin(Point3::new(1.0, 0.0, near));
        let second = ktopo::make::cylinder(&mut store, &second_frame, 1.0, 1.0).unwrap();
        let side_face = |body| {
            store
                .faces_of_body(body)
                .unwrap()
                .into_iter()
                .find(|face| {
                    matches!(
                        store.surface(store.get(*face).unwrap().surface).unwrap(),
                        SurfaceGeom::Cylinder(_)
                    )
                })
                .unwrap()
        };
        let faces = [side_face(first), side_face(second)];
        let policy = SessionPolicy::v1();
        let context = OperationContext::new(&policy, Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(BodySectionBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        let trace = |height| SectionUvLine {
            origin: Point2::new(1.0, height),
            direction: Vec2::new(0.0, 1.0),
        };
        let range = ParamRange::new(-1.0, 3.0);
        let ruling_clip::RulingClipOutcome::Spans(left) =
            ruling_clip::clip_ruling_to_face(&store, faces[0], trace(0.0), range, &mut scope)
                .unwrap()
        else {
            panic!("first cylinder must retain one exact band")
        };
        let ruling_clip::RulingClipOutcome::Spans(right) =
            ruling_clip::clip_ruling_to_face(&store, faces[1], trace(-near), range, &mut scope)
                .unwrap()
        else {
            panic!("second cylinder must retain one exact band")
        };
        assert!(
            certify_coincident_cap_ring_endpoints(
                &store,
                faces,
                [trace(0.0), trace(-near)],
                &left,
                &right,
                &mut scope,
            )
            .unwrap()
            .is_none()
        );
        assert_eq!(
            ruling_clip::merge_ruling_spans(&left, &right, range, &mut scope).unwrap(),
            ruling_clip::RulingMergeOutcome::Indeterminate(
                ruling_clip::RulingClipGap::UnorderedCrossings
            )
        );
    }

    fn assert_exact_axial_contact_publication(graph: &BodySectionGraph) {
        assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
        assert_eq!(graph.branches().len(), 2);
        assert_eq!(graph.curve_endpoints().len(), 2);
        assert_eq!(graph.curve_fragments().len(), 2);
        let [component] = graph.curve_components() else {
            panic!("strict axial contact must publish one closed component")
        };
        assert!(component.closed());
        assert_eq!(component.fragments(), [0, 1]);

        for endpoint in graph.curve_endpoints() {
            let SectionCurveEndpointTopology::Trim {
                sites,
                source_parameters,
            } = endpoint.topology()
            else {
                panic!("physical ring crossing cannot be a parameter seam")
            };
            for operand in 0..2 {
                let SectionSite::EdgeInterior(edge) = &sites[operand] else {
                    panic!("both topology-owned rings must key every contact endpoint")
                };
                let root = source_parameters[operand]
                    .as_ref()
                    .expect("both ring roots require scalar authority");
                assert_eq!(*edge, root.edge());
                let scalar = root.root_parameter();
                let enclosure = root.root_parameter_enclosure();
                assert!(scalar.is_finite());
                assert!(enclosure.lo().is_finite() && enclosure.hi().is_finite());
                assert!(enclosure.lo() < enclosure.hi());
                assert!(enclosure.contains(scalar));
                assert!(
                    endpoint.edge_parameters()[operand]
                        .expect("dual edge site requires observed parameter evidence")
                        .contains(scalar)
                );
            }
        }

        for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
            assert!(matches!(
                graph.branches()[fragment.branch()].carrier(),
                SectionCarrier::Circle { .. }
            ));
            let SectionCurveFragmentSpan::Arc { endpoints, .. } = fragment.span() else {
                panic!("contact boundary publication must retain bounded circle arcs")
            };
            let mut endpoint_indices = endpoints.each_ref().map(|end| end.endpoint());
            endpoint_indices.sort_unstable();
            assert_eq!(endpoint_indices, [0, 1]);
            for end in endpoints.iter() {
                let trim = end.trim();
                let SectionCurveEndpointTopology::Trim {
                    source_parameters, ..
                } = graph.curve_endpoints()[end.endpoint()].topology()
                else {
                    unreachable!()
                };
                assert_eq!(
                    source_parameters[trim.operand()].as_ref(),
                    Some(trim.source_parameter())
                );
            }
            assert_eq!(fragment.source_ordinal(), 0, "fragment {fragment_index}");
        }

        assert_eq!(
            graph
                .gaps()
                .iter()
                .filter(|gap| gap.reason() == GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY)
                .count(),
            2
        );
        assert!(
            graph
                .gaps()
                .iter()
                .any(|gap| gap.reason() == GAP_COINCIDENT_FACE_PAIR)
        );
    }

    #[test]
    fn exact_axial_contact_secant_rings_publish_two_proof_joined_arcs() {
        for placement in [Placement::World, Placement::Oblique] {
            for reverse_second_axis in [false, true] {
                let fixture = public_fixture_geometry(
                    placement,
                    (0.0, 1.0),
                    (1.0, 1.0),
                    reverse_second_axis,
                    1.0,
                    [1.0, 1.0],
                );
                let (forward, forward_report) = section(&fixture, false);
                let (replay, replay_report) = section(&fixture, false);
                let (swapped, swapped_report) = section(&fixture, true);
                assert_eq!(forward, replay);
                assert_eq!(forward_report, replay_report);
                assert_eq!(forward_report.usage(), swapped_report.usage());
                assert_exact_axial_contact_publication(&forward);
                assert_exact_axial_contact_publication(&swapped);
            }
        }
    }

    #[test]
    fn axial_contact_tangent_internal_and_coincident_rings_fail_closed() {
        for (name, radial_offset, radii) in [
            ("tangent", 2.0, [1.0, 1.0]),
            ("strict internal", 0.5, [2.0, 1.0]),
            ("coincident", 0.0, [1.0, 1.0]),
        ] {
            let fixture = public_fixture_geometry(
                Placement::World,
                (0.0, 1.0),
                (1.0, 1.0),
                false,
                radial_offset,
                radii,
            );
            let (graph, _) = section(&fixture, false);
            assert_eq!(
                graph.completion(),
                SectionCompletion::Indeterminate,
                "{name}"
            );
            assert!(graph.curve_endpoints().is_empty(), "{name}");
            assert!(graph.curve_fragments().is_empty(), "{name}");
            assert!(graph.curve_components().is_empty(), "{name}");
        }
    }

    #[test]
    fn exact_axial_contact_publication_work_accepts_n_and_refuses_n_minus_one() {
        let fixture = public_fixture_geometry(
            Placement::World,
            (0.0, 1.0),
            (1.0, 1.0),
            false,
            1.0,
            [1.0, 1.0],
        );
        let (expected, report) = section(&fixture, false);
        assert_exact_axial_contact_publication(&expected);
        let exact_work = section_work(&report);
        assert_eq!(exact_work, 65_629);

        let admitted = section_with_work_limit(&fixture, exact_work);
        assert_eq!(admitted.result().unwrap(), &expected);
        let refused = section_with_work_limit(&fixture, exact_work - 1);
        let snapshot = refused
            .result()
            .unwrap_err()
            .limit()
            .expect("section refusal must retain exact limit evidence");
        assert_eq!(snapshot.stage, SECTION_WORK);
        assert_eq!(snapshot.resource, ResourceKind::Work);
        assert_eq!(snapshot.consumed, exact_work);
        assert_eq!(snapshot.allowed, exact_work - 1);
    }
}
