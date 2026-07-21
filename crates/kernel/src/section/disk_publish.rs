//! Facade publication for proof-keyed affine chords across circular cap disks.
//!
//! Disk clipping owns complete circle-root identity and carrier ordering. This
//! adapter turns that evidence into the same affine fragment representation as
//! a topology-clipped ruling, so cap chords and cylinder-side rulings intern at
//! one exact endpoint and participate in one mixed directed component.

use super::closed_stitch::{
    CertifiedClosedEndpoint, CertifiedSourceParameterKey as ClosedSourceParameterKey,
};
use super::disk_clip::{CertifiedDiskCapChord, CertifiedDiskCapEndpoint};
use super::ruling_public::{
    SectionCarrierParameterInterval, SectionRulingFragmentEnd, SectionRulingTrimProvenance,
};
use super::{
    AdmittedFace, AdmittedFaceBoundary, PairCarrier, RulingRecertification, SectionAccumulator,
    SectionBranch, SectionBranchEvidence, SectionBranchTopology, SectionCurveEndpoint,
    SectionCurveFragment, SectionCurveFragmentSpan, SectionEdgeParameterInterval,
    SectionFragmentSite, SectionSourceParameterKey, SectionUvCurve, clip, root_identity, stitch,
};
use crate::error::{Error, Result};
use crate::{FaceId, FinId, LoopId, PartId};
use kcore::operation::OperationScope;
use ktopo::store::Store;

/// One certified chord linked to the branch allocated during section discovery.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CertifiedDiskCapFragment {
    branch: usize,
    chord: CertifiedDiskCapChord,
}

/// Clip, trim-admit, and accumulate one disk/polygon Plane pair.
#[allow(clippy::too_many_arguments)]
pub(super) fn process_pair(
    store: &Store,
    a: &AdmittedFace,
    b: &AdmittedFace,
    disk: super::disk_clip::CertifiedDiskCapAdmission,
    cap_operand: usize,
    pair: PairCarrier,
    linear: f64,
    root_identity: &mut root_identity::RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    if disk.face() != [a.raw, b.raw][cap_operand] {
        return Err(inconsistent_topology(
            "disk-cap admission changed face identity after dispatch",
        ));
    }
    let polygon_boundary = if cap_operand == 0 {
        &b.boundary
    } else {
        &a.boundary
    };
    let polygon = match polygon_boundary {
        AdmittedFaceBoundary::Polygon(polygon) => polygon,
        AdmittedFaceBoundary::Disk(_) => {
            return Err(inconsistent_topology(
                "disk-cap chord requires one opposing polygon trim",
            ));
        }
    };
    let polygon_spans = match clip::clip_face_with_analytic_plane(
        polygon,
        &pair.carrier,
        &disk.plane(),
        linear,
        scope,
    )? {
        clip::ClipOutcome::Spans(spans) => spans,
        clip::ClipOutcome::Gap(reason) => {
            acc.pair_gap(reason, a, b);
            return Ok(());
        }
    };
    let evidence = super::disk_clip::DiskCapPlanePairEvidence::new(
        pair.carrier,
        pair.uv_lines,
        pair.residual_bounds,
    );
    let chord = match super::disk_clip::clip_disk_cap(
        store,
        [a.raw, b.raw],
        cap_operand,
        disk.boundary_edge(),
        evidence,
        root_identity,
        scope,
    )? {
        super::disk_clip::DiskCapClipOutcome::Chord(chord) => chord,
        super::disk_clip::DiskCapClipOutcome::Indeterminate(
            super::disk_clip::DiskCapClipGap::EmptyIntersection,
        ) => return Ok(()),
        super::disk_clip::DiskCapClipOutcome::Indeterminate(gap) => {
            acc.pair_gap(gap.reason(), a, b);
            return Ok(());
        }
    };
    match polygon_chord_relation(&polygon_spans, &chord) {
        PolygonChordRelation::Disjoint => Ok(()),
        PolygonChordRelation::Contains => append_chord(
            &[a.facade.clone(), b.facade.clone()],
            chord,
            pair.tolerance,
            acc,
        ),
        PolygonChordRelation::Unresolved => {
            acc.pair_gap(super::GAP_DISK_CHORD_TRIM_UNRESOLVED, a, b);
            Ok(())
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PolygonChordRelation {
    Disjoint,
    Contains,
    Unresolved,
}

fn polygon_chord_relation(
    spans: &[clip::ClipSpan],
    chord: &CertifiedDiskCapChord,
) -> PolygonChordRelation {
    let [start, end] = chord.endpoints();
    let start = start.carrier_parameter();
    let end = end.carrier_parameter();
    let mut contains = 0usize;
    for span in spans {
        if span.end.parameter.hi() < start.lo() || end.hi() < span.start.parameter.lo() {
            continue;
        }
        if span.start.parameter.hi() < start.lo() && end.hi() < span.end.parameter.lo() {
            contains += 1;
            continue;
        }
        return PolygonChordRelation::Unresolved;
    }
    match contains {
        0 => PolygonChordRelation::Disjoint,
        1 => PolygonChordRelation::Contains,
        _ => PolygonChordRelation::Unresolved,
    }
}

/// Allocate the chord's affine carrier branch and retain its proof evidence.
pub(super) fn append_chord(
    facades: &[FaceId; 2],
    chord: CertifiedDiskCapChord,
    tolerance: f64,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    if chord
        .faces()
        .iter()
        .zip(facades)
        .any(|(raw, facade)| *raw != facade.raw())
        || !tolerance.is_finite()
        || tolerance < 0.0
    {
        return Err(inconsistent_topology(
            "disk-cap chord publication disagreed with its certified face pair",
        ));
    }
    let endpoints = chord.endpoints();
    let uv_lines = *chord.uv_lines();
    let branch = acc.branches.len();
    acc.branches.push(SectionBranch {
        faces: facades.clone(),
        carrier: chord.carrier(),
        range: chord.range(),
        topology: SectionBranchTopology::Open,
        pcurves: uv_lines.map(SectionUvCurve::Line),
        fragment_sites: endpoints
            .iter()
            .map(|endpoint| {
                let parameter = endpoint.carrier_parameter_representative();
                SectionFragmentSite {
                    point: endpoint.point(),
                    surface_parameters: uv_lines.map(|line| {
                        let uv = line.origin + line.direction * parameter;
                        [uv.x, uv.y]
                    }),
                    surface_window_boundaries: [false; 2],
                }
            })
            .collect(),
        endpoint_sites: [0, 1],
        evidence: SectionBranchEvidence {
            residual_bounds: chord.residual_bounds(),
            tolerance,
        },
        ruling_recertification: None::<RulingRecertification>,
        ruling_parameter_flipped: false,
    });
    acc.disk_fragments
        .push(CertifiedDiskCapFragment { branch, chord });
    Ok(())
}

/// Publish disk chords after existing analytic fragments, interning their
/// endpoints through the same operation-shared root identity seam.
pub(super) fn publish_fragments(
    part: &PartId,
    certified: &[CertifiedDiskCapFragment],
    endpoints: &mut Vec<SectionCurveEndpoint>,
    fragments: &mut Vec<SectionCurveFragment>,
) -> Result<()> {
    for certified in certified {
        let chord = &certified.chord;
        let mut public_ends = Vec::with_capacity(2);
        for evidence in chord.endpoints().iter().copied() {
            let exact = certified_endpoint(chord, evidence)?;
            let mut root_scalars = [None; 2];
            root_scalars[chord.cap_operand()] = Some(evidence.source_root_scalar());
            let endpoint =
                super::ruling_publish::intern_endpoint(part, exact, root_scalars, endpoints)?;
            let mut trims = [None, None];
            trims[chord.cap_operand()] = Some(adapt_trim(part, chord, evidence));
            public_ends.push(SectionRulingFragmentEnd {
                endpoint,
                point: evidence.point(),
                carrier_parameter: evidence.carrier_parameter_representative(),
                trims,
            });
        }
        let [start, end] = public_ends
            .try_into()
            .map_err(|_| inconsistent_topology("disk-cap chord did not retain two endpoints"))?;
        fragments.push(SectionCurveFragment {
            branch: certified.branch,
            source_ordinal: 0,
            span: SectionCurveFragmentSpan::LineSegment {
                endpoints: Box::new([start, end]),
            },
        });
    }
    Ok(())
}

fn certified_endpoint(
    chord: &CertifiedDiskCapChord,
    endpoint: CertifiedDiskCapEndpoint,
) -> Result<CertifiedClosedEndpoint> {
    let cap_operand = chord.cap_operand();
    if cap_operand >= 2 || endpoint.root().edge() != chord.boundary_edge() {
        return Err(inconsistent_topology(
            "disk-cap endpoint escaped its certified cap boundary",
        ));
    }
    let mut sites = chord.faces().map(stitch::SiteKey::Face);
    sites[cap_operand] = stitch::SiteKey::Edge(chord.boundary_edge());
    let mut keys = [None, None];
    keys[cap_operand] = Some(ClosedSourceParameterKey::new(
        chord.boundary_edge(),
        endpoint.root().ordinal(),
    ));
    let mut parameters = [None, None];
    parameters[cap_operand] = Some(endpoint.source_parameter());
    Ok(CertifiedClosedEndpoint::trim_site(
        stitch::VertexKey {
            a: sites[0],
            b: sites[1],
        },
        keys,
        parameters,
    ))
}

fn adapt_trim(
    part: &PartId,
    chord: &CertifiedDiskCapChord,
    endpoint: CertifiedDiskCapEndpoint,
) -> SectionRulingTrimProvenance {
    SectionRulingTrimProvenance {
        operand: chord.cap_operand(),
        face: FaceId::new(part.clone(), chord.faces()[chord.cap_operand()]),
        loop_id: LoopId::new(part.clone(), endpoint.cap_loop()),
        fin: FinId::new(part.clone(), endpoint.cap_fin()),
        source_parameter: SectionSourceParameterKey::from_certified_root(
            part,
            endpoint.root(),
            endpoint.source_root_scalar(),
        ),
        edge_parameter: SectionEdgeParameterInterval::from_interval(endpoint.source_parameter()),
        carrier_parameter: SectionCarrierParameterInterval::from_interval(
            endpoint.carrier_parameter(),
        ),
    }
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}
