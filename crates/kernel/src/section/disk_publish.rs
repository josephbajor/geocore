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
    SectionFragmentSite, SectionSourceParameterKey, SectionUvCurve, clip, interval_midpoint,
    root_identity, stitch,
};
use crate::error::{Error, Result};
use crate::{FaceId, FinId, LoopId, PartId};
use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kgeom::param::ParamRange;
use kgeom::vec::Point3;
use ktopo::entity::FaceId as RawFaceId;
use ktopo::store::Store;

/// One certified chord linked to the branch allocated during section discovery.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct CertifiedDiskCapFragment {
    branch: usize,
    faces: [RawFaceId; 2],
    endpoints: [CertifiedDiskFragmentEndpoint; 2],
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CertifiedDiskFragmentEndpoint {
    sources: [Option<CertifiedDiskCapEndpoint>; 2],
    source_roots: [Option<root_identity::SourceRootKey>; 2],
    source_root_scalars: [Option<root_identity::CertifiedSourceRootScalar>; 2],
    source_parameters: [Option<Interval>; 2],
    carrier_parameter: Interval,
    carrier_parameter_representative: f64,
    point: Point3,
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

/// Intersect two topology-certified disk chords on one Plane/Plane carrier.
///
/// Strict interval order owns ordinary containment. Overlapping endpoint
/// enclosures are admitted only when one already-published isolated contact
/// supplies exact source-root identity on both cap rings.
#[allow(clippy::too_many_arguments)]
pub(super) fn process_disk_pair(
    store: &Store,
    a: &AdmittedFace,
    b: &AdmittedFace,
    disks: [super::disk_clip::CertifiedDiskCapAdmission; 2],
    pair: PairCarrier,
    root_identity: &mut root_identity::RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut SectionAccumulator,
) -> Result<()> {
    if disks[0].face() != a.raw || disks[1].face() != b.raw {
        return Err(inconsistent_topology(
            "disk-pair admission changed face identity after dispatch",
        ));
    }
    let evidence = super::disk_clip::DiskCapPlanePairEvidence::new(
        pair.carrier,
        pair.uv_lines,
        pair.residual_bounds,
    );
    // Run both source-circle proofs before interpreting either outcome so
    // operand order does not change the attempted certification work.
    let clipped = [
        super::disk_clip::clip_disk_cap(
            store,
            [a.raw, b.raw],
            0,
            disks[0].boundary_edge(),
            evidence,
            root_identity,
            scope,
        )?,
        super::disk_clip::clip_disk_cap(
            store,
            [a.raw, b.raw],
            1,
            disks[1].boundary_edge(),
            evidence,
            root_identity,
            scope,
        )?,
    ];
    let chords = match clipped {
        [
            super::disk_clip::DiskCapClipOutcome::Chord(first),
            super::disk_clip::DiskCapClipOutcome::Chord(second),
        ] => [first, second],
        [
            super::disk_clip::DiskCapClipOutcome::Indeterminate(
                super::disk_clip::DiskCapClipGap::EmptyIntersection,
            ),
            _,
        ]
        | [
            _,
            super::disk_clip::DiskCapClipOutcome::Indeterminate(
                super::disk_clip::DiskCapClipGap::EmptyIntersection,
            ),
        ] => return Ok(()),
        [super::disk_clip::DiskCapClipOutcome::Indeterminate(gap), _]
        | [_, super::disk_clip::DiskCapClipOutcome::Indeterminate(gap)] => {
            acc.pair_gap(gap.reason(), a, b);
            return Ok(());
        }
    };
    let endpoints = match disk_pair_intersection(&chords, &acc.isolated_contacts) {
        DiskPairIntersection::Chord(endpoints) => *endpoints,
        DiskPairIntersection::Empty => return Ok(()),
        DiskPairIntersection::Unresolved => {
            acc.pair_gap(super::GAP_DISK_CHORD_TRIM_UNRESOLVED, a, b);
            return Ok(());
        }
    };
    append_certified_chord(
        &[a.facade.clone(), b.facade.clone()],
        chords[0].carrier(),
        *chords[0].uv_lines(),
        chords[0].residual_bounds(),
        pair.tolerance,
        endpoints,
        acc,
    )
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

#[derive(Debug, Clone, PartialEq)]
enum DiskPairIntersection {
    Empty,
    Chord(Box<[CertifiedDiskFragmentEndpoint; 2]>),
    Unresolved,
}

fn disk_pair_intersection(
    chords: &[CertifiedDiskCapChord; 2],
    contacts: &[super::skew_cylinder_fragment::CertifiedSectionIsolatedContact],
) -> DiskPairIntersection {
    if chords[0].faces() != chords[1].faces()
        || chords[0].cap_operand() != 0
        || chords[1].cap_operand() != 1
        || chords[0].carrier() != chords[1].carrier()
        || chords[0].uv_lines() != chords[1].uv_lines()
        || chords[0].residual_bounds() != chords[1].residual_bounds()
    {
        return DiskPairIntersection::Unresolved;
    }
    let [first_start, first_end] = *chords[0].endpoints();
    let [second_start, second_end] = *chords[1].endpoints();
    if first_end.carrier_parameter().hi() < second_start.carrier_parameter().lo()
        || second_end.carrier_parameter().hi() < first_start.carrier_parameter().lo()
    {
        return DiskPairIntersection::Empty;
    }

    // A closed-disk intersection that is exactly one already-published
    // contact stays zero-dimensional and must not become a degenerate chord.
    if intervals_overlap(
        first_end.carrier_parameter(),
        second_start.carrier_parameter(),
    ) && exact_contact_endpoint(chords[0].carrier(), first_end, second_start, contacts).is_some()
        && first_start.carrier_parameter().hi() < second_start.carrier_parameter().lo()
    {
        return DiskPairIntersection::Empty;
    }
    if intervals_overlap(
        second_end.carrier_parameter(),
        first_start.carrier_parameter(),
    ) && exact_contact_endpoint(chords[0].carrier(), first_start, second_end, contacts).is_some()
        && second_start.carrier_parameter().hi() < first_start.carrier_parameter().lo()
    {
        return DiskPairIntersection::Empty;
    }

    let start = if first_start.carrier_parameter().hi() < second_start.carrier_parameter().lo() {
        single_disk_endpoint(second_start, 1)
    } else if second_start.carrier_parameter().hi() < first_start.carrier_parameter().lo() {
        single_disk_endpoint(first_start, 0)
    } else {
        let Some(endpoint) =
            exact_contact_endpoint(chords[0].carrier(), first_start, second_start, contacts)
        else {
            return DiskPairIntersection::Unresolved;
        };
        endpoint
    };
    let end = if first_end.carrier_parameter().hi() < second_end.carrier_parameter().lo() {
        single_disk_endpoint(first_end, 0)
    } else if second_end.carrier_parameter().hi() < first_end.carrier_parameter().lo() {
        single_disk_endpoint(second_end, 1)
    } else {
        let Some(endpoint) =
            exact_contact_endpoint(chords[0].carrier(), first_end, second_end, contacts)
        else {
            return DiskPairIntersection::Unresolved;
        };
        endpoint
    };
    if start.carrier_parameter.hi() >= end.carrier_parameter.lo()
        || start.carrier_parameter_representative >= end.carrier_parameter_representative
    {
        return DiskPairIntersection::Unresolved;
    }
    DiskPairIntersection::Chord(Box::new([start, end]))
}

fn single_disk_endpoint(
    endpoint: CertifiedDiskCapEndpoint,
    operand: usize,
) -> CertifiedDiskFragmentEndpoint {
    let mut sources = [None, None];
    sources[operand] = Some(endpoint);
    let mut source_roots = [None, None];
    source_roots[operand] = Some(endpoint.root());
    let mut source_root_scalars = [None, None];
    source_root_scalars[operand] = Some(endpoint.source_root_scalar());
    let mut source_parameters = [None, None];
    source_parameters[operand] = Some(endpoint.source_parameter());
    CertifiedDiskFragmentEndpoint {
        sources,
        source_roots,
        source_root_scalars,
        source_parameters,
        carrier_parameter: endpoint.carrier_parameter(),
        carrier_parameter_representative: endpoint.carrier_parameter_representative(),
        point: endpoint.point(),
    }
}

fn exact_contact_endpoint(
    carrier: super::SectionCarrier,
    first: CertifiedDiskCapEndpoint,
    second: CertifiedDiskCapEndpoint,
    contacts: &[super::skew_cylinder_fragment::CertifiedSectionIsolatedContact],
) -> Option<CertifiedDiskFragmentEndpoint> {
    let contact = contacts.iter().find(|contact| {
        let [Some(left), Some(right)] = contact.root_evidence else {
            return false;
        };
        left.edge == first.root().edge()
            && right.edge == second.root().edge()
            && intervals_overlap(left.edge_parameter, first.source_parameter())
            && intervals_overlap(right.edge_parameter, second.source_parameter())
    })?;
    let [Some(left), Some(right)] = contact.root_evidence else {
        return None;
    };
    let [Some(left_scalar), Some(right_scalar)] = contact.root_scalars else {
        return None;
    };
    let carrier_parameter = hull(first.carrier_parameter(), second.carrier_parameter());
    let carrier_parameter_representative = interval_midpoint(carrier_parameter);
    let point = carrier_point(carrier, carrier_parameter_representative)?;
    Some(CertifiedDiskFragmentEndpoint {
        sources: [Some(first), Some(second)],
        source_roots: [Some(left.root), Some(right.root)],
        source_root_scalars: [Some(left_scalar), Some(right_scalar)],
        source_parameters: [
            Some(hull(first.source_parameter(), left.edge_parameter)),
            Some(hull(second.source_parameter(), right.edge_parameter)),
        ],
        carrier_parameter,
        carrier_parameter_representative,
        point,
    })
}

fn intervals_overlap(a: Interval, b: Interval) -> bool {
    a.lo() <= b.hi() && b.lo() <= a.hi()
}

fn hull(a: Interval, b: Interval) -> Interval {
    Interval::new(a.lo().min(b.lo()), a.hi().max(b.hi()))
}

fn carrier_point(carrier: super::SectionCarrier, parameter: f64) -> Option<Point3> {
    let super::SectionCarrier::Line { origin, direction } = carrier else {
        return None;
    };
    let point = origin + direction * parameter;
    [point.x, point.y, point.z]
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
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
    let endpoints =
        (*chord.endpoints()).map(|endpoint| single_disk_endpoint(endpoint, chord.cap_operand()));
    append_certified_chord(
        facades,
        chord.carrier(),
        *chord.uv_lines(),
        chord.residual_bounds(),
        tolerance,
        endpoints,
        acc,
    )
}

#[allow(clippy::too_many_arguments)]
fn append_certified_chord(
    facades: &[FaceId; 2],
    carrier: super::SectionCarrier,
    uv_lines: [super::SectionUvLine; 2],
    residual_bounds: [f64; 2],
    tolerance: f64,
    endpoints: [CertifiedDiskFragmentEndpoint; 2],
    acc: &mut SectionAccumulator,
) -> Result<()> {
    let faces = [facades[0].raw(), facades[1].raw()];
    if endpoints[0].carrier_parameter.hi() >= endpoints[1].carrier_parameter.lo()
        || endpoints[0].carrier_parameter_representative
            >= endpoints[1].carrier_parameter_representative
    {
        return Err(inconsistent_topology(
            "disk-cap fragment lost strict endpoint order before publication",
        ));
    }
    let branch = acc.branches.len();
    acc.branches.push(SectionBranch {
        faces: facades.clone(),
        carrier,
        range: ParamRange::new(
            endpoints[0].carrier_parameter_representative,
            endpoints[1].carrier_parameter_representative,
        ),
        topology: SectionBranchTopology::Open,
        pcurves: uv_lines.map(SectionUvCurve::Line),
        fragment_sites: endpoints
            .iter()
            .map(|endpoint| {
                let parameter = endpoint.carrier_parameter_representative;
                SectionFragmentSite {
                    point: endpoint.point,
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
            residual_bounds,
            tolerance,
        },
        skew_cylinder_embedding: None,
        ruling_recertification: None::<RulingRecertification>,
        ruling_parameter_flipped: false,
    });
    acc.disk_fragments.push(CertifiedDiskCapFragment {
        branch,
        faces,
        endpoints,
    });
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
        let mut public_ends = Vec::with_capacity(2);
        for evidence in certified.endpoints {
            let exact = certified_endpoint(certified.faces, evidence)?;
            let endpoint = super::ruling_publish::intern_endpoint(
                part,
                exact,
                evidence.source_root_scalars,
                endpoints,
            )?;
            let mut trims = [None, None];
            for (operand, trim) in trims.iter_mut().enumerate() {
                *trim = adapt_trim(part, certified.faces, operand, evidence);
            }
            public_ends.push(SectionRulingFragmentEnd {
                endpoint,
                point: evidence.point,
                carrier_parameter: evidence.carrier_parameter_representative,
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
    faces: [RawFaceId; 2],
    endpoint: CertifiedDiskFragmentEndpoint,
) -> Result<CertifiedClosedEndpoint> {
    let mut sites = faces.map(stitch::SiteKey::Face);
    let mut keys = [None, None];
    for operand in 0..2 {
        match (
            endpoint.sources[operand],
            endpoint.source_roots[operand],
            endpoint.source_root_scalars[operand],
            endpoint.source_parameters[operand],
        ) {
            (Some(source), Some(root), Some(_), Some(_)) => {
                sites[operand] = stitch::SiteKey::Edge(source.root().edge());
                keys[operand] = Some(ClosedSourceParameterKey::new(root.edge(), root.ordinal()));
            }
            (None, None, None, None) => {}
            _ => {
                return Err(inconsistent_topology(
                    "disk endpoint source identity and scalar authority disagree",
                ));
            }
        }
    }
    Ok(CertifiedClosedEndpoint::trim_site(
        stitch::VertexKey {
            a: sites[0],
            b: sites[1],
        },
        keys,
        endpoint.source_parameters,
    ))
}

fn adapt_trim(
    part: &PartId,
    faces: [RawFaceId; 2],
    operand: usize,
    endpoint: CertifiedDiskFragmentEndpoint,
) -> Option<SectionRulingTrimProvenance> {
    let source = endpoint.sources[operand]?;
    let source_root = endpoint.source_roots[operand]?;
    let source_root_scalar = endpoint.source_root_scalars[operand]?;
    let source_parameter = endpoint.source_parameters[operand]?;
    Some(SectionRulingTrimProvenance {
        operand,
        face: FaceId::new(part.clone(), faces[operand]),
        loop_id: LoopId::new(part.clone(), source.cap_loop()),
        fin: FinId::new(part.clone(), source.cap_fin()),
        source_parameter: SectionSourceParameterKey::from_certified_root(
            part,
            source_root,
            source_root_scalar,
        ),
        edge_parameter: SectionEdgeParameterInterval::from_interval(source_parameter),
        carrier_parameter: SectionCarrierParameterInterval::from_interval(
            endpoint.carrier_parameter,
        ),
    })
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}
