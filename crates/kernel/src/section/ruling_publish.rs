//! Adapt topology-clipped rulings into proof-keyed facade fragments.

use kcore::interval::Interval;
use kcore::math;
use kcore::operation::OperationScope;
use kgeom::param::ParamRange;
use kgeom::vec::{Point3, Vec3};
use kgraph::certify_paired_plane_cylinder_ruling_residuals;
use ktopo::entity::FaceId as RawFaceId;
use ktopo::geom::CurveGeom;
use ktopo::store::Store;

use super::closed_stitch::{
    CertifiedClosedEndpoint, CertifiedClosedEndpointKey, CertifiedSourceParameterKey,
};
use super::root_identity::{RootIdentityAuthority, RootResolution, SourceRootKey, SourceRootQuery};
use super::ruling_clip::{MergedRulingEndpoint, MergedRulingSpan, RulingTrimSite};
use super::ruling_public::{
    SectionCarrierParameterInterval, SectionRulingFragmentEnd, SectionRulingTrimProvenance,
};
use super::{
    SectionBranch, SectionCarrier, SectionCurveEndpoint, SectionCurveEndpointTopology,
    SectionCurveFragment, SectionCurveFragmentSpan, SectionEdgeParameterInterval, SectionGap,
    SectionSourceParameterKey, SectionUvCurve, adapt_site, interval_midpoint, stitch,
};
use crate::error::{Error, Result};
use crate::{FaceId, FinId, LoopId, PartId};

/// Clip, merge, identity-certify, and accumulate one ruling branch.
pub(super) fn append_branch(
    store: &Store,
    raw_faces: [RawFaceId; 2],
    facades: &[FaceId; 2],
    branch: SectionBranch,
    root_identity: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
    acc: &mut super::SectionAccumulator,
) -> Result<()> {
    let [SectionUvCurve::Line(trace_a), SectionUvCurve::Line(trace_b)] = branch.pcurves else {
        acc.branches.push(branch);
        acc.gaps.push(SectionGap {
            reason: super::GAP_PAIR_UNRESOLVED,
            faces: facades.to_vec(),
        });
        return Ok(());
    };
    let clipped = [
        super::ruling_clip::clip_ruling_to_face(store, raw_faces[0], trace_a, branch.range, scope)?,
        super::ruling_clip::clip_ruling_to_face(store, raw_faces[1], trace_b, branch.range, scope)?,
    ];
    let branch_index = acc.branches.len();
    acc.branches.push(branch);
    let spans = match (&clipped[0], &clipped[1]) {
        (
            super::ruling_clip::RulingClipOutcome::Spans(a),
            super::ruling_clip::RulingClipOutcome::Spans(b),
        ) => match super::ruling_clip::merge_ruling_spans(
            a,
            b,
            acc.branches[branch_index].range,
            scope,
        )? {
            super::ruling_clip::RulingMergeOutcome::Spans(spans) => spans,
            super::ruling_clip::RulingMergeOutcome::Indeterminate(gap) => {
                acc.gaps.push(SectionGap {
                    reason: gap.reason(),
                    faces: facades.to_vec(),
                });
                return Ok(());
            }
        },
        (super::ruling_clip::RulingClipOutcome::Indeterminate(gap), _)
        | (_, super::ruling_clip::RulingClipOutcome::Indeterminate(gap)) => {
            acc.gaps.push(SectionGap {
                reason: gap.reason(),
                faces: facades.to_vec(),
            });
            return Ok(());
        }
    };
    match certify_fragments(
        store,
        raw_faces,
        branch_index,
        &acc.branches[branch_index],
        &spans,
        root_identity,
        scope,
    )? {
        RulingCertificationOutcome::Fragments(fragments) => {
            if !fragments.is_empty() {
                let Some(certificate) =
                    recertify_expanded_branch(&acc.branches[branch_index], &fragments)?
                else {
                    acc.gaps.push(SectionGap {
                        reason: super::GAP_CARRIER_ORIENTATION,
                        faces: facades.to_vec(),
                    });
                    return Ok(());
                };
                acc.branches[branch_index].range = certificate.range;
                acc.branches[branch_index].evidence.residual_bounds = certificate.residual_bounds;
                acc.ruling_fragments.extend(fragments);
            }
        }
        RulingCertificationOutcome::Indeterminate(reason) => acc.gaps.push(SectionGap {
            reason,
            faces: facades.to_vec(),
        }),
    }
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedRulingEndpoint {
    endpoint: CertifiedClosedEndpoint,
    carrier_parameter: Interval,
    sites: [Option<CertifiedRulingTrimSite>; 2],
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CertifiedRulingTrimSite {
    operand: usize,
    site: RulingTrimSite,
    root: SourceRootKey,
    source_root_scalar: super::root_identity::CertifiedSourceRootScalar,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct CertifiedRulingFragment {
    pub(super) branch: usize,
    pub(super) ordinal: usize,
    endpoints: [CertifiedRulingEndpoint; 2],
}

#[derive(Debug, Clone, PartialEq)]
pub(super) enum RulingCertificationOutcome {
    Fragments(Vec<CertifiedRulingFragment>),
    Indeterminate(&'static str),
}

/// Issue operation-shared source-root identities for every ruling endpoint.
pub(super) fn certify_fragments(
    store: &Store,
    faces: [RawFaceId; 2],
    branch: usize,
    branch_evidence: &SectionBranch,
    spans: &[MergedRulingSpan],
    roots: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<RulingCertificationOutcome> {
    let mut fragments = Vec::with_capacity(spans.len());
    let mut previous_end = None;
    for (ordinal, span) in spans.iter().copied().enumerate() {
        let start = match certify_endpoint(store, faces, branch_evidence, span.start, roots, scope)?
        {
            Ok(endpoint) => endpoint,
            Err(reason) => return Ok(RulingCertificationOutcome::Indeterminate(reason)),
        };
        let end = match certify_endpoint(store, faces, branch_evidence, span.end, roots, scope)? {
            Ok(endpoint) => endpoint,
            Err(reason) => return Ok(RulingCertificationOutcome::Indeterminate(reason)),
        };
        if !strict_fragment_order(previous_end, start.carrier_parameter, end.carrier_parameter) {
            return Ok(RulingCertificationOutcome::Indeterminate(
                super::GAP_UNORDERED_CROSSINGS,
            ));
        }
        previous_end = Some(end.carrier_parameter);
        fragments.push(CertifiedRulingFragment {
            branch,
            ordinal,
            endpoints: [start, end],
        });
    }
    Ok(RulingCertificationOutcome::Fragments(fragments))
}

fn strict_fragment_order(previous_end: Option<Interval>, start: Interval, end: Interval) -> bool {
    previous_end.is_none_or(|previous| previous.hi() < start.lo()) && start.hi() < end.lo()
}

fn certify_endpoint(
    store: &Store,
    faces: [RawFaceId; 2],
    branch: &SectionBranch,
    merged: MergedRulingEndpoint,
    roots: &mut RootIdentityAuthority,
    scope: &mut OperationScope<'_, '_>,
) -> Result<core::result::Result<CertifiedRulingEndpoint, &'static str>> {
    let mut topology_sites = faces.map(stitch::SiteKey::Face);
    let mut root_keys = [None, None];
    let mut parameters = [None, None];
    let mut certified_sites = [None, None];
    let mut carrier_parameter = None;
    for operand in 0..2 {
        let Some(site) = merged.sites[operand] else {
            if merged.edge_parameters[operand].is_some() {
                return Ok(Err(super::GAP_INCOMPATIBLE_EDGE_PARAMETERS));
            }
            continue;
        };
        if site.face != faces[operand]
            || merged.edge_parameters[operand] != Some(site.edge_parameter)
        {
            return Ok(Err(super::GAP_INCOMPATIBLE_EDGE_PARAMETERS));
        }
        let root = match roots.resolve(
            store,
            SourceRootQuery::new(site.edge, faces[1 - operand]),
            site.edge_parameter,
            scope,
        )? {
            RootResolution::Resolved(root) => root,
            RootResolution::Indeterminate(gap) => return Ok(Err(gap.reason())),
        };
        let (root_parameter, source_root_scalar) = match roots.certify_order(
            store,
            SourceRootQuery::new(site.edge, faces[1 - operand]),
            scope,
        )? {
            super::root_identity::RootOrderOutcome::Certified(order) => {
                let parameter = order.roots().get(root.ordinal()).copied().ok_or(
                    Error::InconsistentTopology {
                        source: kcore::error::Error::InvalidGeometry {
                            reason: "source-root ordinal escaped its certified order",
                        },
                    },
                )?;
                let scalar = order.materialize(root).ok_or(Error::InconsistentTopology {
                    source: kcore::error::Error::InvalidGeometry {
                        reason: "ruling source root has no canonical scalar materialization",
                    },
                })?;
                (parameter, scalar)
            }
            super::root_identity::RootOrderOutcome::Indeterminate(gap) => {
                return Ok(Err(gap.reason()));
            }
        };
        let projected =
            match project_source_root_to_carrier(store, site.edge, root_parameter, branch)? {
                Some(parameter) => parameter,
                None => return Ok(Err(super::GAP_CARRIER_ORIENTATION)),
            };
        // Whole-fin incidence permits a tolerance-close pcurve lift rather
        // than coefficient-exact equality with the 3D edge. The unique
        // overlap in source-edge parameter associates this pcurve event with
        // `root`, but intersecting their independent enclosures could discard
        // both exact quantities. Retain hulls so public evidence encloses the
        // analytic source root and the topology-owned pcurve observation.
        let associated_edge_parameter = hull_interval(root_parameter, site.edge_parameter);
        let associated_carrier_parameter = hull_interval(projected, site.carrier_parameter);
        carrier_parameter = match carrier_parameter {
            None => Some(associated_carrier_parameter),
            Some(current) => Some(hull_interval(current, associated_carrier_parameter)),
        };
        let mut site = site;
        site.edge_parameter = associated_edge_parameter;
        topology_sites[operand] = stitch::SiteKey::Edge(site.edge);
        root_keys[operand] = Some(CertifiedSourceParameterKey::new(
            root.edge(),
            root.ordinal(),
        ));
        parameters[operand] = Some(associated_edge_parameter);
        certified_sites[operand] = Some(CertifiedRulingTrimSite {
            operand,
            site,
            root,
            source_root_scalar,
        });
    }
    if certified_sites.iter().all(Option::is_none) {
        return Ok(Err(super::GAP_INCOMPATIBLE_EDGE_PARAMETERS));
    }
    Ok(Ok(CertifiedRulingEndpoint {
        endpoint: CertifiedClosedEndpoint::trim_site(
            stitch::VertexKey {
                a: topology_sites[0],
                b: topology_sites[1],
            },
            root_keys,
            parameters,
        ),
        carrier_parameter: carrier_parameter.ok_or(Error::InconsistentTopology {
            source: kcore::error::Error::InvalidGeometry {
                reason: "ruling endpoint has no certified carrier parameter",
            },
        })?,
        sites: certified_sites,
    }))
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ExpandedRulingCertificate {
    range: ParamRange,
    residual_bounds: [f64; 2],
}

/// Reissue the paired trace proof over every outward endpoint enclosure.
///
/// Topology clipping starts from the graph-owned finite carrier window, but
/// an analytic source-root projection can straddle that window under outward
/// arithmetic and whole-fin tolerance correspondence. Publishing only an
/// overlap would discard possible event values. Instead, retain the full
/// topology-observation/analytic-root hull and extend the branch proof to
/// cover it before publication.
fn recertify_expanded_branch(
    branch: &SectionBranch,
    fragments: &[CertifiedRulingFragment],
) -> Result<Option<ExpandedRulingCertificate>> {
    let mut range = branch.range;
    for fragment in fragments {
        for endpoint in fragment.endpoints {
            range.lo = range.lo.min(endpoint.carrier_parameter.lo());
            range.hi = range.hi.max(endpoint.carrier_parameter.hi());
        }
    }
    if !range.is_finite() || range.lo >= range.hi {
        return Err(inconsistent_topology(
            "ruling endpoints produced an invalid expanded proof range",
        ));
    }
    let source = branch
        .ruling_recertification
        .as_ref()
        .ok_or_else(|| inconsistent_topology("ruling branch lost its recertification source"))?;
    let source_range = if branch.ruling_parameter_flipped {
        ParamRange {
            lo: -range.hi,
            hi: -range.lo,
        }
    } else {
        range
    };
    let residual_bounds = match source {
        super::RulingRecertification::Graph(source) => {
            let Ok(certificate) = certify_paired_plane_cylinder_ruling_residuals(
                source.carrier(),
                source_range,
                source.traces(),
                source.tolerance(),
            ) else {
                return Ok(None);
            };
            certificate.residual_bounds()
        }
        super::RulingRecertification::Semantic(source) => {
            let Some(bounds) = super::semantic_ruling::recertify(branch, range, source) else {
                return Ok(None);
            };
            bounds
        }
    };
    Ok(Some(ExpandedRulingCertificate {
        range,
        residual_bounds,
    }))
}

fn hull_interval(a: Interval, b: Interval) -> Interval {
    Interval::new(a.lo().min(b.lo()), a.hi().max(b.hi()))
}

fn interval_point(point: Point3) -> [Interval; 3] {
    point.to_array().map(Interval::point)
}

fn interval_vec(vector: Vec3) -> [Interval; 3] {
    vector.to_array().map(Interval::point)
}

fn dot(a: [Interval; 3], b: [Interval; 3]) -> Interval {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn project_point_to_line(
    point: [Interval; 3],
    origin: Point3,
    direction: Vec3,
) -> Option<Interval> {
    let origin = interval_point(origin);
    let direction = interval_vec(direction);
    let relative = core::array::from_fn(|axis| point[axis] - origin[axis]);
    dot(relative, direction).checked_div(dot(direction, direction))
}

fn project_source_root_to_carrier(
    store: &Store,
    edge_id: ktopo::entity::EdgeId,
    root: Interval,
    branch: &SectionBranch,
) -> Result<Option<Interval>> {
    let SectionCarrier::Line { origin, direction } = branch.carrier else {
        return Err(inconsistent_topology(
            "ruling source-root projection received a non-line branch",
        ));
    };
    let edge = store
        .get(edge_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let Some(curve_id) = edge.curve() else {
        return Err(inconsistent_topology(
            "resolved ruling source root lost its source curve",
        ));
    };
    let curve = store
        .curve(curve_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let point = match curve {
        CurveGeom::Line(line) => {
            let line_origin = interval_point(line.origin());
            let line_direction = interval_vec(line.dir());
            core::array::from_fn(|axis| line_origin[axis] + line_direction[axis] * root)
        }
        CurveGeom::Circle(circle) => {
            let midpoint = 0.5 * root.lo() + 0.5 * root.hi();
            let delta = (root.hi() - root.lo()).next_up();
            let (sin, cos) = math::sincos(midpoint);
            if !midpoint.is_finite() || !delta.is_finite() || !sin.is_finite() || !cos.is_finite() {
                return Ok(None);
            }
            // Deterministic trig is accurate to <1 ulp and mathematical
            // sin/cos are 1-Lipschitz. This encloses every source-circle
            // point over the certified root interval without assuming the
            // rounded frame axes are exactly orthogonal to the ruling.
            let sin = Interval::new(
                (sin.next_down() - delta).next_down(),
                (sin.next_up() + delta).next_up(),
            );
            let cos = Interval::new(
                (cos.next_down() - delta).next_down(),
                (cos.next_up() + delta).next_up(),
            );
            let center = interval_point(circle.frame().origin());
            let x = interval_vec(circle.frame().x());
            let y = interval_vec(circle.frame().y());
            let radius = Interval::point(circle.radius());
            core::array::from_fn(|axis| center[axis] + radius * (x[axis] * cos + y[axis] * sin))
        }
        _ => {
            return Err(inconsistent_topology(
                "resolved ruling source root changed to an unsupported curve class",
            ));
        }
    };
    Ok(project_point_to_line(point, origin, direction))
}

/// Append ruling fragments, interning endpoints against previously-published
/// conic endpoints by exact topology/root identity.
pub(super) fn publish_fragments(
    part: &PartId,
    branches: &[SectionBranch],
    certified: &[CertifiedRulingFragment],
    endpoints: &mut Vec<SectionCurveEndpoint>,
    fragments: &mut Vec<SectionCurveFragment>,
) -> Result<()> {
    for fragment in certified {
        let Some(branch) = branches.get(fragment.branch) else {
            return Err(inconsistent_topology(
                "ruling fragment referenced an unknown section branch",
            ));
        };
        let mut public_ends = Vec::with_capacity(2);
        for evidence in fragment.endpoints {
            let root_scalars = evidence
                .sites
                .map(|site| site.map(|site| site.source_root_scalar));
            let endpoint = intern_endpoint(part, evidence.endpoint, root_scalars, endpoints)?;
            let parameter = interval_midpoint(evidence.carrier_parameter);
            let point = line_point(branch, parameter).ok_or_else(|| {
                inconsistent_topology("ruling fragment has no finite carrier representative")
            })?;
            let trims = evidence
                .sites
                .map(|site| site.map(|site| adapt_trim(part, evidence.carrier_parameter, site)));
            public_ends.push(SectionRulingFragmentEnd {
                endpoint,
                point,
                carrier_parameter: parameter,
                trims,
            });
        }
        let [start, end] = public_ends
            .try_into()
            .map_err(|_| inconsistent_topology("ruling fragment did not retain two endpoints"))?;
        fragments.push(SectionCurveFragment {
            branch: fragment.branch,
            source_ordinal: fragment.ordinal,
            span: SectionCurveFragmentSpan::LineSegment {
                endpoints: Box::new([start, end]),
            },
        });
    }
    Ok(())
}

fn line_point(branch: &SectionBranch, parameter: f64) -> Option<Point3> {
    let SectionCarrier::Line { origin, direction } = branch.carrier else {
        return None;
    };
    let point = origin + direction * parameter;
    [point.x, point.y, point.z]
        .into_iter()
        .all(f64::is_finite)
        .then_some(point)
}

fn adapt_trim(
    part: &PartId,
    carrier_parameter: Interval,
    certified: CertifiedRulingTrimSite,
) -> SectionRulingTrimProvenance {
    let site = certified.site;
    SectionRulingTrimProvenance {
        operand: certified.operand,
        face: FaceId::new(part.clone(), site.face),
        loop_id: LoopId::new(part.clone(), site.loop_id),
        fin: FinId::new(part.clone(), site.fin),
        source_parameter: SectionSourceParameterKey::from_certified_root(
            part,
            certified.root,
            certified.source_root_scalar,
        ),
        edge_parameter: SectionEdgeParameterInterval::from_interval(site.edge_parameter),
        carrier_parameter: SectionCarrierParameterInterval::from_interval(carrier_parameter),
    }
}

pub(super) fn intern_endpoint(
    part: &PartId,
    certified: CertifiedClosedEndpoint,
    root_scalars: [Option<super::root_identity::CertifiedSourceRootScalar>; 2],
    endpoints: &mut Vec<SectionCurveEndpoint>,
) -> Result<usize> {
    let candidate = adapt_endpoint(part, certified, root_scalars)?;
    if let Some(index) = endpoints
        .iter()
        .position(|endpoint| endpoint.topology == candidate.topology)
    {
        if !endpoint_materializations_match(&endpoints[index], &candidate) {
            return Err(inconsistent_topology(
                "one source-root identity retained inconsistent scalar materializations",
            ));
        }
        for operand in 0..2 {
            match (
                endpoints[index].edge_parameters[operand],
                candidate.edge_parameters[operand],
            ) {
                (None, None) => {}
                (Some(current), Some(incoming)) => {
                    let lo = current.lo().max(incoming.lo());
                    let hi = current.hi().min(incoming.hi());
                    if lo > hi {
                        return Err(inconsistent_topology(
                            "identical section endpoint roots retained disjoint edge parameters",
                        ));
                    }
                    endpoints[index].edge_parameters[operand] =
                        Some(SectionEdgeParameterInterval { lo, hi });
                }
                _ => {
                    return Err(inconsistent_topology(
                        "identical section endpoint roots retained incompatible operand evidence",
                    ));
                }
            }
        }
        return Ok(index);
    }
    let index = endpoints.len();
    endpoints.push(candidate);
    Ok(index)
}

fn endpoint_materializations_match(
    first: &SectionCurveEndpoint,
    second: &SectionCurveEndpoint,
) -> bool {
    match (&first.topology, &second.topology) {
        (
            SectionCurveEndpointTopology::Trim {
                source_parameters: first,
                ..
            },
            SectionCurveEndpointTopology::Trim {
                source_parameters: second,
                ..
            },
        ) => first
            .iter()
            .zip(second)
            .all(|(first, second)| match (first, second) {
                (None, None) => true,
                (Some(first), Some(second)) => first.has_same_materialization(second),
                _ => false,
            }),
        (
            SectionCurveEndpointTopology::ParameterSeam { .. },
            SectionCurveEndpointTopology::ParameterSeam { .. },
        ) => true,
        _ => false,
    }
}

fn inconsistent_topology(reason: &'static str) -> Error {
    Error::InconsistentTopology {
        source: kcore::error::Error::InvalidGeometry { reason },
    }
}

fn adapt_endpoint(
    part: &PartId,
    certified: CertifiedClosedEndpoint,
    root_scalars: [Option<super::root_identity::CertifiedSourceRootScalar>; 2],
) -> Result<SectionCurveEndpoint> {
    let topology = match certified.key {
        CertifiedClosedEndpointKey::TrimSite {
            site,
            edge_parameter_keys,
        } => {
            let mut source_parameters = [None, None];
            for operand in 0..2 {
                source_parameters[operand] =
                    match (edge_parameter_keys[operand], root_scalars[operand]) {
                        (None, None) => None,
                        (Some(key), Some(scalar)) => {
                            Some(SectionSourceParameterKey::from_certified_root(
                                part,
                                SourceRootKey::new(key.edge(), key.root_ordinal()),
                                scalar,
                            ))
                        }
                        _ => {
                            return Err(inconsistent_topology(
                                "ruling endpoint root identity and scalar authority disagree",
                            ));
                        }
                    };
            }
            SectionCurveEndpointTopology::Trim {
                sites: [adapt_site(part, site.a), adapt_site(part, site.b)],
                source_parameters,
            }
        }
        CertifiedClosedEndpointKey::PeriodSeam { branch, site } => {
            if root_scalars != [None, None] {
                return Err(inconsistent_topology(
                    "parameter-seam endpoint retained a physical root scalar",
                ));
            }
            SectionCurveEndpointTopology::ParameterSeam {
                branch: branch.index(),
                site,
            }
        }
    };
    Ok(SectionCurveEndpoint {
        topology,
        edge_parameters: certified
            .edge_parameters
            .map(|value| value.map(SectionEdgeParameterInterval::from_interval)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn association_hull_retains_both_independent_event_enclosures() {
        let analytic = Interval::new(-2.0, 1.0);
        let topology = Interval::new(0.0, 3.0);
        let hull = hull_interval(analytic, topology);

        assert_eq!(hull, Interval::new(-2.0, 3.0));
        assert!(hull.lo() <= analytic.lo() && hull.hi() >= analytic.hi());
        assert!(hull.lo() <= topology.lo() && hull.hi() >= topology.hi());
    }

    #[test]
    fn widened_fragment_hulls_require_global_strict_order() {
        assert!(strict_fragment_order(
            None,
            Interval::new(-3.0, -2.0),
            Interval::new(-1.0, 0.0),
        ));
        assert!(strict_fragment_order(
            Some(Interval::new(-1.0, 0.0)),
            Interval::new(1.0, 2.0),
            Interval::new(3.0, 4.0),
        ));
        assert!(!strict_fragment_order(
            Some(Interval::new(-1.0, 1.5)),
            Interval::new(1.0, 2.0),
            Interval::new(3.0, 4.0),
        ));
        assert!(!strict_fragment_order(
            None,
            Interval::new(1.0, 3.5),
            Interval::new(3.0, 4.0),
        ));
    }
}
