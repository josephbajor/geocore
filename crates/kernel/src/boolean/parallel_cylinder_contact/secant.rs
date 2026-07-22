//! Exact split-ring realization for strict-secant axial contact.
//!
//! Section publishes one inside arc from each touching source ring. This
//! module binds those arcs to the two topology-owned rings through their dual
//! source-root identities, recovers source-edge orientation from pcurves, and
//! completes each ring with its exact complementary outside arc. The result
//! has two cylinder bands, two far caps, and two crescent contact caps.

use kgeom::curve::{Circle, Curve};
use kgeom::param::ParamRange;
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticEdgeSplitPiece, AnalyticFaceKey, AnalyticPcurveUse,
    AnalyticShellClosedEdge, AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace,
    AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop, AnalyticShellSurface,
    AnalyticShellVertex, AnalyticVertexKey,
};
use ktopo::entity::{EntityRef, FaceDomain, Sense};
use ktopo::geom::Curve2dGeom;
use ktopo::store::Store;

use super::super::periodic_chart;
use super::{
    ContactPlanGap, ContactSource, PERIOD, projected_circle_pcurve, source_circle,
    source_face_data, source_fin_sense, source_pcurve, source_plane, vectors_are_exactly_parallel,
};
use crate::section::{
    GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY, GAP_COINCIDENT_FACE_PAIR, GAP_PAIR_UNRESOLVED,
};
use crate::{
    BodySectionGraph, SectionBranchTopology, SectionCarrier, SectionCompletion,
    SectionCurveEndpointTopology, SectionCurveFragmentSpan, SectionPeriodicEmbeddingGap,
    SectionPeriodicFaceEmbeddingEvidence, SectionSite, SectionUvCurve,
};

#[derive(Debug, Clone, Copy)]
struct BoundRoot {
    endpoint: usize,
    parameter: f64,
    enclosure: [f64; 2],
}

#[derive(Debug, Clone, Copy)]
struct BoundRing {
    fragment: usize,
    circle: Circle,
    roots: [BoundRoot; 2],
    inside_piece: usize,
}

#[derive(Debug, Clone, Copy)]
struct RingPiece {
    key: AnalyticEdgeKey,
    vertices: [AnalyticVertexKey; 2],
    range: ParamRange,
}

#[derive(Debug, Clone, Copy)]
struct PreparedSideChart {
    surface: AnalyticShellSurface,
    domain: FaceDomain,
    far_pcurve: AnalyticPcurveUse,
    piece_pcurves: [AnalyticPcurveUse; 2],
}

pub(super) fn prepare_strict_secant_contact_shell(
    store: &Store,
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<AnalyticShellInput, ContactPlanGap> {
    let rings = bind_strict_secant_graph(store, graph, sources)?;
    build_strict_secant_shell(store, sources, &rings)
}

fn bind_strict_secant_graph(
    store: &Store,
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<[BoundRing; 2], ContactPlanGap> {
    if graph.completion() != SectionCompletion::Indeterminate
        || !graph.vertices().is_empty()
        || !graph.edges().is_empty()
        || !graph.loops().is_empty()
        || !graph.rings().is_empty()
        || !graph
            .cylinder_cylinder_exterior_radial_separations()
            .is_empty()
        || graph.branches().len() != 2
        || graph.curve_endpoints().len() != 2
        || graph.curve_fragments().len() != 2
    {
        return Err(ContactPlanGap::RelationBinding);
    }
    let [component] = graph.curve_components() else {
        return Err(ContactPlanGap::RelationBinding);
    };
    let mut component_fragments = component.fragments().to_vec();
    component_fragments.sort_unstable();
    if !component.closed() || component_fragments != [0, 1] {
        return Err(ContactPlanGap::RelationBinding);
    }
    validate_contact_gaps(graph, sources)?;

    let roots = bind_dual_roots(graph, sources)?;
    let mut bound = [None, None];
    for (fragment_index, fragment) in graph.curve_fragments().iter().enumerate() {
        let branch = graph
            .branches()
            .get(fragment.branch())
            .ok_or(ContactPlanGap::RelationBinding)?;
        let owner = (0..2)
            .find(|&operand| {
                let peer = 1 - operand;
                branch.faces()[operand].raw() == sources[operand].source.side_face()
                    && branch.faces()[peer].raw()
                        == sources[peer].source.boundaries()[sources[peer].contact_boundary]
                            .cap_face()
            })
            .ok_or(ContactPlanGap::RelationBinding)?;
        if bound[owner].is_some() {
            return Err(ContactPlanGap::RelationBinding);
        }
        let peer = 1 - owner;
        let circle = source_circle(
            store,
            sources[owner].source.boundaries()[sources[owner].contact_boundary],
        )?;
        validate_branch_support(branch, circle)?;
        let SectionCurveFragmentSpan::Arc { endpoints, .. } = fragment.span() else {
            return Err(ContactPlanGap::RelationBinding);
        };
        if fragment.source_ordinal() != 0 || endpoints[0].endpoint() == endpoints[1].endpoint() || {
            let mut endpoint_ids = endpoints.each_ref().map(|end| end.endpoint());
            endpoint_ids.sort_unstable();
            endpoint_ids != [0, 1]
        } {
            return Err(ContactPlanGap::RelationBinding);
        }
        for end in endpoints.iter() {
            let trim = end.trim();
            let boundary = sources[peer].source.boundaries()[sources[peer].contact_boundary];
            let endpoint = graph
                .curve_endpoints()
                .get(end.endpoint())
                .ok_or(ContactPlanGap::RelationBinding)?;
            let SectionCurveEndpointTopology::Trim {
                source_parameters, ..
            } = endpoint.topology()
            else {
                return Err(ContactPlanGap::RelationBinding);
            };
            if trim.operand() != peer
                || trim.face().raw() != boundary.cap_face()
                || trim.loop_id().raw() != boundary.cap_loop()
                || trim.fin().raw() != boundary.cap_fin()
                || trim.source_parameter().edge().raw() != boundary.edge()
                || source_parameters[peer].as_ref() != Some(trim.source_parameter())
            {
                return Err(ContactPlanGap::RelationBinding);
            }
        }
        let aligned = branch_and_source_parameters_align(store, branch, sources, owner)?;
        let inside_start_endpoint = if aligned {
            endpoints[0].endpoint()
        } else {
            endpoints[1].endpoint()
        };
        let inside_piece = roots[owner]
            .iter()
            .position(|root| root.endpoint == inside_start_endpoint)
            .ok_or(ContactPlanGap::RelationBinding)?;
        bound[owner] = Some(BoundRing {
            fragment: fragment_index,
            circle,
            roots: roots[owner],
            inside_piece,
        });
    }
    let [Some(first), Some(second)] = bound else {
        return Err(ContactPlanGap::RelationBinding);
    };
    validate_periodic_gaps(graph, sources, [first.fragment, second.fragment])?;
    Ok([first, second])
}

fn validate_branch_support(
    branch: &crate::SectionBranch,
    source: Circle,
) -> Result<(), ContactPlanGap> {
    let SectionCarrier::Circle {
        center,
        normal,
        radius,
        ..
    } = branch.carrier()
    else {
        return Err(ContactPlanGap::RelationBinding);
    };
    if branch.topology() != SectionBranchTopology::Closed
        || !branch.range().is_finite()
        || branch.range().width() != PERIOD
        || center != source.frame().origin()
        || radius.to_bits() != source.radius().to_bits()
        || !vectors_are_exactly_parallel(normal, source.frame().z())
    {
        return Err(ContactPlanGap::RelationBinding);
    }
    Ok(())
}

fn bind_dual_roots(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<[[BoundRoot; 2]; 2], ContactPlanGap> {
    let mut roots = [[None; 2]; 2];
    for (endpoint_index, endpoint) in graph.curve_endpoints().iter().enumerate() {
        let SectionCurveEndpointTopology::Trim {
            sites,
            source_parameters,
        } = endpoint.topology()
        else {
            return Err(ContactPlanGap::RelationBinding);
        };
        for operand in 0..2 {
            let boundary = sources[operand].source.boundaries()[sources[operand].contact_boundary];
            let (SectionSite::EdgeInterior(site_edge), Some(parameter), Some(observed)) = (
                &sites[operand],
                source_parameters[operand].as_ref(),
                endpoint.edge_parameters()[operand],
            ) else {
                return Err(ContactPlanGap::RelationBinding);
            };
            let ordinal = parameter.root_ordinal();
            let scalar = parameter.root_parameter();
            let enclosure = parameter.root_parameter_enclosure();
            if site_edge.raw() != boundary.edge()
                || parameter.edge().raw() != boundary.edge()
                || ordinal >= 2
                || !scalar.is_finite()
                || scalar <= 0.0
                || scalar >= PERIOD
                || !enclosure.lo().is_finite()
                || !enclosure.hi().is_finite()
                || enclosure.lo() > scalar
                || enclosure.hi() < scalar
                || !observed.lo().is_finite()
                || !observed.hi().is_finite()
                || observed.lo() > scalar
                || observed.hi() < scalar
                || roots[operand][ordinal]
                    .replace(BoundRoot {
                        endpoint: endpoint_index,
                        parameter: scalar,
                        enclosure: [enclosure.lo(), enclosure.hi()],
                    })
                    .is_some()
            {
                return Err(ContactPlanGap::RelationBinding);
            }
        }
    }
    let [[Some(a0), Some(a1)], [Some(b0), Some(b1)]] = roots else {
        return Err(ContactPlanGap::RelationBinding);
    };
    if a0.endpoint == a1.endpoint
        || b0.endpoint == b1.endpoint
        || a0.parameter >= a1.parameter
        || b0.parameter >= b1.parameter
        || a0.enclosure[1] >= a1.enclosure[0]
        || b0.enclosure[1] >= b1.enclosure[0]
    {
        return Err(ContactPlanGap::RelationBinding);
    }
    Ok([[a0, a1], [b0, b1]])
}

fn branch_and_source_parameters_align(
    store: &Store,
    branch: &crate::SectionBranch,
    sources: &[ContactSource<'_>; 2],
    owner: usize,
) -> Result<bool, ContactPlanGap> {
    let SectionUvCurve::Line(branch_line) = branch.pcurves()[owner] else {
        return Err(ContactPlanGap::RelationBinding);
    };
    let boundary = sources[owner].source.boundaries()[sources[owner].contact_boundary];
    let fin = store
        .get(boundary.side_fin())
        .map_err(|_| ContactPlanGap::SourceTopology)?;
    let use_ = fin.pcurve().ok_or(ContactPlanGap::SourceTopology)?;
    let Curve2dGeom::Line(source_line) = store
        .pcurve(use_.curve())
        .map_err(|_| ContactPlanGap::SourceTopology)?
    else {
        return Err(ContactPlanGap::SourceTopology);
    };
    let factors = [
        branch_line.direction().x,
        source_line.dir().x,
        use_.edge_to_pcurve().scale(),
    ];
    if branch_line.direction().y != 0.0
        || source_line.dir().y != 0.0
        || factors
            .into_iter()
            .any(|value| !value.is_finite() || value == 0.0)
    {
        return Err(ContactPlanGap::RelationBinding);
    }
    Ok(factors
        .into_iter()
        .filter(|value| value.is_sign_negative())
        .count()
        % 2
        == 0)
}

fn validate_contact_gaps(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
) -> Result<(), ContactPlanGap> {
    let contact = sources.map(|source| source.source.boundaries()[source.contact_boundary]);
    let expected = [
        (
            GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY,
            [sources[0].source.side_face(), contact[1].cap_face()],
        ),
        (
            GAP_CLOSED_CONIC_COINCIDENT_BOUNDARY,
            [contact[0].cap_face(), sources[1].source.side_face()],
        ),
        (
            GAP_COINCIDENT_FACE_PAIR,
            [contact[0].cap_face(), contact[1].cap_face()],
        ),
        (
            GAP_PAIR_UNRESOLVED,
            [sources[0].source.side_face(), sources[1].source.side_face()],
        ),
    ];
    if graph.gaps().len() != expected.len() {
        return Err(ContactPlanGap::RelationBinding);
    }
    let mut consumed = [false; 4];
    for gap in graph.gaps() {
        let actual = gap.faces();
        let Some(index) = expected.iter().enumerate().position(|(index, candidate)| {
            !consumed[index]
                && gap.reason() == candidate.0
                && actual.len() == 2
                && ((actual[0].raw() == candidate.1[0] && actual[1].raw() == candidate.1[1])
                    || (actual[0].raw() == candidate.1[1] && actual[1].raw() == candidate.1[0]))
        }) else {
            return Err(ContactPlanGap::RelationBinding);
        };
        consumed[index] = true;
    }
    consumed
        .into_iter()
        .all(|value| value)
        .then_some(())
        .ok_or(ContactPlanGap::RelationBinding)
}

fn validate_periodic_gaps(
    graph: &BodySectionGraph,
    sources: &[ContactSource<'_>; 2],
    fragments: [usize; 2],
) -> Result<(), ContactPlanGap> {
    if graph.periodic_face_embeddings().len() != 2 {
        return Err(ContactPlanGap::RelationBinding);
    }
    let mut seen = [false; 2];
    for evidence in graph.periodic_face_embeddings() {
        let SectionPeriodicFaceEmbeddingEvidence::Indeterminate {
            operand,
            face,
            gap:
                SectionPeriodicEmbeddingGap::BoundaryTerminalUnavailable {
                    component,
                    fragment,
                    end,
                },
        } = evidence
        else {
            return Err(ContactPlanGap::RelationBinding);
        };
        if *operand >= 2
            || seen[*operand]
            || face.raw() != sources[*operand].source.side_face()
            || *fragment != fragments[*operand]
            || *component != 0
            || *end > 1
        {
            return Err(ContactPlanGap::RelationBinding);
        }
        seen[*operand] = true;
    }
    seen.into_iter()
        .all(|value| value)
        .then_some(())
        .ok_or(ContactPlanGap::RelationBinding)
}

fn build_strict_secant_shell(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    rings: &[BoundRing; 2],
) -> Result<AnalyticShellInput, ContactPlanGap> {
    let vertices = rings[0]
        .roots
        .iter()
        .enumerate()
        .map(|(ordinal, root)| {
            AnalyticShellVertex::new(
                AnalyticVertexKey::new(ordinal as u64),
                rings[0].circle.eval(root.parameter),
            )
        })
        .collect::<Vec<_>>();
    let pieces = [ring_pieces(rings, 0)?, ring_pieces(rings, 1)?];
    if pieces
        .iter()
        .flatten()
        .any(|piece| !piece.range.is_finite() || piece.range.lo >= piece.range.hi)
    {
        return Err(ContactPlanGap::ArithmeticGuard);
    }
    let edges = pieces
        .iter()
        .enumerate()
        .flat_map(|(operand, pieces)| {
            pieces.iter().enumerate().map(move |(piece, spec)| {
                AnalyticShellEdge::new(
                    spec.key,
                    spec.vertices,
                    AnalyticShellCurve::Circle(rings[operand].circle),
                    spec.range,
                )
                .with_split_lineage(
                    EntityRef::Edge(
                        sources[operand].source.boundaries()[sources[operand].contact_boundary]
                            .edge(),
                    ),
                    if piece == 0 {
                        AnalyticEdgeSplitPiece::First
                    } else {
                        AnalyticEdgeSplitPiece::Second
                    },
                )
            })
        })
        .collect::<Vec<_>>();

    let charts = [
        prepare_side_chart(store, sources, &pieces, 0)?,
        prepare_side_chart(store, sources, &pieces, 1)?,
    ];
    let closed_edges = (0..2)
        .map(|operand| {
            let boundary = sources[operand].source.boundaries()[sources[operand].far_boundary];
            let circle = source_circle(store, boundary)?;
            Ok(AnalyticShellClosedEdge::new(
                AnalyticEdgeKey::new(operand as u64),
                AnalyticShellCurve::Circle(circle),
                charts[operand].domain.u,
            )
            .with_source(EntityRef::Edge(boundary.edge())))
        })
        .collect::<Result<Vec<_>, ContactPlanGap>>()?;

    let mut faces = Vec::with_capacity(6);
    for operand in 0..2 {
        faces.push(side_face(store, sources, &pieces, &charts, operand)?);
    }
    for operand in 0..2 {
        faces.push(far_cap_face(store, sources, operand)?);
    }
    for operand in 0..2 {
        faces.push(contact_crescent_face(
            store, sources, rings, &pieces, operand,
        )?);
    }
    Ok(AnalyticShellInput::new(vertices, edges, faces).with_closed_edges(closed_edges))
}

fn ring_pieces(rings: &[BoundRing; 2], operand: usize) -> Result<[RingPiece; 2], ContactPlanGap> {
    let vertex = |endpoint: usize| {
        rings[0]
            .roots
            .iter()
            .position(|root| root.endpoint == endpoint)
            .map(|ordinal| AnalyticVertexKey::new(ordinal as u64))
            .ok_or(ContactPlanGap::RelationBinding)
    };
    let roots = rings[operand].roots;
    let wrap_end = roots[0].parameter + PERIOD;
    if !wrap_end.is_finite() {
        return Err(ContactPlanGap::ArithmeticGuard);
    }
    Ok([
        RingPiece {
            key: AnalyticEdgeKey::new((2 + 2 * operand) as u64),
            vertices: [vertex(roots[0].endpoint)?, vertex(roots[1].endpoint)?],
            range: ParamRange::new(roots[0].parameter, roots[1].parameter),
        },
        RingPiece {
            key: AnalyticEdgeKey::new((3 + 2 * operand) as u64),
            vertices: [vertex(roots[1].endpoint)?, vertex(roots[0].endpoint)?],
            range: ParamRange::new(roots[1].parameter, wrap_end),
        },
    ])
}

fn prepare_side_chart(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    pieces: &[[RingPiece; 2]; 2],
    operand: usize,
) -> Result<PreparedSideChart, ContactPlanGap> {
    let source = sources[operand];
    let face = source_face_data(store, source.source.side_face())?;
    let source_domain = face.domain().ok_or(ContactPlanGap::SourceTopology)?;
    let surface = AnalyticShellSurface::Cylinder(source.source.cylinder());
    let contact = source.source.boundaries()[source.contact_boundary];
    let base = source_pcurve(store, contact.side_fin(), false)?;
    let uses = pieces[operand].map(|piece| (base, piece.range));
    let window =
        periodic_chart::select_common_periodic_window_for_uses(surface, source_domain.u, &uses)
            .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let first = periodic_chart::normalize_intrinsic_periodic_pcurve_chart(
        surface,
        window,
        base,
        pieces[operand][0].range,
    )
    .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let second = periodic_chart::normalize_intrinsic_periodic_pcurve_chart(
        surface,
        window,
        base,
        pieces[operand][1].range,
    )
    .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let far = source.source.boundaries()[source.far_boundary];
    let far_pcurve = periodic_chart::shift_endpoint_free_intrinsic_periodic_ring(
        surface,
        source_pcurve(store, far.side_fin(), true)?,
        window,
    )
    .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    Ok(PreparedSideChart {
        surface,
        domain: side_domain_with_window(source_domain, window)?,
        far_pcurve,
        piece_pcurves: [first, second],
    })
}

fn side_face(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    pieces: &[[RingPiece; 2]; 2],
    charts: &[PreparedSideChart; 2],
    operand: usize,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let source = sources[operand];
    let face = source_face_data(store, source.source.side_face())?;
    let far = source.source.boundaries()[source.far_boundary];
    let contact = source.source.boundaries()[source.contact_boundary];
    let far_loop = AnalyticShellLoop::new(vec![AnalyticShellFin::new(
        AnalyticEdgeKey::new(operand as u64),
        source_fin_sense(store, far.side_fin())?,
        charts[operand].far_pcurve,
    )]);
    let side_sense = source_fin_sense(store, contact.side_fin())?;
    let order = if side_sense == Sense::Forward {
        [(0, Sense::Forward), (1, Sense::Forward)]
    } else {
        [(1, Sense::Reversed), (0, Sense::Reversed)]
    };
    let contact_loop = AnalyticShellLoop::new(
        order
            .into_iter()
            .map(|(piece, sense)| {
                Ok(AnalyticShellFin::new(
                    pieces[operand][piece].key,
                    sense,
                    charts[operand].piece_pcurves[piece],
                ))
            })
            .collect::<Result<Vec<_>, ContactPlanGap>>()?,
    );
    Ok(AnalyticShellFace::new(
        AnalyticFaceKey::new(operand as u64),
        charts[operand].surface,
        face.sense(),
        charts[operand].domain,
        vec![far_loop, contact_loop],
    )
    .with_source(EntityRef::Face(source.source.side_face())))
}

fn far_cap_face(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    operand: usize,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let boundary = sources[operand].source.boundaries()[sources[operand].far_boundary];
    let face = source_face_data(store, boundary.cap_face())?;
    Ok(AnalyticShellFace::new(
        AnalyticFaceKey::new((2 + operand) as u64),
        AnalyticShellSurface::Plane(source_plane(store, boundary.cap_face())?),
        face.sense(),
        face.domain().ok_or(ContactPlanGap::SourceTopology)?,
        vec![AnalyticShellLoop::new(vec![AnalyticShellFin::new(
            AnalyticEdgeKey::new(operand as u64),
            source_fin_sense(store, boundary.cap_fin())?,
            source_pcurve(store, boundary.cap_fin(), true)?,
        )])],
    )
    .with_source(EntityRef::Face(boundary.cap_face())))
}

fn contact_crescent_face(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    rings: &[BoundRing; 2],
    pieces: &[[RingPiece; 2]; 2],
    operand: usize,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let peer = 1 - operand;
    let boundary = sources[operand].source.boundaries()[sources[operand].contact_boundary];
    let face = source_face_data(store, boundary.cap_face())?;
    let plane = source_plane(store, boundary.cap_face())?;
    let outside = 1 - rings[operand].inside_piece;
    let inside = rings[peer].inside_piece;
    let outside_piece = pieces[operand][outside];
    let inside_piece = pieces[peer][inside];
    let outside_sense = source_fin_sense(store, boundary.cap_fin())?;
    let [outside_tail, outside_head] = directed_vertices(outside_piece, outside_sense);
    let inside_sense = if inside_piece.vertices == [outside_head, outside_tail] {
        Sense::Forward
    } else if [inside_piece.vertices[1], inside_piece.vertices[0]] == [outside_head, outside_tail] {
        Sense::Reversed
    } else {
        return Err(ContactPlanGap::RelationBinding);
    };
    Ok(AnalyticShellFace::new(
        AnalyticFaceKey::new((4 + operand) as u64),
        AnalyticShellSurface::Plane(plane),
        face.sense(),
        face.domain().ok_or(ContactPlanGap::SourceTopology)?,
        vec![AnalyticShellLoop::new(vec![
            AnalyticShellFin::new(
                outside_piece.key,
                outside_sense,
                source_pcurve(store, boundary.cap_fin(), false)?,
            ),
            AnalyticShellFin::new(
                inside_piece.key,
                inside_sense,
                projected_circle_pcurve(plane, rings[peer].circle)?,
            ),
        ])],
    )
    .with_source(EntityRef::Face(boundary.cap_face())))
}

fn directed_vertices(piece: RingPiece, sense: Sense) -> [AnalyticVertexKey; 2] {
    if sense == Sense::Forward {
        piece.vertices
    } else {
        [piece.vertices[1], piece.vertices[0]]
    }
}

fn side_domain_with_window(
    domain: FaceDomain,
    window: ParamRange,
) -> Result<FaceDomain, ContactPlanGap> {
    FaceDomain::new(window, domain.v).map_err(|_| ContactPlanGap::ArithmeticGuard)
}
