//! Finite-window occupancy and non-wrapping spans for exact skew cylinders.
//! Wall-time budget: less than 10 seconds for the focused perpendicular oracle.

use kcore::error::CapabilityId;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSnapshot, LimitSpec, OperationContext,
    ResourceKind, SessionPolicy, StageId,
};
use kcore::proof::IncompleteCause;
use kcore::tolerance::Tolerances;
use kgeom::curve::Curve;
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgraph::{Curve2dDescriptor, CurveDescriptor, GeometryGraph, SkewCylinderSheet};
use kops::intersect::{
    ContactKind, GraphSurfaceIntersectionError, IntersectionBranchEndpointEvent,
    IntersectionBranchEndpointProof, IntersectionBranchTopology, IntersectionBranchVertexEvent,
    SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK, SKEW_CYLINDER_AXIAL_CLIP_WORK,
    SKEW_CYLINDER_CLIPPED_BRANCH_TOPOLOGY, SKEW_CYLINDER_CLIPPED_TOPOLOGY_INCOMPLETE,
    SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH, SKEW_CYLINDER_OPEN_SPAN_WORK,
    SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER, SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
    SKEW_CYLINDER_TWO_SHEET_WORK, SkewCylinderAxialBoundaryProof, SkewCylinderAxialRelationProof,
    SkewCylinderRootInsideSideProof, SurfaceIntersectionCurve, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_with_context,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn cylinder_window(height: ParamRange) -> [ParamRange; 2] {
    [range(0.0, core::f64::consts::TAU), height]
}

fn perpendicular_pair() -> [Cylinder; 2] {
    let frame = Frame::world();
    [
        Cylinder::new(frame, 1.0).unwrap(),
        Cylinder::new(
            Frame::new(frame.origin(), frame.x(), frame.y()).unwrap(),
            2.0,
        )
        .unwrap(),
    ]
}

fn graph_pair(cylinders: [Cylinder; 2]) -> (GeometryGraph, [kgraph::SurfaceHandle; 2]) {
    let mut graph = GeometryGraph::new();
    let handles = cylinders.map(|cylinder| graph.insert_surface(cylinder).unwrap());
    (graph, handles)
}

fn assert_no_skew_branches(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.points.is_empty());
    assert!(result.raw.curves.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(result.branch_graph.vertices.is_empty());
    assert!(result.branch_graph.edges.is_empty());
    assert!(
        result.skew_cylinder_strict_discriminant_miss().is_none(),
        "finite-window occupancy must not mint an infinite-support miss witness"
    );
    assert!(
        result
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );
}

fn assert_single_typed_gap(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    code: DiagnosticCode,
    stage: StageId,
    capability: CapabilityId,
) {
    assert_no_skew_branches(result, sources);
    assert!(!result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert_eq!(result.raw.incomplete_evidence().len(), 1);
    let evidence = result.raw.incomplete_evidence()[0];
    assert_eq!(evidence.code, code);
    assert_eq!(evidence.stage, stage);
    assert_eq!(
        evidence.cause,
        IncompleteCause::ProofMethodUnavailable { capability }
    );
}

fn assert_single_lower_sheet(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    source_cylinders: [Cylinder; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(result.raw.points.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(result.raw.incomplete_evidence().is_empty());
    assert_eq!(result.raw.curves.len(), 1);
    assert_eq!(result.branch_graph.edges.len(), 1);
    assert_eq!(result.branch_graph.vertices.len(), 1);
    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());

    let raw_branch = &result.raw.curves[0];
    let SurfaceIntersectionCurve::SkewCylinder(raw_carrier) = raw_branch.curve else {
        panic!("root-free retained sheet must use the procedural skew carrier");
    };
    assert_eq!(raw_carrier.sheet(), SkewCylinderSheet::Lower);

    let edge = &result.branch_graph.edges[0];
    let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
        panic!("verified retained sheet must preserve the procedural carrier");
    };
    assert_eq!(carrier, raw_carrier);
    assert_eq!(carrier.sheet(), SkewCylinderSheet::Lower);
    assert_eq!(edge.source_surfaces, sources);
    assert_eq!(edge.carrier_range, raw_branch.curve_range);
    assert_eq!(edge.topology, IntersectionBranchTopology::Closed);
    assert_eq!(edge.endpoint_vertices, [0, 0]);
    assert!(matches!(
        result.branch_graph.vertices[0].event,
        IntersectionBranchVertexEvent::PeriodSeam { .. }
    ));
    assert!(
        edge.endpoint_events
            .iter()
            .all(|event| matches!(event, IntersectionBranchEndpointEvent::PeriodSeam { .. }))
    );
    assert!(
        edge.parameter_maps
            .iter()
            .all(|map| map.scale() == 1.0 && map.offset() == 0.0)
    );

    let certificate = edge.certificate.as_skew_cylinder_two_sheet().unwrap();
    assert_eq!(certificate.carrier(), carrier);
    assert_eq!(certificate.sheet(), SkewCylinderSheet::Lower);
    assert_eq!(
        certificate.traces().map(|trace| trace.surface()),
        source_cylinders
    );
    assert_eq!(certificate.parameter_maps(), edge.parameter_maps);
    assert_eq!(
        edge.pcurves,
        certificate
            .traces()
            .map(|trace| Curve2dDescriptor::SkewCylinderBranch(trace.pcurve()))
    );
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );

    let raw_starts = [raw_branch.uv_a_start, raw_branch.uv_b_start];
    let raw_ends = [raw_branch.uv_a_end, raw_branch.uv_b_end];
    for operand in 0..2 {
        let trace = edge.pcurves[operand].as_curve();
        let map = edge.parameter_maps[operand];
        let start = trace.eval(map.map(edge.carrier_range.lo));
        let end = trace.eval(map.map(edge.carrier_range.hi));
        assert_eq!([start.x, start.y], raw_starts[operand]);
        assert_eq!([end.x, end.y], raw_ends[operand]);
    }

    // Independent perpendicular-axis oracle:
    // P_lower(u) = (cos u, sin u, -sqrt(4 - sin²u)).
    let frame = Frame::world();
    for parameter in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.25),
        edge.carrier_range.lerp(0.5),
        edge.carrier_range.lerp(0.75),
        edge.carrier_range.hi,
    ] {
        let (sine, cosine) = kcore::math::sincos(parameter);
        let expected_point = frame.origin() + frame.x() * cosine + frame.y() * sine
            - frame.z() * (4.0 - sine * sine).sqrt();
        let point = carrier.eval(parameter);
        assert!(
            point.dist(expected_point) <= certificate.tolerance(),
            "retained Lower carrier disagrees with the analytic oracle at u={parameter}"
        );
        for (operand, cylinder) in source_cylinders.iter().enumerate() {
            let uv = edge.pcurves[operand]
                .as_curve()
                .eval(edge.parameter_maps[operand].map(parameter));
            assert!(
                point.dist(cylinder.eval([uv.x, uv.y])) <= certificate.tolerance(),
                "operand {operand} pcurve does not lift to the retained carrier"
            );
        }
    }
}

fn assert_single_upper_open_span(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    source_cylinders: [Cylinder; 2],
    boundary_surfaces: [bool; 2],
) {
    const EXPECTED_LO: f64 = 2.082_769_014_844_373;
    const EXPECTED_HI: f64 = 4.200_416_292_335_213;
    const PARAMETER_TOLERANCE: f64 = 1.0e-12;

    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.is_complete(), "{:#?}", result.raw);
    assert!(!result.raw.is_proven_empty());
    assert!(result.raw.points.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(result.raw.incomplete_evidence().is_empty());
    assert_eq!(result.raw.curves.len(), 1);
    assert_eq!(result.branch_graph.edges.len(), 1);
    assert_eq!(result.branch_graph.vertices.len(), 2);
    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());
    assert!(
        result
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );

    let raw_branch = &result.raw.curves[0];
    let SurfaceIntersectionCurve::SkewCylinder(raw_carrier) = raw_branch.curve else {
        panic!("bounded retained sheet must use the procedural skew carrier");
    };
    assert_eq!(raw_carrier.sheet(), SkewCylinderSheet::Upper);
    assert_eq!(raw_branch.kind, ContactKind::Transverse);

    let edge = &result.branch_graph.edges[0];
    let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
        panic!("verified bounded sheet must preserve the procedural carrier");
    };
    assert_eq!(carrier, raw_carrier);
    assert_eq!(carrier.sheet(), SkewCylinderSheet::Upper);
    assert_eq!(edge.source_surfaces, sources);
    assert_eq!(edge.carrier_range, raw_branch.curve_range);
    assert_eq!(edge.kind, ContactKind::Transverse);
    assert_eq!(edge.topology, IntersectionBranchTopology::Open);
    assert_eq!(edge.endpoint_vertices, [0, 1]);
    assert!(
        (edge.carrier_range.lo - EXPECTED_LO).abs() <= PARAMETER_TOLERANCE,
        "unexpected low root enclosure: {:?}",
        edge.carrier_range
    );
    assert!(
        (edge.carrier_range.hi - EXPECTED_HI).abs() <= PARAMETER_TOLERANCE,
        "unexpected high root enclosure: {:?}",
        edge.carrier_range
    );
    assert!(
        edge.carrier_range.lo > core::f64::consts::FRAC_PI_2
            && edge.carrier_range.hi < 3.0 * core::f64::consts::FRAC_PI_2
            && edge.carrier_range.width() < core::f64::consts::TAU,
        "the first clipped-span slice must remain in one non-wrapping chart"
    );
    assert_eq!(
        edge.endpoint_events,
        [IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
            surfaces: boundary_surfaces,
        }; 2]
    );
    assert!(
        edge.parameter_maps
            .iter()
            .all(|map| map.scale() == 1.0 && map.offset() == 0.0)
    );
    assert!(edge.pcurves.iter().all(|pcurve| matches!(
        pcurve,
        Curve2dDescriptor::SkewCylinderBranch(trace)
            if trace.sheet() == SkewCylinderSheet::Upper
    )));

    assert!(
        edge.certificate.as_skew_cylinder_two_sheet().is_none(),
        "a clipped subrange must not reuse the full-cycle containment certificate"
    );
    let certificate = edge
        .certificate
        .as_skew_cylinder_open_span()
        .expect("the clipped branch must retain its independent subrange proof");
    assert_eq!(certificate.carrier(), carrier);
    assert_eq!(certificate.carrier_range(), edge.carrier_range);
    assert_eq!(certificate.sheet(), SkewCylinderSheet::Upper);
    assert_eq!(
        certificate.traces().map(|trace| trace.surface()),
        source_cylinders
    );
    assert_eq!(certificate.parameter_maps(), edge.parameter_maps);
    assert_eq!(
        edge.pcurves,
        certificate
            .traces()
            .map(|trace| Curve2dDescriptor::SkewCylinderBranch(trace.pcurve()))
    );
    let tolerance = certificate.tolerance();
    assert!(tolerance.is_finite() && tolerance > 0.0);
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound.is_finite() && bound >= 0.0 && bound <= tolerance)
    );

    assert_upper_open_span_endpoint_proofs(result, boundary_surfaces, tolerance);
    assert!(
        result.branch_graph.vertices[0]
            .point
            .dist(result.branch_graph.vertices[1].point)
            > 1.0,
        "the two axial-bound roots must remain distinct physical vertices"
    );

    assert_upper_open_span_lifts(result, source_cylinders, tolerance);
}

fn assert_upper_open_span_endpoint_proofs(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    boundary_surfaces: [bool; 2],
    tolerance: f64,
) {
    let raw_branch = &result.raw.curves[0];
    let edge = &result.branch_graph.edges[0];
    let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
        panic!("verified bounded sheet must preserve the procedural carrier");
    };
    let raw_parameters = [
        [raw_branch.uv_a_start, raw_branch.uv_b_start],
        [raw_branch.uv_a_end, raw_branch.uv_b_end],
    ];
    for endpoint_slot in 0..2 {
        let vertex = &result.branch_graph.vertices[edge.endpoint_vertices[endpoint_slot]];
        assert_eq!(
            vertex.event,
            IntersectionBranchVertexEvent::BoundaryEndpoint {
                surfaces: boundary_surfaces,
            }
        );
        assert_eq!(vertex.surface_parameters, raw_parameters[endpoint_slot]);
        let parameter = if endpoint_slot == 0 {
            edge.carrier_range.lo
        } else {
            edge.carrier_range.hi
        };
        let Some(IntersectionBranchEndpointProof::SkewCylinderAxialRoot(proof)) =
            edge.endpoint_proofs[endpoint_slot]
        else {
            panic!("bounded skew endpoint must retain its exact axial root proof");
        };
        assert_eq!(
            proof.source_operand,
            boundary_surfaces.iter().position(|flag| *flag).unwrap()
        );
        assert_eq!(proof.boundary, SkewCylinderAxialBoundaryProof::Lower);
        assert_eq!(proof.bound, 1.8);
        assert_eq!(proof.sheet, SkewCylinderSheet::Upper);
        assert!(proof.cyclic_ordinal < 4);
        assert!(proof.half_angle_bracket[0] <= proof.half_angle_bracket[1]);
        assert!(proof.half_angle_bracket.into_iter().all(f64::is_finite));
        assert_eq!(proof.inside_parameter, parameter);
        if endpoint_slot == 0 {
            assert_eq!(proof.before, SkewCylinderAxialRelationProof::Below);
            assert_eq!(proof.after, SkewCylinderAxialRelationProof::Above);
            assert_eq!(proof.inside_side, SkewCylinderRootInsideSideProof::After);
        } else {
            assert_eq!(proof.before, SkewCylinderAxialRelationProof::Above);
            assert_eq!(proof.after, SkewCylinderAxialRelationProof::Below);
            assert_eq!(proof.inside_side, SkewCylinderRootInsideSideProof::Before);
        }
        assert!(vertex.point.dist(carrier.eval(parameter)) <= tolerance);
    }
}

fn assert_upper_open_span_lifts(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    source_cylinders: [Cylinder; 2],
    tolerance: f64,
) {
    let edge = &result.branch_graph.edges[0];
    let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
        panic!("verified bounded sheet must preserve the procedural carrier");
    };

    // Independent perpendicular-axis oracle:
    // P_upper(u) = (cos u, sin u, sqrt(4 - sin²u)).
    let frame = Frame::world();
    for parameter in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.25),
        edge.carrier_range.lerp(0.5),
        edge.carrier_range.lerp(0.75),
        edge.carrier_range.hi,
    ] {
        let (sine, cosine) = kcore::math::sincos(parameter);
        let expected_point = frame.origin()
            + frame.x() * cosine
            + frame.y() * sine
            + frame.z() * (4.0 - sine * sine).sqrt();
        let point = carrier.eval(parameter);
        assert!(
            point.dist(expected_point) <= tolerance,
            "retained Upper span disagrees with the analytic oracle at u={parameter}"
        );
        for (operand, cylinder) in source_cylinders.iter().enumerate() {
            let uv = edge.pcurves[operand]
                .as_curve()
                .eval(edge.parameter_maps[operand].map(parameter));
            assert!(
                point.dist(cylinder.eval([uv.x, uv.y])) <= tolerance,
                "operand {operand} pcurve does not lift to the bounded carrier"
            );
        }
    }
}

fn assert_four_upper_open_spans(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    source_cylinders: [Cylinder; 2],
    boundary_surfaces: [bool; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.is_complete(), "{:#?}", result.raw);
    assert!(result.raw.points.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(result.raw.incomplete_evidence().is_empty());
    assert_eq!(result.raw.curves.len(), 4);
    assert_eq!(result.branch_graph.edges.len(), 4);
    assert_eq!(result.branch_graph.vertices.len(), 8);

    let mut endpoint_vertices = Vec::with_capacity(8);
    for (ordinal, (raw_branch, edge)) in result
        .raw
        .curves
        .iter()
        .zip(&result.branch_graph.edges)
        .enumerate()
    {
        let SurfaceIntersectionCurve::SkewCylinder(raw_carrier) = raw_branch.curve else {
            panic!("bounded retained sheet must use the procedural skew carrier");
        };
        let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
            panic!("verified bounded sheet must preserve the procedural carrier");
        };
        assert_eq!(carrier, raw_carrier);
        assert_eq!(carrier.sheet(), SkewCylinderSheet::Upper);
        assert_eq!(edge.source_surfaces, sources);
        assert_eq!(edge.carrier_range, raw_branch.curve_range);
        assert_eq!(edge.topology, IntersectionBranchTopology::Open);
        assert!(0.0 < edge.carrier_range.lo);
        assert!(edge.carrier_range.hi < core::f64::consts::TAU);
        assert!(0.0 < edge.carrier_range.width());
        if ordinal > 0 {
            assert!(
                result.branch_graph.edges[ordinal - 1].carrier_range.hi < edge.carrier_range.lo
            );
        }
        assert_eq!(
            edge.endpoint_events,
            [IntersectionBranchEndpointEvent::SurfaceWindowBoundary {
                surfaces: boundary_surfaces,
            }; 2]
        );
        endpoint_vertices.extend(edge.endpoint_vertices);

        let certificate = edge
            .certificate
            .as_skew_cylinder_open_span()
            .expect("each clipped branch must retain an independent subrange proof");
        assert_eq!(certificate.carrier(), carrier);
        assert_eq!(certificate.carrier_range(), edge.carrier_range);
        assert_eq!(certificate.sheet(), SkewCylinderSheet::Upper);
        assert_eq!(
            certificate.traces().map(|trace| trace.surface()),
            source_cylinders
        );
        let tolerance = certificate.tolerance();

        for endpoint_slot in 0..2 {
            let Some(IntersectionBranchEndpointProof::SkewCylinderAxialRoot(proof)) =
                edge.endpoint_proofs[endpoint_slot]
            else {
                panic!("bounded skew endpoint must retain its exact axial root proof");
            };
            let vertex = &result.branch_graph.vertices[edge.endpoint_vertices[endpoint_slot]];
            let inside_parameter = if endpoint_slot == 0 {
                edge.carrier_range.lo
            } else {
                edge.carrier_range.hi
            };
            let lower_boundary = (ordinal + endpoint_slot) % 2 == 1;
            assert_eq!(
                proof.source_operand,
                boundary_surfaces.iter().position(|flag| *flag).unwrap()
            );
            assert_eq!(proof.cyclic_ordinal, ordinal);
            assert_eq!(proof.sheet, SkewCylinderSheet::Upper);
            assert_eq!(
                (proof.boundary, proof.bound),
                if lower_boundary {
                    (SkewCylinderAxialBoundaryProof::Lower, 1.8)
                } else {
                    (SkewCylinderAxialBoundaryProof::Upper, 1.9)
                }
            );
            assert_eq!(proof.inside_parameter, inside_parameter);
            assert_eq!(
                proof.inside_side,
                if endpoint_slot == 0 {
                    SkewCylinderRootInsideSideProof::After
                } else {
                    SkewCylinderRootInsideSideProof::Before
                }
            );
            assert_eq!(
                if endpoint_slot == 0 {
                    proof.after
                } else {
                    proof.before
                },
                if lower_boundary {
                    SkewCylinderAxialRelationProof::Above
                } else {
                    SkewCylinderAxialRelationProof::Below
                }
            );
            assert!(proof.half_angle_bracket.into_iter().all(f64::is_finite));
            assert!(proof.half_angle_bracket[0] <= proof.half_angle_bracket[1]);
            assert_eq!(
                vertex.event,
                IntersectionBranchVertexEvent::BoundaryEndpoint {
                    surfaces: boundary_surfaces,
                }
            );
            assert!(vertex.point.dist(carrier.eval(inside_parameter)) <= tolerance);
        }
    }
    endpoint_vertices.sort_unstable();
    endpoint_vertices.dedup();
    assert_eq!(endpoint_vertices.len(), 8);
}

#[test]
fn root_free_axial_windows_prove_empty_without_an_infinite_support_miss() {
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(-1.0, 1.0)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let result = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();

    assert_no_skew_branches(&result, handles);
    assert!(result.raw.is_complete());
    assert!(result.raw.is_proven_empty());
    assert!(result.raw.incomplete_evidence().is_empty());
}

#[test]
fn root_free_one_sheet_window_is_complete_replay_and_swap_stable() {
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(-2.25, 0.0)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let forward = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        handles[1],
        windows[1],
        handles[0],
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert_single_lower_sheet(&forward, handles, cylinders);
    assert_single_lower_sheet(
        &reversed,
        [handles[1], handles[0]],
        [cylinders[1], cylinders[0]],
    );
    assert_eq!(reversed.raw, forward.raw.clone().swapped());

    let forward_edge = &forward.branch_graph.edges[0];
    let reversed_edge = &reversed.branch_graph.edges[0];
    assert_eq!(forward_edge.carrier, reversed_edge.carrier);
    assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
    assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
}

#[test]
fn nonperiodic_longitude_keeps_the_existing_two_sheet_gap() {
    let cylinders = perpendicular_pair();
    let windows = [
        [
            range(0.0, core::f64::consts::TAU.next_down()),
            range(-2.25, 2.25),
        ],
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let result = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();

    assert_single_typed_gap(
        &result,
        handles,
        SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
        SKEW_CYLINDER_TWO_SHEET_WORK,
        SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
    );
}

#[test]
fn nonwrapping_axial_clip_publishes_one_open_upper_span_replay_and_swap_stably() {
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(1.8, 2.1)),
        cylinder_window(range(-1.25, 0.0)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let forward = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        handles[1],
        windows[1],
        handles[0],
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert_single_upper_open_span(&forward, handles, cylinders, [true, false]);
    assert_single_upper_open_span(
        &reversed,
        [handles[1], handles[0]],
        [cylinders[1], cylinders[0]],
        [false, true],
    );
    assert_eq!(reversed.raw, forward.raw.clone().swapped());

    let forward_edge = &forward.branch_graph.edges[0];
    let reversed_edge = &reversed.branch_graph.edges[0];
    assert_eq!(forward_edge.carrier, reversed_edge.carrier);
    assert_eq!(forward_edge.carrier_range, reversed_edge.carrier_range);
    assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
    assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
    assert_eq!(
        forward_edge.certificate.residual_bounds(),
        [
            reversed_edge.certificate.residual_bounds()[1],
            reversed_edge.certificate.residual_bounds()[0],
        ]
    );
}

#[test]
fn two_axial_bounds_publish_four_nonwrapping_upper_spans() {
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(1.8, 1.9)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let forward = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        handles[1],
        windows[1],
        handles[0],
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert_four_upper_open_spans(&forward, handles, cylinders, [true, false]);
    assert_four_upper_open_spans(
        &reversed,
        [handles[1], handles[0]],
        [cylinders[1], cylinders[0]],
        [false, true],
    );
    assert_eq!(reversed.raw, forward.raw.clone().swapped());

    for (forward_edge, reversed_edge) in forward
        .branch_graph
        .edges
        .iter()
        .zip(&reversed.branch_graph.edges)
    {
        assert_eq!(forward_edge.carrier, reversed_edge.carrier);
        assert_eq!(forward_edge.carrier_range, reversed_edge.carrier_range);
        assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
        assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
        assert_eq!(
            forward_edge.certificate.residual_bounds(),
            [
                reversed_edge.certificate.residual_bounds()[1],
                reversed_edge.certificate.residual_bounds()[0],
            ]
        );
    }
}

#[test]
fn seam_wrapping_axial_clip_remains_one_typed_gap_without_partial_publication() {
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(-2.25, 2.25)),
        cylinder_window(range(0.0, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);

    let result = intersect_bounded_graph_surfaces(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        Tolerances::default(),
    )
    .unwrap();

    assert_single_typed_gap(
        &result,
        handles,
        SKEW_CYLINDER_CLIPPED_TOPOLOGY_INCOMPLETE,
        SKEW_CYLINDER_AXIAL_CLIP_WORK,
        SKEW_CYLINDER_CLIPPED_BRANCH_TOPOLOGY,
    );
}

fn observed_work(report: &kcore::operation::OperationReport, stage: StageId) -> u64 {
    report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .map_or(0, |usage| usage.consumed)
}

#[test]
fn axial_clip_work_has_exact_n_and_atomic_n_minus_one_boundary() {
    assert_eq!(
        SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK, 256,
        "four exact axial-bound queries must remain one 4×64 atomic unit"
    );
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(-1.0, 1.0)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_AXIAL_CLIP_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &exact_context,
    );
    assert!(exact.result().unwrap().raw.is_proven_empty());
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_AXIAL_CLIP_WORK),
        SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_AXIAL_CLIP_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_AXIAL_CLIP_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK,
        allowed: SKEW_CYLINDER_AXIAL_CLIP_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_AXIAL_CLIP_WORK),
        0,
        "the rejected four-query debit must not consume a prefix"
    );
}

#[test]
fn open_span_certificate_work_has_exact_n_and_atomic_n_minus_one_boundary() {
    assert_eq!(
        SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH, 256,
        "one independently certified open branch must remain one atomic 256-work unit"
    );
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(1.8, 2.1)),
        cylinder_window(range(-1.25, 0.0)),
    ];
    let (graph, handles) = graph_pair(cylinders);
    let graph_counts = (
        graph.surface_count(),
        graph.curve_count(),
        graph.curve2d_count(),
    );
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_OPEN_SPAN_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &exact_context,
    );
    let exact_result = exact.result();
    let exact_result = exact_result.as_ref().unwrap();
    assert!(exact_result.raw.is_complete());
    assert_eq!(exact_result.raw.curves.len(), 1);
    assert_eq!(exact_result.branch_graph.edges.len(), 1);
    assert_eq!(exact_result.branch_graph.vertices.len(), 2);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_OPEN_SPAN_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH,
        allowed: SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0,
        "a rejected branch-certificate debit must not consume or expose a partial span"
    );
    assert_eq!(
        (
            graph.surface_count(),
            graph.curve_count(),
            graph.curve2d_count(),
        ),
        graph_counts,
        "N-1 refusal must leave the source graph untouched"
    );
}

#[test]
fn four_open_span_certificates_are_one_atomic_work_debit() {
    let required_work = 4 * SKEW_CYLINDER_OPEN_SPAN_EXACT_WORK_PER_BRANCH;
    let cylinders = perpendicular_pair();
    let windows = [
        cylinder_window(range(1.8, 1.9)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, handles) = graph_pair(cylinders);
    let graph_counts = (
        graph.surface_count(),
        graph.curve_count(),
        graph.curve2d_count(),
    );
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_OPEN_SPAN_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        required_work,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &exact_context,
    );
    let exact_result = exact.result();
    let exact_result = exact_result.as_ref().unwrap();
    assert!(exact_result.raw.is_complete());
    assert_eq!(exact_result.raw.curves.len(), 4);
    assert_eq!(exact_result.branch_graph.edges.len(), 4);
    assert_eq!(exact_result.branch_graph.vertices.len(), 8);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        required_work
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_OPEN_SPAN_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        required_work - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        handles[0],
        windows[0],
        handles[1],
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: required_work,
        allowed: required_work - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        0,
        "the rejected four-branch debit must not consume a prefix"
    );
    assert_eq!(
        (
            graph.surface_count(),
            graph.curve_count(),
            graph.curve2d_count(),
        ),
        graph_counts,
        "N-1 refusal must not publish a prefix or mutate the source graph"
    );
}
