//! Verified graph admission for exact parallel rulings and skew discriminants.
//! Wall-time budget: less than 10 seconds for the focused analytic matrix.

use kcore::error::CapabilityId;
use kcore::operation::{
    AccountingMode, BudgetPlan, DiagnosticCode, LimitSnapshot, LimitSpec, OperationContext,
    OperationScope, ResourceKind, SessionPolicy,
};
use kcore::proof::IncompleteCause;
use kcore::tolerance::Tolerances;
use kgeom::curve::{Curve, Line};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Surface};
use kgeom::vec::{Point3, Vec3};
use kgraph::{
    Curve2dDescriptor, CurveDescriptor, GeometryGraph, IntersectionCertificateError,
    SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
    SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
    SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK,
    SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK, SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
    SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK, SkewCylinderAxialBoundary, SkewCylinderSheet,
};
use kops::intersect::{
    ContactKind, GraphSurfaceBudgetProfile, GraphSurfaceIntersectionError,
    IntersectionBranchEndpointEvent, IntersectionBranchTopology, IntersectionBranchVertexEvent,
    IntersectionError, SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION, SKEW_CYLINDER_DISCRIMINANT_WORK,
    SKEW_CYLINDER_OPEN_SPAN_WORK, SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
    SKEW_CYLINDER_TWO_SHEET_EXACT_WORK, SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
    SKEW_CYLINDER_TWO_SHEET_WORK, SurfaceIntersectionCurve, SurfaceSurfaceCurve,
    SurfaceSurfaceIntersections, intersect_bounded_graph_surfaces,
    intersect_bounded_graph_surfaces_in_scope, intersect_bounded_graph_surfaces_with_context,
    persist_verified_graph_surface_intersections,
};

fn range(lo: f64, hi: f64) -> ParamRange {
    ParamRange::new(lo, hi)
}

fn cylinder_window(height: ParamRange) -> [ParamRange; 2] {
    [range(0.0, core::f64::consts::TAU), height]
}

fn graph_pair(
    first: Cylinder,
    second: Cylinder,
) -> (GeometryGraph, kgraph::SurfaceHandle, kgraph::SurfaceHandle) {
    let mut graph = GeometryGraph::new();
    let first_handle = graph.insert_surface(first).unwrap();
    let second_handle = graph.insert_surface(second).unwrap();
    (graph, first_handle, second_handle)
}

fn perpendicular_axis_pair(frame: Frame, offset: f64, second_radius: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() + frame.y() * offset, frame.x(), frame.y()).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn touching_body_axis_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap();
    let second_center = frame.origin() + frame.y() * 0.125;
    let second = Cylinder::new(
        Frame::new(second_center - frame.x() * 0.5, frame.x(), frame.y()).unwrap(),
        0.375,
    )
    .unwrap();
    [first, second]
}

fn seam_touching_body_axis_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap();
    let second_center = frame.origin() - frame.x() * 0.125;
    let second = Cylinder::new(
        Frame::new(second_center - frame.y() * 0.5, frame.y(), frame.x()).unwrap(),
        0.375,
    )
    .unwrap();
    [first, second]
}

fn opposite_pole_touching_body_axis_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap();
    let second_center = frame.origin() + frame.x() * 0.125;
    let second = Cylinder::new(
        Frame::new(second_center - frame.y() * 0.5, frame.y(), frame.x()).unwrap(),
        0.375,
    )
    .unwrap();
    [first, second]
}

fn double_touching_body_axis_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame.with_origin(frame.origin() - frame.z() * 0.5), 0.25).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - frame.x() * 0.5, frame.x(), frame.y()).unwrap(),
        0.25,
    )
    .unwrap();
    [first, second]
}

fn seam_perpendicular_axis_pair(frame: Frame, offset: f64, second_radius: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() + frame.x() * offset, frame.y(), frame.x()).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn seam_root_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 0.0625).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() + frame.y() * 0.125, frame.x(), frame.y()).unwrap(),
        0.125,
    )
    .unwrap();
    [first, second]
}

fn seam_root_across_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 0.0625).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - frame.y() * 0.125, frame.x(), frame.y()).unwrap(),
        0.125,
    )
    .unwrap();
    [first, second]
}

fn short_seam_root_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
    let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let first = Cylinder::new(frame, first_radius).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn short_seam_root_across_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let second_axis = frame.x() * 0.6 - frame.y() * 0.8;
    let second_radial = frame.x() * -0.8 - frame.y() * 0.6;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let reversed_first = Frame::new(frame.origin() + frame.z(), -frame.z(), frame.x()).unwrap();
    let first = Cylinder::new(reversed_first, first_radius).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - offset, second_axis, frame.z()).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn long_seam_root_across_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let second_axis = frame.x() * -0.6 + frame.y() * 0.8;
    let second_radial = frame.x() * 0.8 + frame.y() * 0.6;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let first = Cylinder::new(frame, first_radius).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - offset, second_axis, second_radial).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn long_seam_root_between_folded_support_pair(frame: Frame) -> [Cylinder; 2] {
    let first_radius = 0.0625;
    let second_radius = 0.125;
    let second_axis = frame.x() * -0.6 + frame.y() * 0.8;
    let second_radial = frame.x() * 0.8 + frame.y() * 0.6;
    let offset = second_radial * second_radius - frame.x() * first_radius + second_axis * 0.125
        - frame.z() * 0.125;
    let reversed_first = Frame::new(frame.origin(), -frame.z(), frame.x()).unwrap();
    let first = Cylinder::new(reversed_first, first_radius).unwrap();
    let second = Cylinder::new(
        Frame::new(frame.origin() - offset, second_axis, second_radial).unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn non_right_angle_axis_pair(frame: Frame, offset: f64, second_radius: f64) -> [Cylinder; 2] {
    let first = Cylinder::new(frame, 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            frame.origin() + frame.y() * offset,
            frame.x() * 0.6 + frame.z() * 0.8,
            frame.y(),
        )
        .unwrap(),
        second_radius,
    )
    .unwrap();
    [first, second]
}

fn one_sided_envelope_retry_pair() -> [Cylinder; 2] {
    let first = Cylinder::new(Frame::world(), 2.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(0.0, 8.0, 0.0),
            Vec3::new(1.0, 1.0, 2.0_f64.powi(-500)),
            Vec3::new(1.0, -1.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    [first, second]
}

fn skew_windows() -> [[ParamRange; 2]; 2] {
    [
        cylinder_window(range(-2.25, 2.25)),
        cylinder_window(range(-1.25, 1.25)),
    ]
}

fn assert_empty_skew_branch_graph(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.branch_graph.vertices.is_empty());
    assert!(result.branch_graph.edges.is_empty());
    assert!(result.raw.points.is_empty());
    assert!(result.raw.curves.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(
        result
            .parallel_cylinder_exterior_radial_separation()
            .is_none(),
        "a skew proof must not mint parallel radial-separation evidence"
    );
}

fn assert_single_skew_incomplete(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    code: DiagnosticCode,
    stage: kcore::operation::StageId,
    capability: CapabilityId,
    fixture: &str,
) {
    assert_empty_skew_branch_graph(result, sources);
    assert!(!result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(
        result.skew_cylinder_strict_discriminant_miss().is_none(),
        "an unresolved skew contact family must not carry a miss witness"
    );
    assert_eq!(result.raw.incomplete_evidence().len(), 1, "{fixture}");
    let evidence = result.raw.incomplete_evidence()[0];
    assert_eq!(evidence.code, code, "{fixture}");
    assert_eq!(evidence.stage, stage, "{fixture}");
    assert_eq!(
        evidence.cause,
        IncompleteCause::ProofMethodUnavailable { capability },
        "{fixture}"
    );
}

fn observed_work(
    report: &kcore::operation::OperationReport,
    stage: kcore::operation::StageId,
) -> u64 {
    report
        .usage()
        .iter()
        .find(|usage| usage.stage == stage && usage.resource == ResourceKind::Work)
        .map_or(0, |usage| usage.consumed)
}

fn assert_ruling_lifts(edge: &kops::intersect::IntersectionBranchEdge, cylinders: [Cylinder; 2]) {
    let CurveDescriptor::Line(carrier) = edge.carrier else {
        panic!("Cylinder/Cylinder ruling must retain an exact line carrier");
    };
    assert_eq!(edge.topology, IntersectionBranchTopology::Open);
    assert!(
        edge.pcurves
            .iter()
            .all(|pcurve| matches!(pcurve, Curve2dDescriptor::Line(_)))
    );
    let certificate = edge.certificate.as_cylinder_cylinder_ruling().unwrap();
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
    for parameter in [
        edge.carrier_range.lo,
        edge.carrier_range.lerp(0.37),
        edge.carrier_range.hi,
    ] {
        let point = carrier.eval(parameter);
        for (operand, cylinder) in cylinders.iter().enumerate() {
            let uv = edge.pcurves[operand]
                .as_curve()
                .eval(edge.parameter_maps[operand].map(parameter));
            assert!(point.dist(cylinder.eval([uv.x, uv.y])) <= certificate.tolerance());
        }
    }
}

fn assert_perpendicular_two_sheet_result(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    source_cylinders: [Cylinder; 2],
    construction_frame: Frame,
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(result.raw.points.is_empty());
    assert!(result.raw.regions.is_empty());
    assert!(result.raw.incomplete_evidence().is_empty());
    assert_eq!(result.raw.curves.len(), 2);
    assert_eq!(result.branch_graph.edges.len(), 2);
    assert_eq!(result.branch_graph.vertices.len(), 2);
    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());
    assert!(
        result
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );

    for (branch_index, expected_sheet) in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
        .into_iter()
        .enumerate()
    {
        let raw_branch = &result.raw.curves[branch_index];
        let SurfaceIntersectionCurve::SkewCylinder(raw_carrier) = raw_branch.curve else {
            panic!("strict-positive skew branch must use its procedural carrier");
        };
        assert_eq!(raw_carrier.sheet(), expected_sheet);

        let edge = &result.branch_graph.edges[branch_index];
        let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
            panic!("verified skew branch must retain its procedural carrier");
        };
        assert_eq!(carrier, raw_carrier);
        assert_eq!(carrier.sheet(), expected_sheet);
        assert_eq!(edge.carrier_range, raw_branch.curve_range);
        assert_eq!(edge.topology, IntersectionBranchTopology::Closed);
        assert_eq!(edge.endpoint_vertices, [branch_index, branch_index]);
        assert!(matches!(
            result.branch_graph.vertices[branch_index].event,
            IntersectionBranchVertexEvent::PeriodSeam { .. }
        ));
        assert!(
            edge.endpoint_events
                .iter()
                .all(|event| matches!(event, IntersectionBranchEndpointEvent::PeriodSeam { .. }))
        );
        assert!(
            edge.pcurves
                .iter()
                .all(|pcurve| matches!(pcurve, Curve2dDescriptor::SkewCylinderBranch(_)))
        );
        assert!(
            edge.parameter_maps
                .iter()
                .all(|map| map.scale() == 1.0 && map.offset() == 0.0)
        );

        let certificate = edge.certificate.as_skew_cylinder_two_sheet().unwrap();
        assert_eq!(certificate.carrier(), carrier);
        assert_eq!(certificate.sheet(), expected_sheet);
        assert_eq!(
            certificate.traces().map(|trace| trace.surface()),
            source_cylinders
        );
        assert_eq!(certificate.parameter_maps(), edge.parameter_maps);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );

        for parameter in [
            edge.carrier_range.lo,
            edge.carrier_range.lerp(0.25),
            edge.carrier_range.lerp(0.5),
            edge.carrier_range.lerp(0.75),
            edge.carrier_range.hi,
        ] {
            let (sine, cosine) = kcore::math::sincos(parameter);
            let ruling_height = (4.0 - sine * sine).sqrt()
                * if expected_sheet == SkewCylinderSheet::Lower {
                    -1.0
                } else {
                    1.0
                };
            let expected_point = construction_frame.origin()
                + construction_frame.x() * cosine
                + construction_frame.y() * sine
                + construction_frame.z() * ruling_height;
            let point = carrier.eval(parameter);
            assert!(
                point.dist(expected_point) <= certificate.tolerance(),
                "{expected_sheet:?} carrier disagrees with the perpendicular-cylinder oracle"
            );
            for (operand, cylinder) in source_cylinders.iter().enumerate() {
                let uv = edge.pcurves[operand]
                    .as_curve()
                    .eval(edge.parameter_maps[operand].map(parameter));
                assert!(
                    point.dist(cylinder.eval([uv.x, uv.y])) <= certificate.tolerance(),
                    "{expected_sheet:?} pcurve {operand} does not lift to the carrier"
                );
            }
        }
    }
}

fn assert_folded_support_result(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
) {
    assert_folded_support_result_in_root_order(result, sources, [0, 1]);
}

fn assert_folded_support_result_in_root_order(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    sources: [kgraph::SurfaceHandle; 2],
    root_order: [usize; 2],
) {
    assert_eq!(result.branch_graph.source_surfaces, sources);
    assert!(result.raw.is_complete());
    assert!(result.raw.points.is_empty());
    assert_eq!(result.raw.curves.len(), 2);
    assert_eq!(result.branch_graph.edges.len(), 2);
    assert_eq!(result.branch_graph.vertices.len(), 2);
    assert!(result.raw.incomplete_evidence().is_empty());
    assert!(result.skew_cylinder_support_contacts().is_empty());
    let [folded] = result.skew_cylinder_folded_support_curves() else {
        panic!("expected one exact folded support component")
    };
    assert_eq!(folded.certificate().topology().roots().len(), 2);
    assert!(
        folded
            .certificate()
            .topology()
            .roots()
            .iter()
            .all(|root| !root.repeated())
    );
    assert!(folded.certificate().required_edge_tolerance() <= folded.certificate().tolerance());
    for (vertex_index, vertex) in result.branch_graph.vertices.iter().enumerate() {
        let root_ordinal = root_order[vertex_index];
        assert_eq!(
            vertex.event,
            IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal }
        );
        assert_eq!(vertex.point, folded.endpoint_points()[root_ordinal]);
        assert_eq!(
            vertex.surface_parameters,
            folded.source_endpoint_parameters()[root_ordinal]
        );
    }
    for edge in &result.branch_graph.edges {
        let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
            panic!("folded support member must retain a procedural carrier")
        };
        assert_eq!(edge.topology, IntersectionBranchTopology::Open);
        assert_eq!(edge.endpoint_vertices, [0, 1]);
        assert_eq!(
            edge.endpoint_events,
            [
                IntersectionBranchEndpointEvent::FoldedSupportJoin {
                    root_ordinal: root_order[0],
                },
                IntersectionBranchEndpointEvent::FoldedSupportJoin {
                    root_ordinal: root_order[1],
                },
            ]
        );
        assert!(edge.carrier_range.width() > 0.0);
        let certificate = edge
            .certificate
            .as_skew_cylinder_folded_support()
            .expect("folded member retains its shared exact topology");
        assert_eq!(certificate.residual_certificate().carrier(), carrier);
        assert!(
            certificate
                .residual_certificate()
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.residual_certificate().tolerance())
        );
    }
}

fn assert_non_right_two_sheet_result(
    result: &kops::intersect::GraphSurfaceSurfaceIntersections,
    source_cylinders: [Cylinder; 2],
    construction_frame: Frame,
) {
    assert!(result.raw.is_complete());
    assert_eq!(result.raw.curves.len(), 2);
    assert_eq!(result.branch_graph.edges.len(), 2);
    assert_eq!(result.branch_graph.vertices.len(), 2);

    for (branch_index, expected_sheet) in [SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
        .into_iter()
        .enumerate()
    {
        let edge = &result.branch_graph.edges[branch_index];
        let CurveDescriptor::SkewCylinderBranch(carrier) = edge.carrier else {
            panic!("non-right skew branch must retain its procedural carrier");
        };
        assert_eq!(carrier.sheet(), expected_sheet);
        assert_eq!(edge.topology, IntersectionBranchTopology::Closed);
        let certificate = edge.certificate.as_skew_cylinder_two_sheet().unwrap();
        assert_eq!(
            certificate.traces().map(|trace| trace.surface()),
            source_cylinders
        );

        for parameter in [
            edge.carrier_range.lo,
            edge.carrier_range.lerp(0.25),
            edge.carrier_range.lerp(0.5),
            edge.carrier_range.lerp(0.75),
            edge.carrier_range.hi,
        ] {
            let (sine, cosine) = kcore::math::sincos(parameter);
            let signed_root = (4.0 - sine * sine).sqrt()
                * if expected_sheet == SkewCylinderSheet::Lower {
                    -1.0
                } else {
                    1.0
                };
            let ruling_height = (0.8 * cosine + signed_root) / 0.6;
            let expected_point = construction_frame.origin()
                + construction_frame.x() * cosine
                + construction_frame.y() * sine
                + construction_frame.z() * ruling_height;
            let point = carrier.eval(parameter);
            assert!(
                point.dist(expected_point) <= certificate.tolerance(),
                "{expected_sheet:?} carrier disagrees with the non-right oracle"
            );
            for (operand, cylinder) in source_cylinders.iter().enumerate() {
                let uv = edge.pcurves[operand]
                    .as_curve()
                    .eval(edge.parameter_maps[operand].map(parameter));
                assert!(
                    point.dist(cylinder.eval([uv.x, uv.y])) <= certificate.tolerance(),
                    "{expected_sheet:?} non-right pcurve {operand} does not lift"
                );
            }
        }
    }
}

#[test]
fn strict_parallel_secant_promotes_two_deterministic_rulings_in_both_orders() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.25),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    let first_range = cylinder_window(range(-1.0, 2.0));
    let second_range = cylinder_window(range(-0.75, 1.5));
    let (graph, first_handle, second_handle) = graph_pair(first, second);

    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_range,
        second_handle,
        second_range,
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_range,
        second_handle,
        second_range,
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        second_range,
        first_handle,
        first_range,
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert!(
        forward
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );
    assert!(
        reversed
            .parallel_cylinder_exterior_radial_separation()
            .is_none()
    );
    assert_eq!(forward.branch_graph.edges.len(), 2);
    assert_eq!(forward.branch_graph.vertices.len(), 4);
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    assert_eq!(reversed.branch_graph.edges.len(), 2);
    for edge in &forward.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [first_handle, second_handle]);
        assert_ruling_lifts(edge, [first, second]);
    }
    for edge in &reversed.branch_graph.edges {
        assert_eq!(edge.source_surfaces, [second_handle, first_handle]);
        assert_ruling_lifts(edge, [second, first]);
    }
    assert_eq!(
        forward
            .branch_graph
            .edges
            .iter()
            .map(|edge| (edge.carrier.clone(), edge.carrier_range))
            .collect::<Vec<_>>(),
        reversed
            .branch_graph
            .edges
            .iter()
            .map(|edge| (edge.carrier.clone(), edge.carrier_range))
            .collect::<Vec<_>>()
    );
}

#[test]
fn exact_antiparallel_oblique_axes_retain_operand_ordered_lifts() {
    let first_frame = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(-1.0, -1.0, 0.5),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let first = Cylinder::new(first_frame, 1.25).unwrap();
    let second = Cylinder::new(
        Frame::new(
            first_frame.origin() + first_frame.x(),
            -first_frame.z(),
            first_frame.x(),
        )
        .unwrap(),
        1.25,
    )
    .unwrap();
    let window = cylinder_window(range(-1.5, 2.0));
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let hit = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(hit.branch_graph.edges.len(), 2);
    assert!(hit.parallel_cylinder_exterior_radial_separation().is_none());
    for edge in &hit.branch_graph.edges {
        assert_ruling_lifts(edge, [first, second]);
        assert!(edge.parameter_maps[0].scale() * edge.parameter_maps[1].scale() < 0.0);
    }
}

fn assert_typed_gap(
    first: Cylinder,
    first_window: [ParamRange; 2],
    second: Cylinder,
    second_window: [ParamRange; 2],
) {
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let error = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        first_window,
        second_handle,
        second_window,
        Tolerances::default(),
    )
    .unwrap_err();
    assert!(matches!(
        error,
        GraphSurfaceIntersectionError::BranchCertificate(
            IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
        )
    ));
}

#[test]
fn exact_exterior_radial_misses_are_complete_witnessed_and_swap_stable() {
    let oblique = Frame::new(
        Point3::new(0.0, -1.0, 3.0),
        Vec3::new(0.0, 0.6, 0.8),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap();
    let cases = [
        (
            Cylinder::new(Frame::world(), 1.0).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(3.0, 0.0, 0.25),
                    Vec3::new(0.0, 0.0, 1.0),
                    Vec3::new(1.0, 0.0, 0.0),
                )
                .unwrap(),
                1.0,
            )
            .unwrap(),
        ),
        (
            Cylinder::new(oblique, 1.25).unwrap(),
            Cylinder::new(
                Frame::new(
                    Point3::new(2.0_f64.next_up(), -1.0, 3.0),
                    -oblique.z(),
                    oblique.x(),
                )
                .unwrap(),
                0.75,
            )
            .unwrap(),
        ),
    ];
    let windows = [
        cylinder_window(range(-1.0, 2.0)),
        cylinder_window(range(-0.5, 1.25)),
    ];

    for (first, second) in cases {
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let forward = intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        );
        let reversed = intersect_bounded_graph_surfaces_with_context(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            &context,
        );
        for (outcome, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            let result = outcome.result().unwrap();
            assert!(result.raw.is_proven_empty());
            assert!(result.raw.incomplete_evidence().is_empty());
            assert!(
                result
                    .parallel_cylinder_exterior_radial_separation()
                    .is_some()
            );
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.branch_graph.vertices.is_empty());
            assert!(result.branch_graph.edges.is_empty());
            let visits = outcome
                .report()
                .usage()
                .iter()
                .find(|usage| {
                    usage.stage == kgraph::eval_stage::NODE_VISITS
                        && usage.resource == ResourceKind::Work
                })
                .unwrap();
            assert_eq!(visits.consumed, 0);
        }
        assert_eq!(
            reversed.result().unwrap().raw,
            forward.result().unwrap().raw
        );
    }
}

#[test]
fn exact_exterior_miss_boundary_is_tolerance_independent_and_fails_closed() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let window = cylinder_window(range(-1.0, 1.0));
    let cylinder_at = |distance: f64, radius: f64| {
        Cylinder::new(
            Frame::new(
                Point3::new(distance, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            radius,
        )
        .unwrap()
    };

    let just_outside = cylinder_at(2.0_f64.next_up(), 1.0);
    let (graph, first_handle, second_handle) = graph_pair(first, just_outside);
    let miss = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(miss.raw.is_proven_empty());
    assert!(
        miss.parallel_cylinder_exterior_radial_separation()
            .is_some()
    );

    for distance in [2.0, 2.0_f64.next_down()] {
        assert_typed_gap(first, window, cylinder_at(distance, 1.0), window);
    }

    // This separation is entirely inside the default linear tolerance. The
    // graph proof must use exact source coefficients rather than inheriting the
    // lower solver's near-coincident policy.
    let tiny_first = Cylinder::new(Frame::world(), 1.0e-12).unwrap();
    let tiny_second = cylinder_at(4.0e-12, 2.0e-12);
    let (graph, first_handle, second_handle) = graph_pair(tiny_first, tiny_second);
    let tiny = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        window,
        second_handle,
        window,
        Tolerances::default(),
    )
    .unwrap();
    assert!(tiny.raw.is_proven_empty());
    assert!(
        tiny.parallel_cylinder_exterior_radial_separation()
            .is_some()
    );
}

#[test]
fn perpendicular_skew_miss_is_complete_swap_replay_and_rigid_stable() {
    let oblique = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(1.0, -2.0, 3.0),
        Vec3::new(2.0, 1.0, 0.5),
    )
    .unwrap();
    let windows = skew_windows();

    // In the fixture frame, A is the local z-axis and B is the local x-axis
    // through (0, d, 0). Substitution gives
    // v^2 = R^2 - (sin(u) - d)^2. For d=4 and R=2 the right-hand side is
    // strictly negative over the complete cycle. The upper one-ULP neighbor
    // of the d=3 repeated contact is independently strict-negative.
    for (name, frame, offset) in [
        ("world", Frame::world(), 4.0),
        ("rigid-oblique", oblique, 4.0),
        ("one-ulp-strict-miss", Frame::world(), 3.0_f64.next_up()),
    ] {
        let [first, second] = perpendicular_axis_pair(frame, offset, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();

        assert_eq!(forward, replay, "{name} replay changed the exact result");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_empty_skew_branch_graph(result, sources);
            assert!(result.raw.is_proven_empty(), "{name}");
            assert!(result.raw.incomplete_evidence().is_empty(), "{name}");
            assert!(
                result.skew_cylinder_strict_discriminant_miss().is_some(),
                "{name}"
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped(), "{name}");
        assert_eq!(
            reversed.skew_cylinder_strict_discriminant_miss(),
            forward.skew_cylinder_strict_discriminant_miss(),
            "{name}"
        );
    }
}

#[test]
fn non_right_angle_skew_miss_matches_axis_distance_oracle_and_is_swap_stable() {
    let [first, second] = non_right_angle_axis_pair(Frame::world(), 4.0, 2.0);
    let axis_cross = first.frame().z().cross(second.frame().z());
    let axis_cosine = first.frame().z().dot(second.frame().z())
        / (first.frame().z().norm() * second.frame().z().norm());
    let axis_distance = ((second.frame().origin() - first.frame().origin()).dot(axis_cross)).abs()
        / axis_cross.norm();
    assert!((axis_cosine - 0.8).abs() < 1.0e-14);
    assert!(axis_cosine != 0.0 && axis_cosine.abs() != 1.0);
    assert!((axis_distance - 4.0).abs() < 1.0e-14);
    assert!(axis_distance > first.radius() + second.radius());

    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    for (result, sources) in [
        (&forward, [first_handle, second_handle]),
        (&reversed, [second_handle, first_handle]),
    ] {
        assert_empty_skew_branch_graph(result, sources);
        assert!(result.raw.is_proven_empty());
        assert!(result.raw.incomplete_evidence().is_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
    }
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    assert_eq!(
        reversed.skew_cylinder_strict_discriminant_miss(),
        forward.skew_cylinder_strict_discriminant_miss()
    );
}

#[test]
fn one_sided_exact_envelope_refusal_retries_reversed_parameterization() {
    let [first, second] = one_sided_envelope_retry_pair();
    let axis_cross = first.frame().z().cross(second.frame().z());
    let axis_distance = ((second.frame().origin() - first.frame().origin()).dot(axis_cross)).abs()
        / axis_cross.norm();
    assert!(axis_distance > first.radius() + second.radius());

    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let forward = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let replay = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let reversed = intersect_bounded_graph_surfaces_with_context(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        &context,
    );

    assert_eq!(forward, replay);
    for (outcome, sources) in [
        (&forward, [first_handle, second_handle]),
        (&reversed, [second_handle, first_handle]),
    ] {
        let result = outcome.result().unwrap();
        assert_empty_skew_branch_graph(result, sources);
        assert!(result.raw.is_proven_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
        assert_eq!(
            observed_work(outcome.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
        );
        assert!(outcome.report().numeric_resolution_stages().is_empty());
        assert!(outcome.report().limit_events().is_empty());
    }
    assert_eq!(
        reversed.result().unwrap().raw,
        forward.result().unwrap().raw.clone().swapped()
    );
}

#[test]
fn mixed_skew_and_non_skew_canonicalization_is_permutation_invariant() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 0.0, 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();
    let branch_for = |sheet| {
        reversed
            .raw
            .curves
            .iter()
            .find(|branch| {
                matches!(
                    &branch.curve,
                    SurfaceIntersectionCurve::SkewCylinder(carrier)
                        if carrier.sheet() == sheet
                )
            })
            .unwrap()
            .clone()
    };
    let lower = branch_for(SkewCylinderSheet::Lower);
    let upper = branch_for(SkewCylinderSheet::Upper);
    let line = SurfaceSurfaceCurve {
        curve: SurfaceIntersectionCurve::Line(
            Line::new(Point3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        ),
        curve_range: range(0.0, core::f64::consts::TAU),
        uv_a_start: [core::f64::consts::PI, 0.0],
        uv_a_end: [core::f64::consts::PI, 1.0],
        uv_b_start: [0.0, 0.0],
        uv_b_end: [1.0, 0.0],
        kind: ContactKind::Transverse,
    };
    let branches = [lower, upper, line];
    let expected =
        SurfaceSurfaceIntersections::canonicalized_complete(Vec::new(), branches.to_vec()).unwrap();

    for permutation in [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ] {
        let result = SurfaceSurfaceIntersections::canonicalized_complete(
            Vec::new(),
            permutation.map(|index| branches[index].clone()).to_vec(),
        )
        .unwrap();
        assert_eq!(result, expected, "permutation {permutation:?}");
    }
    assert!(matches!(
        &expected.curves[0].curve,
        SurfaceIntersectionCurve::SkewCylinder(carrier)
            if carrier.sheet() == SkewCylinderSheet::Lower
    ));
    assert!(matches!(
        &expected.curves[1].curve,
        SurfaceIntersectionCurve::SkewCylinder(carrier)
            if carrier.sheet() == SkewCylinderSheet::Upper
    ));
    assert!(matches!(
        expected.curves[2].curve,
        SurfaceIntersectionCurve::Line(_)
    ));
}

#[test]
fn perpendicular_skew_positive_pair_promotes_two_closed_branches_rigidly_and_in_both_orders() {
    let frames = [
        Frame::world(),
        Frame::new(
            Point3::new(3.0, -2.0, 5.0),
            Vec3::new(0.0, 0.8, 0.6),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    ];
    let windows = skew_windows();

    for frame in frames {
        let [first, second] = perpendicular_axis_pair(frame, 0.0, 2.0);
        let (mut graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();

        assert_eq!(forward, replay);
        assert_perpendicular_two_sheet_result(
            &forward,
            [first_handle, second_handle],
            [first, second],
            frame,
        );
        assert_perpendicular_two_sheet_result(
            &reversed,
            [second_handle, first_handle],
            [second, first],
            frame,
        );
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
        for (forward_edge, reversed_edge) in forward
            .branch_graph
            .edges
            .iter()
            .zip(&reversed.branch_graph.edges)
        {
            assert_eq!(forward_edge.carrier, reversed_edge.carrier);
            assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
            assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
        }

        let counts_before = (
            graph.surface_count(),
            graph.curve_count(),
            graph.curve2d_count(),
        );
        assert!(matches!(
            persist_verified_graph_surface_intersections(&mut graph, &forward),
            Err(GraphSurfaceIntersectionError::BranchCertificate(
                IntersectionCertificateError::UnsupportedCarrierParameterization { .. }
            ))
        ));
        assert_eq!(
            (
                graph.surface_count(),
                graph.curve_count(),
                graph.curve2d_count()
            ),
            counts_before,
            "operation-local skew persistence must refuse before inserting descriptors"
        );
    }
}

#[test]
fn primitive_base_origins_retry_contact_in_the_reverse_two_sheet_parameterization() {
    let construction_frame = Frame::world();
    let first = Cylinder::new(
        construction_frame.with_origin(Point3::new(0.0, 0.0, -2.25)),
        1.0,
    )
    .unwrap();
    let second = Cylinder::new(
        Frame::new(
            Point3::new(-1.25, 0.0, 0.0),
            construction_frame.x(),
            construction_frame.y(),
        )
        .unwrap(),
        2.0,
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 4.5)),
        cylinder_window(range(0.0, 2.5)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert_perpendicular_two_sheet_result(
        &forward,
        [first_handle, second_handle],
        [first, second],
        construction_frame,
    );
    assert_perpendicular_two_sheet_result(
        &reversed,
        [second_handle, first_handle],
        [second, first],
        construction_frame,
    );
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    for (forward_edge, reversed_edge) in forward
        .branch_graph
        .edges
        .iter()
        .zip(&reversed.branch_graph.edges)
    {
        assert_eq!(forward_edge.carrier, reversed_edge.carrier);
        assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
        assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
    }
}

#[test]
fn non_right_skew_positive_pair_matches_independent_oracle_and_is_swap_stable() {
    let frame = Frame::world();
    let [first, second] = non_right_angle_axis_pair(frame, 0.0, 2.0);
    let windows = [
        cylinder_window(range(-5.0, 5.0)),
        cylinder_window(range(-5.0, 5.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let replay = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    assert_eq!(forward, replay);
    assert_non_right_two_sheet_result(&forward, [first, second], frame);
    assert_non_right_two_sheet_result(&reversed, [second, first], frame);
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
    for (forward_edge, reversed_edge) in forward
        .branch_graph
        .edges
        .iter()
        .zip(&reversed.branch_graph.edges)
    {
        assert_eq!(forward_edge.carrier, reversed_edge.carrier);
        assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
        assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
    }
}

#[test]
fn exact_perpendicular_support_contact_and_rooted_ulp_neighbor_publish() {
    let support_rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", support_rotated)] {
        let [first, second] = perpendicular_axis_pair(frame, 3.0, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();

        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}");
            assert_eq!(result.raw.points.len(), 1, "{name}");
            assert!(result.raw.curves.is_empty(), "{name}");
            assert!(result.raw.regions.is_empty(), "{name}");
            assert_eq!(result.branch_graph.vertices.len(), 1, "{name}");
            assert!(result.branch_graph.edges.is_empty(), "{name}");
            assert_eq!(
                result.branch_graph.vertices[0].event,
                IntersectionBranchVertexEvent::IsolatedContact
            );
            assert_eq!(result.branch_graph.vertices[0].kind, ContactKind::Tangent);
            assert!(result.skew_cylinder_isolated_contacts().is_empty());
            assert!(result.skew_cylinder_through_contacts().is_empty());
            let [contact] = result.skew_cylinder_support_contacts() else {
                panic!("{name}: expected one support contact")
            };
            assert!(contact.certificate().root().repeated());
            assert_eq!(contact.certificate().topology().roots().len(), 1);
            assert_eq!(contact.raw_point(), result.raw.points[0]);
            assert!(contact.point().dist(frame.origin() + frame.y()) <= 1.0e-12);
        }
        assert_eq!(
            reversed.raw,
            forward.raw.clone().swapped(),
            "{name} changed under operand reversal"
        );
    }

    let folded_rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    for (name, frame) in [("world", Frame::world()), ("rotated", folded_rotated)] {
        let [first, second] = perpendicular_axis_pair(frame, 3.0_f64.next_down(), 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        assert_folded_support_result(&forward, [first_handle, second_handle]);
        assert_folded_support_result(&reversed, [second_handle, first_handle]);
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
        for (forward_edge, reversed_edge) in forward
            .branch_graph
            .edges
            .iter()
            .zip(&reversed.branch_graph.edges)
        {
            assert_eq!(forward_edge.carrier, reversed_edge.carrier);
            assert_eq!(forward_edge.pcurves[0], reversed_edge.pcurves[1]);
            assert_eq!(forward_edge.pcurves[1], reversed_edge.pcurves[0]);
        }
    }
}

#[test]
fn exact_perpendicular_support_contact_on_authored_seam_publishes() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_perpendicular_axis_pair(frame, 3.0, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.points.len(), 1);
            assert!(result.raw.curves.is_empty());
            assert_eq!(result.branch_graph.vertices.len(), 1);
            assert!(result.branch_graph.edges.is_empty());
            assert_eq!(
                result.branch_graph.vertices[0].event,
                IntersectionBranchVertexEvent::IsolatedContact
            );
            let [contact] = result.skew_cylinder_support_contacts() else {
                panic!("{name}: expected one seam support contact")
            };
            let angular = contact.certificate().root().angular_bracket();
            assert_eq!(angular.lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular.hi.to_bits(), 0.0_f64.to_bits());
            assert_eq!(
                contact.certificate().carrier_parameter().to_bits(),
                0.0_f64.to_bits()
            );
            assert!(contact.point().dist(frame.origin() + frame.x()) <= 1.0e-12);
            let source_seam = contact.certificate().source_longitude_enclosures()
                [usize::from(result.branch_graph.source_surfaces[0] != first_handle)];
            assert_eq!(source_seam.lo().to_bits(), 0.0_f64.to_bits());
            assert_eq!(source_seam.hi().to_bits(), 0.0_f64.to_bits());
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn exact_perpendicular_support_contact_on_opposite_authored_seam_publishes() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_perpendicular_axis_pair(frame, -3.0, 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.points.len(), 1);
            assert!(result.raw.curves.is_empty());
            assert_eq!(result.branch_graph.vertices.len(), 1);
            assert!(result.branch_graph.edges.is_empty());
            assert_eq!(
                result.branch_graph.vertices[0].event,
                IntersectionBranchVertexEvent::IsolatedContact
            );
            let [contact] = result.skew_cylinder_support_contacts() else {
                panic!("{name}: expected one opposite-seam support contact")
            };
            let angular = contact.certificate().root().angular_bracket();
            assert_eq!(angular.lo.to_bits(), core::f64::consts::PI.to_bits());
            assert_eq!(angular.hi.to_bits(), core::f64::consts::PI.to_bits());
            assert_eq!(
                contact.certificate().carrier_parameter().to_bits(),
                core::f64::consts::PI.to_bits()
            );
            assert!(contact.point().dist(frame.origin() - frame.x()) <= 1.0e-12);
            let source_longitudes = contact.certificate().source_longitude_enclosures();
            let opposite_source =
                usize::from(result.branch_graph.source_surfaces[0] == first_handle);
            assert_eq!(
                source_longitudes[opposite_source].lo().to_bits(),
                0.0_f64.to_bits()
            );
            assert_eq!(
                source_longitudes[opposite_source].hi().to_bits(),
                core::f64::consts::TAU.to_bits()
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn four_simple_contact_cycle_publishes_two_folded_support_components() {
    let rotated = Frame::new(
        Point3::new(2.0, -1.0, 3.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = perpendicular_axis_pair(frame, 0.0, 0.03125);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (direction, result, sources) in [
            ("forward", &forward, [first_handle, second_handle]),
            ("reversed", &reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(
                result.raw.is_complete(),
                "{name}/{direction}: {:#?}",
                result.raw
            );
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            assert!(result.skew_cylinder_support_contacts().is_empty());
            assert!(result.skew_cylinder_touching_support_curves().is_empty());
            let folded = result.skew_cylinder_folded_support_curves();
            assert_eq!(folded.len(), 2);
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().topology().root_ordinals())
                    .collect::<Vec<_>>(),
                vec![[0, 3], [1, 2]]
            );
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().topology().positive_cell())
                    .collect::<Vec<_>>(),
                vec![
                    kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam,
                    kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots,
                ]
            );
            assert_eq!(
                folded
                    .iter()
                    .map(|component| component.certificate().formula_residuals().len())
                    .collect::<Vec<_>>(),
                vec![4, 2]
            );
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    ))
                    .count(),
                4
            );
            assert_eq!(
                result
                    .branch_graph
                    .vertices
                    .iter()
                    .filter(|vertex| matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportSeamJoin { .. }
                    ))
                    .count(),
                2
            );
            for edge in &result.branch_graph.edges {
                assert_eq!(edge.topology, IntersectionBranchTopology::Open);
                assert!(edge.certificate.as_skew_cylinder_folded_support().is_some());
            }
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn four_simple_folded_support_components_own_atomic_combined_work() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 0.0, 0.03125);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let exact_work =
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK + SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK;
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(exact_work);
    let result = exact.result().unwrap();
    assert_eq!(result.branch_graph.edges.len(), 6);
    assert_eq!(result.skew_cylinder_folded_support_curves().len(), 2);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        exact_work
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(exact_work - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: exact_work,
        allowed: exact_work - 1,
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
        0
    );
}

#[test]
fn authored_seam_support_contact_owns_existing_atomic_discriminant_work() {
    for offset in [3.0, -3.0] {
        let [first, second] = seam_perpendicular_axis_pair(Frame::world(), offset, 2.0);
        let windows = skew_windows();
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let run = |allowed| {
            let context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(
                    BudgetPlan::new([LimitSpec::new(
                        SKEW_CYLINDER_DISCRIMINANT_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        allowed,
                    )])
                    .unwrap(),
                );
            intersect_bounded_graph_surfaces_with_context(
                &graph,
                first_handle,
                windows[0],
                second_handle,
                windows[1],
                &context,
            )
        };

        let exact = run(SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK);
        assert_eq!(
            exact
                .result()
                .unwrap()
                .skew_cylinder_support_contacts()
                .len(),
            1,
            "offset={offset}"
        );
        assert_eq!(
            observed_work(exact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
        );
        assert!(exact.report().limit_events().is_empty());

        let denied = run(SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1);
        let expected = LimitSnapshot {
            stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
            resource: ResourceKind::Work,
            consumed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
            allowed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
        };
        assert!(matches!(
            denied.result(),
            Err(GraphSurfaceIntersectionError::OperationPolicy(
                kcore::operation::OperationPolicyError::LimitReached(snapshot)
            )) if *snapshot == expected
        ));
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(
            observed_work(denied.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
            0
        );
    }
}

#[test]
fn folded_support_contact_owns_atomic_existing_open_span_work() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 3.0_f64.next_down(), 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                SKEW_CYLINDER_OPEN_SPAN_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
            )])
            .unwrap(),
        );
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert_folded_support_result(exact.result().unwrap(), [first_handle, second_handle]);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(
            BudgetPlan::new([LimitSpec::new(
                SKEW_CYLINDER_OPEN_SPAN_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
            )])
            .unwrap(),
        );
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn seam_folded_support_publishes_four_members_and_four_exact_joins() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_perpendicular_axis_pair(frame, 3.0_f64.next_down(), 2.0);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one seam-folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            let root_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportJoin { .. }
                    )
                })
                .count();
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::FoldedSupportSeamJoin { .. }
                    )
                })
                .count();
            assert_eq!((root_joins, seam_joins), (2, 2));
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn seam_folded_support_owns_atomic_four_member_work() {
    let [first, second] = seam_perpendicular_axis_pair(Frame::world(), 3.0_f64.next_down(), 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn seam_root_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_root_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one seam-root folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            let root_ordinals = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            let chart_sheets = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportChartJoin { sheet } => Some(sheet),
                    _ => None,
                })
                .collect::<Vec<_>>();
            assert_eq!(root_ordinals, vec![0, 1]);
            assert_eq!(
                chart_sheets,
                vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            );
            assert!(
                result
                    .branch_graph
                    .edges
                    .iter()
                    .flat_map(|edge| edge.endpoint_events)
                    .all(|event| matches!(
                        event,
                        IntersectionBranchEndpointEvent::FoldedSupportJoin { .. }
                            | IntersectionBranchEndpointEvent::FoldedSupportChartJoin { .. }
                    ))
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn seam_root_across_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = skew_windows();
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one across-seam pole-pair folded component")
            };
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(3.0 * core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut root_ordinals = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            let chart_sheets = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportChartJoin { sheet } => Some(sheet),
                    _ => None,
                })
                .collect::<Vec<_>>();
            root_ordinals.sort_unstable();
            assert_eq!(root_ordinals, vec![0, 1]);
            assert_eq!(
                chart_sheets,
                vec![SkewCylinderSheet::Lower, SkewCylinderSheet::Upper]
            );
            assert!(
                result
                    .branch_graph
                    .edges
                    .iter()
                    .flat_map(|edge| edge.endpoint_events)
                    .all(|event| matches!(
                        event,
                        IntersectionBranchEndpointEvent::FoldedSupportJoin { .. }
                            | IntersectionBranchEndpointEvent::FoldedSupportChartJoin { .. }
                    ))
            );
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_folded_support_publishes_two_root_joined_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = short_seam_root_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        assert_folded_support_result(&forward, [first_handle, second_handle]);
        assert_folded_support_result(&reversed, [second_handle, first_handle]);
        for result in [&forward, &reversed] {
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one short seam-root folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 2);
            assert_eq!(folded.certificate().chart_join_longitude(), None);
            assert!(folded.certificate().seam_points().is_none());
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_folded_support_owns_atomic_two_member_work() {
    let [first, second] = short_seam_root_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_folded_support_result(exact.result().unwrap(), [first_handle, second_handle]);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn short_seam_root_across_folded_support_publishes_two_root_joined_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = short_seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        assert_folded_support_result_in_root_order(&forward, [first_handle, second_handle], [1, 0]);
        assert_folded_support_result_in_root_order(
            &reversed,
            [second_handle, first_handle],
            [1, 0],
        );
        for result in [&forward, &reversed] {
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one short across-seam folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(
                angular[1].lo > core::f64::consts::PI && angular[1].hi < core::f64::consts::TAU
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert_eq!(folded.certificate().formula_residuals().len(), 2);
            assert_eq!(folded.certificate().chart_join_longitude(), None);
            assert!(folded.certificate().seam_points().is_none());
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn short_seam_root_across_folded_support_owns_atomic_two_member_work() {
    let [first, second] = short_seam_root_across_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK);
    assert_folded_support_result_in_root_order(
        exact.result().unwrap(),
        [first_handle, second_handle],
        [1, 0],
    );
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn long_seam_root_across_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = long_seam_root_across_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one long across-seam folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > 0.0 && angular[1].hi < core::f64::consts::PI);
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(3.0 * core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::AcrossCanonicalSeam
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut roots = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![0, 1]);
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn long_seam_root_across_folded_support_owns_atomic_subdivided_work() {
    let [first, second] = long_seam_root_across_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn long_seam_root_between_folded_support_publishes_four_chart_split_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = long_seam_root_between_folded_support_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 4);
            assert_eq!(result.branch_graph.edges.len(), 4);
            assert_eq!(result.branch_graph.vertices.len(), 4);
            let [folded] = result.skew_cylinder_folded_support_curves() else {
                panic!("{name}: expected one long between-roots folded component")
            };
            let angular = folded
                .certificate()
                .topology()
                .roots()
                .map(|root| root.angular_bracket());
            assert_eq!(angular[0].lo.to_bits(), 0.0_f64.to_bits());
            assert_eq!(angular[0].hi.to_bits(), 0.0_f64.to_bits());
            assert!(angular[1].lo > core::f64::consts::PI);
            assert!(angular[1].hi < core::f64::consts::TAU);
            assert_eq!(folded.certificate().formula_residuals().len(), 4);
            assert_eq!(
                folded.certificate().chart_join_longitude(),
                Some(core::f64::consts::FRAC_PI_2)
            );
            assert_eq!(
                folded.certificate().topology().positive_cell(),
                kgraph::SkewCylinderFoldedSupportCellLocation::BetweenCanonicalRoots
            );
            assert!(folded.certificate().seam_points().is_none());
            let mut roots = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::FoldedSupportJoin { root_ordinal } => {
                        Some(root_ordinal)
                    }
                    _ => None,
                })
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![0, 1]);
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn long_seam_root_between_folded_support_owns_atomic_subdivided_work() {
    let [first, second] = long_seam_root_between_folded_support_pair(Frame::world());
    let windows = [
        cylinder_window(range(-0.5, 0.75)),
        cylinder_window(range(-0.5, 0.75)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_LONG_SEAM_ROOT_FOLDED_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn seam_root_folded_support_owns_atomic_four_member_work() {
    for [first, second] in [
        seam_root_folded_support_pair(Frame::world()),
        seam_root_across_folded_support_pair(Frame::world()),
    ] {
        let windows = skew_windows();
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let session = SessionPolicy::v1();
        let tolerances = Tolerances::default();
        let run = |allowed| {
            let context = OperationContext::new(&session, tolerances)
                .unwrap()
                .with_budget_overrides(
                    BudgetPlan::new([LimitSpec::new(
                        SKEW_CYLINDER_OPEN_SPAN_WORK,
                        ResourceKind::Work,
                        AccountingMode::Cumulative,
                        allowed,
                    )])
                    .unwrap(),
                );
            intersect_bounded_graph_surfaces_with_context(
                &graph,
                first_handle,
                windows[0],
                second_handle,
                windows[1],
                &context,
            )
        };

        let exact = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK);
        assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 4);
        assert_eq!(
            observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
            SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK
        );
        assert!(exact.report().limit_events().is_empty());

        let denied = run(SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1);
        let expected = LimitSnapshot {
            stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
            resource: ResourceKind::Work,
            consumed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK,
            allowed: SKEW_CYLINDER_SEAM_FOLDED_SUPPORT_EXACT_WORK - 1,
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
            0
        );
    }
}

#[test]
fn repeated_positive_support_touch_publishes_six_members_and_six_exact_joins() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = touching_body_axis_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for (result, sources) in [
            (&forward, [first_handle, second_handle]),
            (&reversed, [second_handle, first_handle]),
        ] {
            assert_eq!(result.branch_graph.source_surfaces, sources);
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert!(result.raw.points.is_empty());
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            assert!(result.skew_cylinder_folded_support_curves().is_empty());
            let [touching] = result.skew_cylinder_touching_support_curves() else {
                panic!("{name}: expected one repeated-root touching component")
            };
            assert_eq!(touching.certificate().formula_residuals().len(), 6);
            let root_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportRootJoin { .. }
                    )
                })
                .count();
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportSeamJoin { .. }
                    )
                })
                .count();
            let chart_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportChartJoin { .. }
                    )
                })
                .count();
            assert_eq!((root_joins, seam_joins, chart_joins), (2, 2, 2));
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn repeated_positive_support_touch_owns_atomic_six_member_work() {
    let [first, second] = touching_body_axis_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 6);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn repeated_positive_seam_touch_publishes_six_members_without_a_regular_seam_join() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = seam_touching_body_axis_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for result in [&forward, &reversed] {
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            let [touching] = result.skew_cylinder_touching_support_curves() else {
                panic!("{name}: expected one seam-root touching component")
            };
            assert_eq!(
                touching.certificate().chart_join_longitudes(),
                &[
                    core::f64::consts::FRAC_PI_2,
                    3.0 * core::f64::consts::FRAC_PI_2,
                ]
            );
            let root_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportRootJoin { .. }
                    )
                })
                .count();
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportSeamJoin { .. }
                    )
                })
                .count();
            let chart_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportChartJoin { .. }
                    )
                })
                .count();
            assert_eq!((root_joins, seam_joins, chart_joins), (2, 0, 4));
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn repeated_positive_seam_touch_owns_atomic_six_member_work() {
    let [first, second] = seam_touching_body_axis_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 6);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK
    );

    let denied = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn repeated_positive_opposite_pole_touch_publishes_eight_members() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = opposite_pole_touching_body_axis_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for result in [&forward, &reversed] {
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 8);
            assert_eq!(result.branch_graph.edges.len(), 8);
            assert_eq!(result.branch_graph.vertices.len(), 8);
            let [touching] = result.skew_cylinder_touching_support_curves() else {
                panic!("{name}: expected one opposite-pole touching component")
            };
            let root = touching.certificate().formula_root_longitudes()[0];
            assert_eq!(root.lo(), core::f64::consts::PI);
            assert_eq!(root.hi(), core::f64::consts::PI);
            assert_eq!(
                touching.certificate().chart_join_longitudes(),
                &[
                    core::f64::consts::FRAC_PI_2,
                    3.0 * core::f64::consts::FRAC_PI_2,
                ]
            );
            let root_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportRootJoin { .. }
                    )
                })
                .count();
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportSeamJoin { .. }
                    )
                })
                .count();
            let chart_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportChartJoin { .. }
                    )
                })
                .count();
            assert_eq!((root_joins, seam_joins, chart_joins), (2, 2, 4));
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn repeated_positive_opposite_pole_touch_owns_atomic_eight_member_work() {
    let [first, second] = opposite_pole_touching_body_axis_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 8);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied = run(SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_OPPOSITE_POLE_TOUCHING_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn double_repeated_positive_touch_publishes_two_crossing_closed_curves() {
    let rotated = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    )
    .unwrap();
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    for (name, frame) in [("world", Frame::world()), ("rotated", rotated)] {
        let [first, second] = double_touching_body_axis_pair(frame);
        let (graph, first_handle, second_handle) = graph_pair(first, second);
        let forward = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let replay = intersect_bounded_graph_surfaces(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            Tolerances::default(),
        )
        .unwrap();
        let reversed = intersect_bounded_graph_surfaces(
            &graph,
            second_handle,
            windows[1],
            first_handle,
            windows[0],
            Tolerances::default(),
        )
        .unwrap();
        assert_eq!(forward, replay, "{name} changed across replay");
        for result in [&forward, &reversed] {
            assert!(result.raw.is_complete(), "{name}: {:#?}", result.raw);
            assert_eq!(result.raw.curves.len(), 6);
            assert_eq!(result.branch_graph.edges.len(), 6);
            assert_eq!(result.branch_graph.vertices.len(), 6);
            let [touching] = result.skew_cylinder_touching_support_curves() else {
                panic!("{name}: expected one double-touching family")
            };
            assert_eq!(touching.certificate().topology().roots().len(), 2);
            assert!(touching.certificate().chart_join_longitudes().is_empty());
            let mut roots = result
                .branch_graph
                .vertices
                .iter()
                .filter_map(|vertex| match vertex.event {
                    IntersectionBranchVertexEvent::TouchingSupportRootJoin {
                        root_ordinal,
                        continuation,
                    } => Some((root_ordinal, continuation)),
                    _ => None,
                })
                .collect::<Vec<_>>();
            roots.sort_unstable();
            assert_eq!(roots, vec![(0, 0), (0, 1), (1, 0), (1, 1)]);
            let seam_joins = result
                .branch_graph
                .vertices
                .iter()
                .filter(|vertex| {
                    matches!(
                        vertex.event,
                        IntersectionBranchVertexEvent::TouchingSupportSeamJoin { .. }
                    )
                })
                .count();
            assert_eq!(seam_joins, 2);
        }
        assert_eq!(reversed.raw, forward.raw.clone().swapped());
    }
}

#[test]
fn double_repeated_positive_touch_owns_atomic_six_member_work() {
    let [first, second] = double_touching_body_axis_pair(Frame::world());
    let windows = [
        cylinder_window(range(0.0, 1.0)),
        cylinder_window(range(0.0, 1.0)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let run = |allowed| {
        let context = OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    SKEW_CYLINDER_OPEN_SPAN_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            );
        intersect_bounded_graph_surfaces_with_context(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &context,
        )
    };

    let exact = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK);
    assert_eq!(exact.result().unwrap().branch_graph.edges.len(), 6);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_OPEN_SPAN_WORK),
        SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK
    );

    let denied = run(SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1);
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_OPEN_SPAN_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK,
        allowed: SKEW_CYLINDER_TOUCHING_SUPPORT_EXACT_WORK - 1,
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
        0
    );
}

#[test]
fn exact_boundary_support_contact_retains_authored_bound_identity_in_both_orders() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 3.0, 2.0);
    let windows = [
        cylinder_window(range(0.0, 2.25)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    for (result, expected) in [
        (&forward, [Some(SkewCylinderAxialBoundary::Lower), None]),
        (&reversed, [None, Some(SkewCylinderAxialBoundary::Lower)]),
    ] {
        assert!(result.raw.is_complete(), "{result:#?}");
        assert_eq!(result.raw.points.len(), 1);
        assert!(result.raw.curves.is_empty());
        let [contact] = result.skew_cylinder_support_contacts() else {
            panic!("expected one boundary support contact")
        };
        assert_eq!(contact.certificate().source_axial_boundaries(), expected);
        assert_eq!(contact.certificate().boundary_plan().query_count(), 1);
        assert_eq!(
            contact.certificate().boundary_plan().work(),
            SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
        );
        assert!(contact.point().dist(Point3::new(0.0, 1.0, 0.0)) <= 1.0e-12);
    }
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
}

#[test]
fn exact_corner_support_contact_owns_both_bounds_at_atomic_work() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 3.0, 2.0);
    let windows = [
        cylinder_window(range(0.0, 2.25)),
        cylinder_window(range(0.0, 1.25)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let exact_work = 2 * SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK;
    let context = |allowed| {
        OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
    };

    for (left, left_window, right, right_window) in [
        (first_handle, windows[0], second_handle, windows[1]),
        (second_handle, windows[1], first_handle, windows[0]),
    ] {
        let exact_context = context(exact_work);
        let exact = intersect_bounded_graph_surfaces_with_context(
            &graph,
            left,
            left_window,
            right,
            right_window,
            &exact_context,
        );
        let [contact] = exact.result().unwrap().skew_cylinder_support_contacts() else {
            panic!("expected one corner support contact")
        };
        assert_eq!(
            contact.certificate().source_axial_boundaries(),
            [
                Some(SkewCylinderAxialBoundary::Lower),
                Some(SkewCylinderAxialBoundary::Lower),
            ]
        );
        assert_eq!(contact.certificate().boundary_plan().query_count(), 2);
        assert_eq!(contact.certificate().boundary_plan().work(), exact_work);
        assert_eq!(
            observed_work(
                exact.report(),
                kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK
            ),
            exact_work
        );
    }

    let denied_context = context(exact_work - 1);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK,
        resource: ResourceKind::Work,
        consumed: exact_work,
        allowed: exact_work - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(
            denied.report(),
            kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK
        ),
        0
    );
}

#[test]
fn boundary_support_contact_root_relation_work_is_exact_and_atomic() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 3.0, 2.0);
    let windows = [
        cylinder_window(range(0.0, 2.25)),
        cylinder_window(range(-1.25, 1.25)),
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();
    let context = |allowed| {
        OperationContext::new(&session, tolerances)
            .unwrap()
            .with_budget_overrides(
                BudgetPlan::new([LimitSpec::new(
                    kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK,
                    ResourceKind::Work,
                    AccountingMode::Cumulative,
                    allowed,
                )])
                .unwrap(),
            )
    };

    let exact_context = context(SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert_eq!(
        exact
            .result()
            .unwrap()
            .skew_cylinder_support_contacts()
            .len(),
        1
    );
    assert_eq!(
        observed_work(
            exact.report(),
            kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK
        ),
        SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK
    );

    let denied_context = context(SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK - 1);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK,
        allowed: SKEW_CYLINDER_ROOT_CLUSTER_PAIR_CHART_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(
            denied.report(),
            kops::intersect::SKEW_CYLINDER_ROOT_CLUSTER_WORK
        ),
        0
    );
}

#[test]
fn skew_two_sheet_refuses_nonperiodic_longitude_without_partial_publication() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 0.0, 2.0);
    let wide = skew_windows();
    let windows = [
        [
            range(0.0, core::f64::consts::TAU.next_down()),
            range(-2.25, 2.25),
        ],
        wide[1],
    ];
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let forward = intersect_bounded_graph_surfaces(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        Tolerances::default(),
    )
    .unwrap();
    let reversed = intersect_bounded_graph_surfaces(
        &graph,
        second_handle,
        windows[1],
        first_handle,
        windows[0],
        Tolerances::default(),
    )
    .unwrap();

    for (result, sources) in [
        (&forward, [first_handle, second_handle]),
        (&reversed, [second_handle, first_handle]),
    ] {
        assert_single_skew_incomplete(
            result,
            sources,
            SKEW_CYLINDER_TWO_SHEET_INCOMPLETE,
            SKEW_CYLINDER_TWO_SHEET_WORK,
            SKEW_CYLINDER_TWO_SHEET_BRANCH_CARRIER,
            "non-full-angular-window",
        );
    }
    assert_eq!(reversed.raw, forward.raw.clone().swapped());
}

#[test]
fn skew_miss_proof_validates_windows_and_fails_closed_on_unsafe_expansions() {
    let windows = skew_windows();
    let [first, second] = perpendicular_axis_pair(Frame::world(), 4.0, 2.0);
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let mut reversed_window = windows[0];
    reversed_window[1] = ParamRange { lo: 1.0, hi: -1.0 };
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default()).unwrap();
    let malformed = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        reversed_window,
        second_handle,
        windows[1],
        &context,
    );
    assert!(matches!(
        malformed.result(),
        Err(GraphSurfaceIntersectionError::Intersection(
            IntersectionError::Kernel(kcore::error::Error::InvalidGeometry { .. })
        ))
    ));
    assert_eq!(
        observed_work(malformed.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        0,
        "window validation must precede global discriminant certification"
    );
    assert!(malformed.report().limit_events().is_empty());

    let [first, unsafe_second] = perpendicular_axis_pair(Frame::world(), 1.0e200, 2.0);
    let (unsafe_graph, first_handle, second_handle) = graph_pair(first, unsafe_second);
    let unresolved = intersect_bounded_graph_surfaces_with_context(
        &unsafe_graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &context,
    );
    let result = unresolved
        .result()
        .expect("unsafe exact expansion must be incomplete, not a policy error");
    assert_empty_skew_branch_graph(result, [first_handle, second_handle]);
    assert!(!result.raw.is_complete());
    assert!(!result.raw.is_proven_empty());
    assert!(result.skew_cylinder_strict_discriminant_miss().is_none());
    assert_eq!(result.raw.incomplete_evidence().len(), 1);
    let evidence = result.raw.incomplete_evidence()[0];
    assert_eq!(evidence.code, SKEW_CYLINDER_DISCRIMINANT_NUMERIC_RESOLUTION);
    assert_eq!(evidence.stage, SKEW_CYLINDER_DISCRIMINANT_WORK);
    assert_eq!(evidence.cause, IncompleteCause::NumericResolution);
    assert_eq!(
        observed_work(unresolved.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert_eq!(
        unresolved.report().numeric_resolution_stages(),
        &[SKEW_CYLINDER_DISCRIMINANT_WORK]
    );
    assert!(unresolved.report().limit_events().is_empty());
}

#[test]
fn skew_discriminant_work_has_exact_n_and_atomic_n_minus_one_boundary() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 4.0, 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert!(exact.result().unwrap().raw.is_proven_empty());
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_DISCRIMINANT_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_DISCRIMINANT_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK,
        allowed: SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        0,
        "a rejected single-stage debit must not partially consume work"
    );

    let [first, tangent] = perpendicular_axis_pair(Frame::world(), 3.0, 2.0);
    let (contact_graph, first_handle, tangent_handle) = graph_pair(first, tangent);
    let exact_contact = intersect_bounded_graph_surfaces_with_context(
        &contact_graph,
        first_handle,
        windows[0],
        tangent_handle,
        windows[1],
        &exact_context,
    );
    let exact_contact_result = exact_contact.result().unwrap();
    assert!(exact_contact_result.raw.is_complete());
    assert_eq!(
        exact_contact_result.skew_cylinder_support_contacts().len(),
        1
    );
    assert_eq!(
        observed_work(exact_contact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );

    let denied_contact = intersect_bounded_graph_surfaces_with_context(
        &contact_graph,
        first_handle,
        windows[0],
        tangent_handle,
        windows[1],
        &denied_context,
    );
    assert!(matches!(
        denied_contact.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied_contact.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied_contact.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        0
    );
}

#[test]
fn skew_two_sheet_work_has_exact_n_and_atomic_n_minus_one_boundary() {
    let [first, second] = perpendicular_axis_pair(Frame::world(), 0.0, 2.0);
    let windows = skew_windows();
    let (graph, first_handle, second_handle) = graph_pair(first, second);
    let session = SessionPolicy::v1();
    let tolerances = Tolerances::default();

    let exact_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK,
    )])
    .unwrap();
    let exact_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(exact_plan);
    let exact = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &exact_context,
    );
    assert_eq!(exact.result().unwrap().raw.curves.len(), 2);
    assert_eq!(
        observed_work(exact.report(), SKEW_CYLINDER_TWO_SHEET_WORK),
        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK
    );
    assert!(exact.report().limit_events().is_empty());

    let denied_plan = BudgetPlan::new([LimitSpec::new(
        SKEW_CYLINDER_TWO_SHEET_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        SKEW_CYLINDER_TWO_SHEET_EXACT_WORK - 1,
    )])
    .unwrap();
    let denied_context = OperationContext::new(&session, tolerances)
        .unwrap()
        .with_budget_overrides(denied_plan);
    let denied = intersect_bounded_graph_surfaces_with_context(
        &graph,
        first_handle,
        windows[0],
        second_handle,
        windows[1],
        &denied_context,
    );
    let expected = LimitSnapshot {
        stage: SKEW_CYLINDER_TWO_SHEET_WORK,
        resource: ResourceKind::Work,
        consumed: SKEW_CYLINDER_TWO_SHEET_EXACT_WORK,
        allowed: SKEW_CYLINDER_TWO_SHEET_EXACT_WORK - 1,
    };
    assert!(matches!(
        denied.result(),
        Err(GraphSurfaceIntersectionError::OperationPolicy(
            kcore::operation::OperationPolicyError::LimitReached(snapshot)
        )) if *snapshot == expected
    ));
    assert_eq!(denied.report().limit_events(), &[expected]);
    assert_eq!(
        observed_work(denied.report(), SKEW_CYLINDER_TWO_SHEET_WORK),
        0,
        "a rejected two-certificate debit must not consume or publish one sheet"
    );
}

#[test]
fn default_graph_budget_admits_multiple_skew_pairs_in_one_owner_scope() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let second_at = |offset| perpendicular_axis_pair(Frame::world(), offset, 2.0)[1];
    let mut graph = GeometryGraph::new();
    let first_handle = graph.insert_surface(first).unwrap();
    let second_handles = [
        graph.insert_surface(second_at(4.0)).unwrap(),
        graph.insert_surface(second_at(5.0)).unwrap(),
    ];
    let windows = skew_windows();
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, Tolerances::default())
        .unwrap()
        .with_family_budget_defaults(GraphSurfaceBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);

    for second_handle in second_handles {
        let result = intersect_bounded_graph_surfaces_in_scope(
            &graph,
            first_handle,
            windows[0],
            second_handle,
            windows[1],
            &mut scope,
        )
        .expect("the aggregate graph budget must admit more than one skew face pair");
        assert!(result.raw.is_proven_empty());
        assert!(result.skew_cylinder_strict_discriminant_miss().is_some());
    }

    let outcome = scope.finish_typed::<_, GraphSurfaceIntersectionError>(Ok(()));
    assert_eq!(
        observed_work(outcome.report(), SKEW_CYLINDER_DISCRIMINANT_WORK),
        2 * SKEW_CYLINDER_DISCRIMINANT_EXACT_WORK
    );
    assert!(outcome.report().limit_events().is_empty());
}

#[test]
fn tangent_internal_coincident_and_axially_clipped_secant_remain_typed_gaps() {
    let first = Cylinder::new(Frame::world(), 1.0).unwrap();
    let window = cylinder_window(range(-1.0, 1.0));
    let cases = [
        Cylinder::new(
            Frame::new(
                Point3::new(2.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            1.0,
        )
        .unwrap(),
        Cylinder::new(
            Frame::new(
                Point3::new(0.5, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
            0.25,
        )
        .unwrap(),
        first,
    ];
    for second in cases {
        assert_typed_gap(first, window, second, window);
    }

    let secant = Cylinder::new(
        Frame::new(
            Point3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
        1.0,
    )
    .unwrap();
    assert_typed_gap(
        first,
        cylinder_window(range(-2.0, -1.0)),
        secant,
        cylinder_window(range(1.0, 2.0)),
    );
}
