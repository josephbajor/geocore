//! Public-facade contract for verified transverse Plane/Cylinder rulings.

use super::*;

const RULING_TOL: f64 = 1e-10;

fn ruling_scene(block_frame: Frame, cylinder_frame: Frame) -> (Session, PartId, BodyId, BodyId) {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (block, cylinder) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        // The x faces cross the side surface in four rulings. The y faces lie
        // outside the radius, while the z faces lie beyond the cylinder's
        // finite side range and therefore cannot fabricate circle branches.
        let block = edit
            .create_block(BlockRequest::new(block_frame, [1.0, 4.0, 6.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let cylinder = edit
            .create_cylinder(CylinderRequest::new(cylinder_frame, 1.0, 4.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (block, cylinder)
    };
    (session, part_id, block, cylinder)
}

fn world_ruling_scene() -> (Session, PartId, BodyId, BodyId) {
    let cylinder_frame = Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0));
    let block_frame = Frame::world();
    ruling_scene(block_frame, cylinder_frame)
}

fn line_carrier(branch: &SectionBranch) -> (Point3, Vec3) {
    match branch.carrier() {
        SectionCarrier::Line { origin, direction } => (origin, direction),
        SectionCarrier::Circle { .. } => panic!("ruling branch must expose a line carrier"),
    }
}

fn uv_line(curve: SectionUvCurve) -> SectionUvLine {
    match curve {
        SectionUvCurve::Line(line) => line,
        SectionUvCurve::Circle(_) => panic!("ruling trace must expose a line pcurve"),
    }
}

fn eval_uv(line: SectionUvLine, parameter: f64) -> Point2 {
    line.origin() + line.direction() * parameter
}

fn assert_point_close(actual: Point3, expected: Point3, context: &str) {
    assert!(
        actual.dist(expected) <= RULING_TOL,
        "{context}: expected {expected:?}, got {actual:?}"
    );
}

fn assert_uv_close(actual: Point2, expected: [f64; 2], context: &str) {
    assert!(
        (actual.x - expected[0]).abs() <= RULING_TOL
            && (actual.y - expected[1]).abs() <= RULING_TOL,
        "{context}: expected {expected:?}, got {actual:?}"
    );
}

fn assert_ruling_contract(
    session: &Session,
    part_id: &PartId,
    graph: &BodySectionGraph,
    expected_axis: Vec3,
) {
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert!(graph.edges().is_empty());
    assert!(graph.vertices().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert_eq!(graph.curve_endpoints().len(), 8, "ruling graph: {graph:#?}");
    assert_eq!(graph.curve_fragments().len(), 4);
    assert!(graph.curve_components().is_empty());
    assert_eq!(graph.branches().len(), 4);
    assert_eq!(
        graph
            .gaps()
            .iter()
            .filter(|gap| gap.reason() == GAP_MIXED_FRAGMENT_STITCH)
            .count(),
        1,
        "all disconnected rulings must share one graph-global mixed-stitch gap"
    );
    assert!(
        graph
            .gaps()
            .iter()
            .all(|gap| gap.reason() != GAP_CURVED_TRIM_UNRESOLVED
                && gap.reason() != GAP_RULING_TRIM_UNRESOLVED),
        "topology-clipped rulings must not retain either retired trim reason"
    );
    assert_stable_gap_reasons(graph);

    for (branch_index, branch) in graph.branches().iter().enumerate() {
        assert_eq!(branch.topology(), SectionBranchTopology::Open);
        assert_eq!(branch.endpoint_sites(), [0, 1]);
        assert_eq!(branch.fragment_sites().len(), 2);
        assert_ne!(branch.fragment_sites()[0], branch.fragment_sites()[1]);
        let range = branch.range();
        assert!(range.is_finite() && range.lo < range.hi);
        let (origin, direction) = line_carrier(branch);
        assert!((direction.norm() - 1.0).abs() <= RULING_TOL);
        assert!(direction.cross(expected_axis).norm() <= RULING_TOL);

        let fragment = graph
            .curve_fragments()
            .iter()
            .find(|fragment| fragment.branch() == branch_index)
            .expect("every ruling branch must publish one topology-owned fragment");
        assert_eq!(fragment.source_ordinal(), 0);
        let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
            panic!("ruling branch must publish an affine line segment")
        };
        for end in endpoints.iter() {
            assert!(end.endpoint() < graph.curve_endpoints().len());
            assert_point_close(
                end.point(),
                origin + direction * end.carrier_parameter(),
                "ruling fragment representative",
            );
            let trims = end.trims();
            assert_eq!(trims.iter().filter(|trim| trim.is_some()).count(), 1);
            let trim = trims.iter().flatten().next().unwrap();
            assert_eq!(trim.face(), branch.faces()[trim.operand()]);
            assert!(trim.edge_parameter().lo() < trim.edge_parameter().hi());
            assert!(trim.carrier_parameter().lo() < trim.carrier_parameter().hi());
            let endpoint = &graph.curve_endpoints()[end.endpoint()];
            let SectionCurveEndpointTopology::Trim {
                sites,
                source_parameters,
            } = endpoint.topology()
            else {
                panic!("physical ruling trim must not become a parameter seam")
            };
            assert_eq!(
                sites[trim.operand()],
                SectionSite::EdgeInterior(trim.source_parameter().edge())
            );
            assert_eq!(
                source_parameters[trim.operand()].as_ref(),
                Some(trim.source_parameter())
            );
            assert_eq!(
                endpoint.edge_parameters()[trim.operand()],
                Some(trim.edge_parameter())
            );
        }

        let pcurves = [uv_line(branch.pcurves()[0]), uv_line(branch.pcurves()[1])];
        assert!(pcurves.iter().all(|line| {
            let direction = line.direction();
            direction.x.is_finite() && direction.y.is_finite() && direction.norm() > 0.0
        }));
        let evidence = branch.evidence();
        assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
        assert!(
            evidence.residual_bounds().into_iter().all(|bound| {
                bound.is_finite() && bound >= 0.0 && bound <= evidence.tolerance()
            })
        );

        for (endpoint, parameter) in [range.lo, range.hi].into_iter().enumerate() {
            let site = branch.fragment_sites()[endpoint];
            let carrier_point = origin + direction * parameter;
            assert_point_close(
                carrier_point,
                site.point(),
                &format!("branch {branch_index} endpoint {endpoint}"),
            );
            for (operand, pcurve) in pcurves.into_iter().enumerate() {
                assert_uv_close(
                    eval_uv(pcurve, parameter),
                    site.surface_parameters()[operand],
                    &format!("branch {branch_index} endpoint {endpoint} operand {operand}"),
                );
            }
            assert!(site.surface_window_boundaries().into_iter().any(|hit| hit));
            assert_boundary_on_both(
                session,
                part_id,
                graph.bodies(),
                carrier_point,
                &format!("ruling branch {branch_index} endpoint {endpoint}"),
            );
        }

        for fraction in [0.25, 0.5, 0.75] {
            let parameter = range.lo + range.width() * fraction;
            assert_boundary_on_both(
                session,
                part_id,
                graph.bodies(),
                origin + direction * parameter,
                &format!("ruling branch {branch_index} sample {fraction}"),
            );
        }
    }
}

fn midpoint(branch: &SectionBranch) -> Point3 {
    let (origin, direction) = line_carrier(branch);
    origin + direction * (branch.range().lo + branch.range().hi) * 0.5
}

fn matching_branch<'a>(branches: &'a [SectionBranch], target: &SectionBranch) -> &'a SectionBranch {
    let target_midpoint = midpoint(target);
    branches
        .iter()
        .find(|candidate| midpoint(candidate).dist(target_midpoint) <= RULING_TOL)
        .expect("equivalent query must retain the same geometric ruling")
}

#[test]
fn world_rulings_expose_exact_open_carriers_pcurves_sites_and_residuals() {
    let (session, part_id, block, cylinder) = world_ruling_scene();
    let graph = section_graph(&session, &part_id, &block, &cylinder);
    let repeated = section_graph(&session, &part_id, &block, &cylinder);
    assert_eq!(
        repeated, graph,
        "serial section reruns must reproduce the complete published ruling payload"
    );
    assert_ruling_contract(&session, &part_id, &graph, Frame::world().z());
}

#[test]
fn oblique_rulings_reverse_for_operand_swap_and_ignore_cylinder_axis_sign() {
    let base = Point3::new(2.75, -1.5, 0.625);
    let cylinder_frame = Frame::new(
        base,
        Vec3::new(0.3, 0.4, 0.866_025_403_784_438_6),
        Vec3::new(0.9, -0.1, -0.2),
    )
    .unwrap();
    let block_frame = cylinder_frame.with_origin(base + cylinder_frame.z() * 2.0);
    let (session, part_id, block, cylinder) = ruling_scene(block_frame, cylinder_frame);
    let forward = section_graph(&session, &part_id, &block, &cylinder);
    let swapped = section_graph(&session, &part_id, &cylinder, &block);
    assert_ruling_contract(&session, &part_id, &forward, cylinder_frame.z());
    assert_ruling_contract(&session, &part_id, &swapped, cylinder_frame.z());

    for branch in forward.branches() {
        let counterpart = matching_branch(swapped.branches(), branch);
        let (origin, direction) = line_carrier(branch);
        let (swapped_origin, swapped_direction) = line_carrier(counterpart);
        assert_point_close(swapped_origin, origin, "operand-swap carrier origin");
        assert!((swapped_direction + direction).norm() <= RULING_TOL);
        assert!((counterpart.range().lo + branch.range().hi).abs() <= RULING_TOL);
        assert!((counterpart.range().hi + branch.range().lo).abs() <= RULING_TOL);
        for operand in 0..2 {
            let original = uv_line(branch.pcurves()[operand]);
            let exchanged = uv_line(counterpart.pcurves()[1 - operand]);
            assert!((original.origin() - exchanged.origin()).norm() <= RULING_TOL);
            assert!((original.direction() + exchanged.direction()).norm() <= RULING_TOL);
        }
        let original_bounds = branch.evidence().residual_bounds();
        let exchanged_bounds = counterpart.evidence().residual_bounds();
        assert_eq!(original_bounds, [exchanged_bounds[1], exchanged_bounds[0]]);
    }

    let reversed_cylinder_frame = Frame::new(
        base + cylinder_frame.z() * 4.0,
        -cylinder_frame.z(),
        cylinder_frame.x(),
    )
    .unwrap();
    let (reverse_session, reverse_part, reverse_block, reverse_cylinder) =
        ruling_scene(block_frame, reversed_cylinder_frame);
    let reversed = section_graph(
        &reverse_session,
        &reverse_part,
        &reverse_block,
        &reverse_cylinder,
    );
    assert_ruling_contract(
        &reverse_session,
        &reverse_part,
        &reversed,
        cylinder_frame.z(),
    );
    for branch in forward.branches() {
        let counterpart = matching_branch(reversed.branches(), branch);
        let (_, direction) = line_carrier(branch);
        let (_, reversed_direction) = line_carrier(counterpart);
        assert!(
            (direction - reversed_direction).norm() <= RULING_TOL,
            "reversing only the cylinder chart axis must preserve the canonical model-space line"
        );
    }
}
