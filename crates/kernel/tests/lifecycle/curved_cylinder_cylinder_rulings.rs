//! Facade-only lifecycle evidence for certified Cylinder/Cylinder rulings.
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, ClassifyPointOnFaceRequest, PointFaceVerdict, SectionBranch,
    SectionBranchTopology, SectionCarrier, SectionCurveEndpointTopology, SectionCurveFragmentSpan,
    SectionSite, SectionUvCurve, SurfaceEvaluationRequest,
};

const RADIUS: f64 = 1.0;
const AXIS_OFFSET: f64 = 0.5;
const ROOT_Y: f64 = 0.866_025_403_784_438_6;
const LONG_HALF_HEIGHT: f64 = 2.0;
const SHORT_HALF_HEIGHT: f64 = 1.0;
const TOLERANCE: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

struct Fixture {
    session: Session,
    part_id: PartId,
    first: BodyId,
    second: BodyId,
    frame: Frame,
    before: SourceSignature,
}

type SourceSignature = ([usize; 3], [usize; 3], usize);

fn shared_frame(placement: Placement) -> Frame {
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

fn body_topology(part: &kernel::Part<'_>, body: BodyId) -> [usize; 3] {
    let body = part.body(body).unwrap();
    [
        body.faces().unwrap().len(),
        body.edges().unwrap().len(),
        body.vertices().unwrap().len(),
    ]
}

fn source_signature(
    session: &Session,
    part_id: &PartId,
    first: &BodyId,
    second: &BodyId,
) -> SourceSignature {
    let part = session.part(part_id.clone()).unwrap();
    (
        body_topology(&part, first.clone()),
        body_topology(&part, second.clone()),
        part.bodies().len(),
    )
}

fn fixture(placement: Placement) -> Fixture {
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(-AXIS_OFFSET, 0.0, -LONG_HALF_HEIGHT)),
                RADIUS,
                2.0 * LONG_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(AXIS_OFFSET, 0.0, -SHORT_HALF_HEIGHT)),
                RADIUS,
                2.0 * SHORT_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let before = source_signature(&session, &part_id, &first, &second);
    assert_eq!(before, ([3, 2, 0], [3, 2, 0], 2));
    Fixture {
        session,
        part_id,
        first,
        second,
        frame,
        before,
    }
}

fn section(fixture: &Fixture, first: BodyId, second: BodyId) -> BodySectionGraph {
    fixture
        .session
        .part(fixture.part_id.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

fn face_is_cylinder(part: &kernel::Part<'_>, face: &kernel::FaceId) -> bool {
    let face = part.face(face.clone()).unwrap();
    part.surface(face.surface()).unwrap().class_key().as_str() == "kernel.surface.cylinder.v1"
}

fn cylinder_cylinder_branch(part: &kernel::Part<'_>, branch: &SectionBranch) -> bool {
    branch
        .faces()
        .iter()
        .all(|face| face_is_cylinder(part, face))
}

fn cylinder_cylinder_branch_indices(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
) -> Vec<usize> {
    graph
        .branches()
        .iter()
        .enumerate()
        .filter_map(|(index, branch)| cylinder_cylinder_branch(part, branch).then_some(index))
        .collect()
}

fn line_carrier(branch: &SectionBranch) -> (Point3, Vec3) {
    let SectionCarrier::Line { origin, direction } = branch.carrier() else {
        panic!("Cylinder/Cylinder ruling escaped its line carrier")
    };
    (origin, direction)
}

fn uv_line(branch: &SectionBranch, operand: usize) -> kernel::SectionUvLine {
    let SectionUvCurve::Line(line) = branch.pcurves()[operand] else {
        panic!("Cylinder/Cylinder ruling escaped its paired Line2d pcurves")
    };
    line
}

fn midpoint(branch: &SectionBranch) -> Point3 {
    let (origin, direction) = line_carrier(branch);
    origin + direction * branch.range().lerp(0.5)
}

fn assert_on_both_faces(
    part: &kernel::Part<'_>,
    branch: &SectionBranch,
    point: Point3,
    context: &str,
) {
    for face in branch.faces() {
        let verdict = part
            .classify_point_on_face(ClassifyPointOnFaceRequest::new(face.clone(), point))
            .unwrap()
            .into_result()
            .unwrap();
        assert!(
            matches!(verdict.verdict(), PointFaceVerdict::On(_)),
            "{context}: point escaped source face: {:?}",
            verdict.verdict()
        );
    }
}

fn assert_branch_contract(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    branch_index: usize,
    frame: Frame,
    short_operand: usize,
) {
    let branch = &graph.branches()[branch_index];
    assert_eq!(branch.topology(), SectionBranchTopology::Open);
    assert_eq!(branch.endpoint_sites(), [0, 1]);
    assert_eq!(branch.fragment_sites().len(), 2);
    let range = branch.range();
    assert!(range.is_finite() && range.lo < range.hi);
    let (origin, direction) = line_carrier(branch);
    assert!((direction.norm() - 1.0).abs() <= TOLERANCE);
    assert!(direction.cross(frame.z()).norm() <= TOLERANCE);

    let local_midpoint = frame.to_local(midpoint(branch));
    assert!(local_midpoint.x.abs() <= TOLERANCE);
    assert!((local_midpoint.y.abs() - ROOT_Y).abs() <= TOLERANCE);
    assert!(local_midpoint.z.abs() <= TOLERANCE);
    assert_on_both_faces(part, branch, midpoint(branch), "ruling midpoint");

    let evidence = branch.evidence();
    assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
    assert!(
        evidence
            .residual_bounds()
            .into_iter()
            .all(|bound| { bound.is_finite() && bound >= 0.0 && bound <= evidence.tolerance() })
    );
    for operand in 0..2 {
        let pcurve = uv_line(branch, operand);
        assert!(pcurve.direction().norm() > 0.0);
        let face = part.face(branch.faces()[operand].clone()).unwrap();
        for parameter in [range.lo, range.lerp(0.37), range.hi] {
            let point = origin + direction * parameter;
            let uv = pcurve.origin() + pcurve.direction() * parameter;
            let lifted = part
                .evaluate_surface(SurfaceEvaluationRequest::new(
                    face.surface(),
                    [uv.x, uv.y],
                    SurfaceDerivativeOrder::Position,
                ))
                .unwrap()
                .into_result()
                .unwrap()
                .position();
            let roundoff = 128.0 * f64::EPSILON * (1.0 + point.norm().max(lifted.norm()));
            assert!(
                point.dist(lifted) <= evidence.residual_bounds()[operand] + roundoff,
                "paired pcurve lift escaped its whole-range residual proof"
            );
        }
    }

    let fragments = graph
        .curve_fragments()
        .iter()
        .filter(|fragment| fragment.branch() == branch_index)
        .collect::<Vec<_>>();
    assert_eq!(fragments.len(), 1);
    assert_eq!(fragments[0].source_ordinal(), 0);
    let SectionCurveFragmentSpan::LineSegment { endpoints } = fragments[0].span() else {
        panic!("Cylinder/Cylinder ruling was not published as a bounded line")
    };
    for end in endpoints.iter() {
        assert!(branch.range().contains(end.carrier_parameter()));
        assert!(
            end.point()
                .dist(origin + direction * end.carrier_parameter())
                <= TOLERANCE
        );
        assert!((frame.to_local(end.point()).z.abs() - SHORT_HALF_HEIGHT).abs() <= TOLERANCE);
        assert_eq!(end.trims().iter().filter(|trim| trim.is_some()).count(), 1);
        let trim = end.trims()[short_operand]
            .as_ref()
            .expect("short cylinder must own both physical ruling ends");
        assert_eq!(trim.operand(), short_operand);
        assert_eq!(trim.face(), branch.faces()[short_operand]);
        assert!(end.trims()[1 - short_operand].is_none());
        let endpoint = &graph.curve_endpoints()[end.endpoint()];
        let SectionCurveEndpointTopology::Trim {
            sites,
            source_parameters,
        } = endpoint.topology()
        else {
            panic!("physical cylinder-ring endpoint became a chart seam")
        };
        assert_eq!(
            sites[short_operand],
            SectionSite::EdgeInterior(trim.source_parameter().edge())
        );
        assert_eq!(
            sites[1 - short_operand],
            SectionSite::FaceInterior(branch.faces()[1 - short_operand].clone())
        );
        assert_eq!(
            source_parameters[short_operand].as_ref(),
            Some(trim.source_parameter())
        );
        assert!(source_parameters[1 - short_operand].is_none());
        assert_on_both_faces(part, branch, end.point(), "ruling trim endpoint");
    }
}

fn matching_branch<'a>(
    graph: &'a BodySectionGraph,
    indices: &[usize],
    target: &SectionBranch,
) -> &'a SectionBranch {
    let target_midpoint = midpoint(target);
    indices
        .iter()
        .map(|&index| &graph.branches()[index])
        .find(|candidate| midpoint(candidate).dist(target_midpoint) <= TOLERANCE)
        .expect("swapped query lost an analytic ruling")
}

fn assert_closed_component_and_loci(
    graph: &BodySectionGraph,
    branch_indices: &[usize],
    frame: Frame,
) {
    assert_eq!(graph.curve_fragments().len(), 4);
    assert_eq!(graph.curve_endpoints().len(), 4);
    assert_eq!(graph.curve_components().len(), 1);
    let component = &graph.curve_components()[0];
    assert!(component.closed());
    assert_eq!(component.fragments().len(), 4);
    let mut ruling_fragments = 0;
    let mut cap_arcs = 0;
    for &fragment_index in component.fragments() {
        let fragment = &graph.curve_fragments()[fragment_index];
        match fragment.span() {
            SectionCurveFragmentSpan::LineSegment { .. } => {
                ruling_fragments += 1;
                assert!(branch_indices.contains(&fragment.branch()));
            }
            SectionCurveFragmentSpan::Arc { .. } => cap_arcs += 1,
            SectionCurveFragmentSpan::Whole => {
                panic!("nested-height section unexpectedly retained a whole-period carrier")
            }
            _ => panic!("nested-height section exposed an unknown fragment family"),
        }
    }
    assert_eq!(ruling_fragments, 2);
    assert_eq!(cap_arcs, 2);

    let mut lateral_roots = branch_indices
        .iter()
        .map(|&index| frame.to_local(midpoint(&graph.branches()[index])).y)
        .collect::<Vec<_>>();
    lateral_roots.sort_by(f64::total_cmp);
    assert_eq!(lateral_roots.len(), 2);
    assert!((lateral_roots[0] + ROOT_Y).abs() <= TOLERANCE);
    assert!((lateral_roots[1] - ROOT_Y).abs() <= TOLERANCE);
}

#[test]
fn certified_parallel_rulings_are_read_only_topology_owned_and_swap_deterministic() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = fixture(placement);
        let forward = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(
            forward, replay,
            "serial Section replay changed its evidence"
        );
        assert_eq!(
            forward.completion(),
            SectionCompletion::Complete,
            "forward graph: {forward:#?}"
        );
        assert_eq!(
            swapped.completion(),
            SectionCompletion::Complete,
            "swapped graph: {swapped:#?}"
        );
        assert!(forward.gaps().is_empty());
        assert!(swapped.gaps().is_empty());

        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        let forward_indices = cylinder_cylinder_branch_indices(&part, &forward);
        let swapped_indices = cylinder_cylinder_branch_indices(&part, &swapped);
        assert_eq!(
            forward_indices.len(),
            2,
            "duplicate or missing curved publication"
        );
        assert_eq!(
            swapped_indices.len(),
            2,
            "duplicate or missing swapped publication"
        );
        assert_closed_component_and_loci(&forward, &forward_indices, fixture.frame);
        assert_closed_component_and_loci(&swapped, &swapped_indices, fixture.frame);
        for &index in &forward_indices {
            assert_branch_contract(&part, &forward, index, fixture.frame, 1);
        }
        for &index in &swapped_indices {
            assert_branch_contract(&part, &swapped, index, fixture.frame, 0);
        }

        for &index in &forward_indices {
            let original = &forward.branches()[index];
            let exchanged = matching_branch(&swapped, &swapped_indices, original);
            let (origin, direction) = line_carrier(original);
            let (exchanged_origin, exchanged_direction) = line_carrier(exchanged);
            assert!(origin.dist(exchanged_origin) <= TOLERANCE);
            assert!((direction + exchanged_direction).norm() <= TOLERANCE);
            assert!((original.range().lo + exchanged.range().hi).abs() <= TOLERANCE);
            assert!((original.range().hi + exchanged.range().lo).abs() <= TOLERANCE);
            assert_eq!(
                original.faces(),
                &[exchanged.faces()[1].clone(), exchanged.faces()[0].clone()]
            );
            for operand in 0..2 {
                let original_pcurve = uv_line(original, operand);
                let exchanged_pcurve = uv_line(exchanged, 1 - operand);
                assert!((original_pcurve.origin() - exchanged_pcurve.origin()).norm() <= TOLERANCE);
                assert!(
                    (original_pcurve.direction() + exchanged_pcurve.direction()).norm()
                        <= TOLERANCE
                );
            }
            let bounds = original.evidence().residual_bounds();
            let exchanged_bounds = exchanged.evidence().residual_bounds();
            assert_eq!(bounds, [exchanged_bounds[1], exchanged_bounds[0]]);
        }
        drop(part);

        let after = source_signature(
            &fixture.session,
            &fixture.part_id,
            &fixture.first,
            &fixture.second,
        );
        assert_eq!(after, fixture.before, "Section mutated either source body");
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        for body in [fixture.first.clone(), fixture.second.clone()] {
            let check = part
                .check_body(CheckBodyRequest::new(body, CheckLevel::Full))
                .unwrap()
                .into_result()
                .unwrap();
            assert_eq!(check.outcome(), CheckOutcome::Valid);
            assert!(check.gaps().is_empty());
        }
    }
}

#[test]
fn skew_cylinder_pair_is_one_typed_gap_without_a_planar_fallback_duplicate() {
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(Frame::world(), 1.0, 3.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let skew = Frame::new(
            Point3::new(0.75, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(skew, 1.0, 3.0))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        (first, second)
    };
    let before = source_signature(&session, &part_id, &first, &second);
    let part = session.part(part_id.clone()).unwrap();
    let request = SectionBodiesRequest::new(first.clone(), second.clone());
    let graph = part
        .section_bodies(request.clone())
        .unwrap()
        .into_result()
        .unwrap();
    let replay = part.section_bodies(request).unwrap().into_result().unwrap();
    assert_eq!(graph, replay);
    assert_eq!(graph.completion(), SectionCompletion::Indeterminate);
    assert!(cylinder_cylinder_branch_indices(&part, &graph).is_empty());
    let cylinder_pair_gaps = graph
        .gaps()
        .iter()
        .filter(|gap| {
            gap.faces().len() == 2 && gap.faces().iter().all(|face| face_is_cylinder(&part, face))
        })
        .collect::<Vec<_>>();
    assert_eq!(
        cylinder_pair_gaps.len(),
        1,
        "the curved owner and planar fallback both reported the same pair: {:#?}",
        graph.gaps()
    );
    assert_eq!(
        source_signature(&session, &part_id, &first, &second),
        before
    );
}
