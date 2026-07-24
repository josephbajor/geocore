//! Facade-only lifecycle evidence for certified Cylinder/Cylinder rulings.
//! Wall-time budget: less than 60 seconds as part of the `lifecycle` target.

use super::*;
use kernel::{
    BodySectionGraph, ClassifyPointOnFaceRequest, PointFaceVerdict,
    SectionBoundedProceduralFragmentEnd, SectionBranch, SectionBranchTopology, SectionCarrier,
    SectionCurveEndpointTopology, SectionCurveFragmentSpan, SectionPeriodicEmbeddingGap,
    SectionSite, SectionSkewCylinderBranchCarrier, SectionSkewCylinderBranchPcurve, SectionUvCurve,
    SurfaceEvaluationRequest,
};

const RADIUS: f64 = 1.0;
const AXIS_OFFSET: f64 = 0.5;
const ROOT_Y: f64 = 0.866_025_403_784_438_6;
const LONG_HALF_HEIGHT: f64 = 2.0;
const SHORT_HALF_HEIGHT: f64 = 1.0;
const SKEW_FIRST_HALF_HEIGHT: f64 = 2.25;
const SKEW_SECOND_HALF_HEIGHT: f64 = 1.25;
const SKEW_SECOND_RADIUS: f64 = 2.0;
const BOUNDED_SKEW_LOWER: f64 = 1.8;
const BOUNDED_SKEW_UPPER: f64 = 1.9;
const TOLERANCE: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
enum Placement {
    World,
    Oblique,
}

#[derive(Debug, Clone, Copy)]
enum AxisDirection {
    Parallel,
    Antiparallel,
}

#[derive(Debug, Clone, Copy)]
enum RadialRelation {
    ExteriorDiagonal,
    ExteriorBroadPhase,
    ExteriorOneUlp,
    Tangent,
    Internal,
    Coincident,
    Skew,
    SkewMiss,
}

struct Fixture {
    session: Session,
    part_id: PartId,
    first: BodyId,
    second: BodyId,
    frame: Frame,
    before: SourceSignature,
}

struct RelationFixture {
    session: Session,
    part_id: PartId,
    first: BodyId,
    second: BodyId,
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

fn skew_two_sheet_fixture(placement: Placement, narrow_first_height: bool) -> Fixture {
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first_height = if narrow_first_height {
            SKEW_FIRST_HALF_HEIGHT
        } else {
            2.0 * SKEW_FIRST_HALF_HEIGHT
        };
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -SKEW_FIRST_HALF_HEIGHT)),
                RADIUS,
                first_height,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame = Frame::new(
            frame.point_at(-SKEW_SECOND_HALF_HEIGHT, 0.0, 0.0),
            frame.x(),
            frame.y(),
        )
        .unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                second_frame,
                SKEW_SECOND_RADIUS,
                2.0 * SKEW_SECOND_HALF_HEIGHT,
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

fn bounded_skew_fixture(placement: Placement) -> Fixture {
    let frame = shared_frame(placement);
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, BOUNDED_SKEW_LOWER)),
                RADIUS,
                BOUNDED_SKEW_UPPER - BOUNDED_SKEW_LOWER,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_frame = Frame::new(
            frame.point_at(-SKEW_SECOND_HALF_HEIGHT, 0.0, 0.0),
            frame.x(),
            frame.y(),
        )
        .unwrap();
        let second = edit
            .create_cylinder(CylinderRequest::new(
                second_frame,
                SKEW_SECOND_RADIUS,
                2.0 * SKEW_SECOND_HALF_HEIGHT,
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

fn relation_fixture(
    placement: Placement,
    direction: AxisDirection,
    relation: RadialRelation,
) -> RelationFixture {
    let frame = shared_frame(placement);
    let (first_radius, second_radius, radial_offset) = match relation {
        // Both authored-frame support-axis projections overlap, so the
        // side/side pair reaches the graph-aware Cylinder/Cylinder solver.
        RadialRelation::ExteriorDiagonal => (1.0, 1.0, [1.5, 1.5]),
        // This pair is already separated by the support-function broad phase;
        // Section must still ask kops for analytic radial provenance.
        RadialRelation::ExteriorBroadPhase => (1.0, 1.0, [3.0, 0.0]),
        RadialRelation::ExteriorOneUlp => (1.0, 1.0, [2.0_f64.next_up(), 0.0]),
        RadialRelation::Tangent => (1.0, 1.0, [2.0, 0.0]),
        RadialRelation::Internal => (2.0, 0.5, [0.25, 0.0]),
        RadialRelation::Coincident => (1.0, 1.0, [0.0, 0.0]),
        RadialRelation::Skew => (1.0, 1.0, [0.75, 0.0]),
        // The axes have cosine 4/5 and exact closest separation 4 along the
        // shared y direction. Since 4 > 1+2, the infinite supports are
        // independently disjoint; the non-right angle exercises A != 1.
        RadialRelation::SkewMiss => (1.0, 2.0, [0.0, 4.0]),
    };
    let mut session = Kernel::new().create_session();
    let part_id = session.create_part();
    let (first, second) = {
        let mut edit = session.edit_part(part_id.clone()).unwrap();
        let first = edit
            .create_cylinder(CylinderRequest::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -LONG_HALF_HEIGHT)),
                first_radius,
                2.0 * LONG_HALF_HEIGHT,
            ))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        let second_axial_origin = match direction {
            AxisDirection::Parallel => -SHORT_HALF_HEIGHT,
            AxisDirection::Antiparallel => SHORT_HALF_HEIGHT,
        };
        let second_origin = frame.point_at(radial_offset[0], radial_offset[1], second_axial_origin);
        let second_frame = match relation {
            RadialRelation::Skew => {
                Frame::new(second_origin, frame.y() + frame.z(), frame.x()).unwrap()
            }
            RadialRelation::SkewMiss => {
                Frame::new(second_origin, frame.x() * 0.6 + frame.z() * 0.8, frame.y()).unwrap()
            }
            _ => match direction {
                AxisDirection::Parallel => frame.with_origin(second_origin),
                AxisDirection::Antiparallel => {
                    Frame::new(second_origin, -frame.z(), frame.x()).unwrap()
                }
            },
        };
        let second = edit
            .create_cylinder(CylinderRequest::new(
                second_frame,
                second_radius,
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
    RelationFixture {
        session,
        part_id,
        first,
        second,
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

fn relation_section(fixture: &RelationFixture, first: BodyId, second: BodyId) -> BodySectionGraph {
    fixture
        .session
        .part(fixture.part_id.clone())
        .unwrap()
        .section_bodies(SectionBodiesRequest::new(first, second))
        .unwrap()
        .into_result()
        .unwrap()
}

fn assert_exterior_radial_section(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    first: &BodyId,
    second: &BodyId,
) {
    assert_eq!(graph.bodies(), &[first.clone(), second.clone()]);
    assert_exterior_radial_separation(part, graph, first, second);
    assert!(cylinder_cylinder_branch_indices(part, graph).is_empty());
    assert!(graph.gaps().iter().all(|gap| {
        gap.faces().len() != 2 || !gap.faces().iter().all(|face| face_is_cylinder(part, face))
    }));
}

fn assert_exterior_radial_separation(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    first: &BodyId,
    second: &BodyId,
) {
    let evidence = graph.cylinder_cylinder_exterior_radial_separations();
    assert_eq!(evidence.len(), 1, "missing or duplicate radial evidence");
    assert_eq!(
        evidence[0].faces(),
        &[
            cylinder_face(part, first.clone()),
            cylinder_face(part, second.clone()),
        ]
    );
}

fn cylinder_face(part: &kernel::Part<'_>, body: BodyId) -> kernel::FaceId {
    part.body(body)
        .unwrap()
        .faces()
        .unwrap()
        .find(|face| face_is_cylinder(part, face))
        .expect("cylinder body lost its analytic side face")
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

fn skew_carrier(branch: &SectionBranch) -> SectionSkewCylinderBranchCarrier {
    let SectionCarrier::SkewCylinderBranch(carrier) = branch.carrier() else {
        panic!("contained skew sheet escaped its certified procedural carrier")
    };
    carrier
}

fn skew_pcurve(branch: &SectionBranch, operand: usize) -> SectionSkewCylinderBranchPcurve {
    let SectionUvCurve::SkewCylinderBranch(pcurve) = branch.pcurves()[operand] else {
        panic!("contained skew sheet escaped its certified procedural pcurve")
    };
    pcurve
}

fn assert_contained_skew_branch(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    branch_index: usize,
    frame: Frame,
    operands_swapped: bool,
) -> i8 {
    let branch = &graph.branches()[branch_index];
    let carrier = skew_carrier(branch);
    assert_eq!(branch.topology(), SectionBranchTopology::Closed);
    assert_eq!(branch.endpoint_sites(), [0, 0]);
    assert_eq!(branch.fragment_sites().len(), 1);
    assert_eq!(carrier.range(), branch.range());
    let range = branch.range();
    assert!(range.is_finite() && range.lo < range.hi);

    let evidence = branch.evidence();
    assert!(evidence.tolerance().is_finite() && evidence.tolerance() > 0.0);
    assert!(
        evidence
            .residual_bounds()
            .into_iter()
            .all(|bound| bound.is_finite() && bound >= 0.0 && bound <= evidence.tolerance())
    );
    let pcurves = [skew_pcurve(branch, 0), skew_pcurve(branch, 1)];
    for pcurve in pcurves {
        assert_eq!(pcurve.range(), range);
        assert_eq!(pcurve.reversed(), carrier.reversed());
        assert_eq!(pcurve.source().sheet(), carrier.source().sheet());
    }
    let seam_site = branch.fragment_sites()[0];
    assert!(
        seam_site.point().dist(carrier.eval(range.lo)) <= evidence.tolerance(),
        "retained seam point disagrees with the Section-oriented carrier"
    );
    let seam_parameters = seam_site.surface_parameters();
    for operand in 0..2 {
        let expected = pcurves[operand].eval(range.lo);
        assert!(
            (seam_parameters[operand][0] - expected.x).abs() <= TOLERANCE
                && (seam_parameters[operand][1] - expected.y).abs() <= TOLERANCE,
            "retained seam parameters disagree with the Section-oriented pcurve"
        );
    }
    let expected_boundaries = if operands_swapped {
        [false, true]
    } else {
        [true, false]
    };
    assert_eq!(
        seam_site.surface_window_boundaries(),
        expected_boundaries,
        "the graph chart seam must remain attached to its caller-ordered source"
    );

    let carrier_parameter = |fraction: f64| {
        if carrier.reversed() {
            range.hi - range.width() * fraction
        } else {
            range.lo + range.width() * fraction
        }
    };
    let seam_local = frame.to_local(carrier.eval(carrier_parameter(0.0)));
    let sheet_sign = if seam_local.z < 0.0 { -1 } else { 1 };
    assert!((seam_local.z.abs() - SKEW_SECOND_RADIUS).abs() <= evidence.tolerance());

    for (fraction, sine, cosine, height) in [
        (0.0, 0.0, 1.0, 2.0),
        (0.25, 1.0, 0.0, 2.0 * ROOT_Y),
        (0.5, 0.0, -1.0, 2.0),
        (0.75, -1.0, 0.0, 2.0 * ROOT_Y),
        (1.0, 0.0, 1.0, 2.0),
    ] {
        let parameter = carrier_parameter(fraction);
        let derivatives = carrier.eval_derivs(parameter, 1);
        let point = derivatives.d[0];
        let expected = frame.origin()
            + frame.x() * cosine
            + frame.y() * sine
            + frame.z() * (f64::from(sheet_sign) * height);
        assert!(
            point.dist(expected) <= evidence.tolerance(),
            "procedural sheet disagrees with the perpendicular-cylinder oracle"
        );
        let canonical_tangent = frame.x() * -sine + frame.y() * cosine;
        let expected_tangent = if carrier.reversed() {
            -canonical_tangent
        } else {
            canonical_tangent
        };
        assert!(
            (derivatives.d[1] - expected_tangent).norm() <= evidence.tolerance(),
            "Section traversal escaped the independently oriented sheet tangent"
        );
        let first_cylinder_normal = frame.x() * cosine + frame.y() * sine;
        let second_cylinder_normal =
            (frame.y() * sine + frame.z() * (f64::from(sheet_sign) * height)) / SKEW_SECOND_RADIUS;
        let outward_cross = if operands_swapped {
            second_cylinder_normal.cross(first_cylinder_normal)
        } else {
            first_cylinder_normal.cross(second_cylinder_normal)
        };
        assert!(
            derivatives.d[1].dot(outward_cross) > TOLERANCE,
            "Section traversal opposes the first-outward-normal cross second-outward-normal rule"
        );

        for (operand, pcurve) in pcurves.iter().enumerate() {
            let uv = pcurve.eval(parameter);
            let face = part.face(branch.faces()[operand].clone()).unwrap();
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
                "procedural pcurve lift escaped its whole-range residual proof"
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
    assert!(matches!(
        fragments[0].span(),
        SectionCurveFragmentSpan::Whole
    ));
    sheet_sign
}

fn assert_contained_skew_graph(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    bodies: [BodyId; 2],
    frame: Frame,
    operands_swapped: bool,
    expected_sheet_signs: &[i8],
) {
    assert!(
        matches!(expected_sheet_signs.len(), 1 | 2),
        "contained skew publication must retain one or both ordered sheets"
    );
    let branch_count = expected_sheet_signs.len();
    assert_eq!(graph.bodies(), &bodies);
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty());
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert_eq!(graph.branches().len(), branch_count);
    assert!(graph.curve_endpoints().is_empty());
    assert_eq!(graph.curve_fragments().len(), branch_count);
    assert_eq!(graph.curve_components().len(), branch_count);
    assert_eq!(graph.rings().len(), branch_count);

    let signs = (0..branch_count)
        .map(|branch| assert_contained_skew_branch(part, graph, branch, frame, operands_swapped))
        .collect::<Vec<_>>();
    assert_eq!(
        signs, expected_sheet_signs,
        "retained sheets escaped their Lower-then-Upper order"
    );

    let mut component_fragments = graph
        .curve_components()
        .iter()
        .flat_map(|component| {
            assert!(component.closed());
            assert_eq!(component.fragments().len(), 1);
            component.fragments().iter().copied()
        })
        .collect::<Vec<_>>();
    component_fragments.sort_unstable();
    assert_eq!(component_fragments, (0..branch_count).collect::<Vec<_>>());
    let mut ring_branches = graph
        .rings()
        .iter()
        .map(|ring| ring.branch())
        .collect::<Vec<_>>();
    ring_branches.sort_unstable();
    assert_eq!(ring_branches, (0..branch_count).collect::<Vec<_>>());

    // Periodic embedding evidence is face-owned: one typed entry for each
    // cylinder operand, independent of whether one or both sheets survive.
    assert_eq!(graph.periodic_face_embeddings().len(), 2);
    let mut embedding_operands = Vec::new();
    for embedding in graph.periodic_face_embeddings() {
        embedding_operands.push(embedding.operand());
        let Some(SectionPeriodicEmbeddingGap::NonLinearCylinderPcurve { fragment }) =
            embedding.gap()
        else {
            panic!("nonlinear whole sheet retained unexpected periodic evidence: {embedding:?}")
        };
        let branch = &graph.branches()[graph.curve_fragments()[*fragment].branch()];
        assert_eq!(embedding.face(), branch.faces()[embedding.operand()]);
    }
    embedding_operands.sort_unstable();
    assert_eq!(embedding_operands, vec![0, 1]);
}

fn assert_contained_skew_swap_equivalence(forward: &BodySectionGraph, swapped: &BodySectionGraph) {
    assert_eq!(forward.branches().len(), swapped.branches().len());
    for branch_index in 0..forward.branches().len() {
        let original = &forward.branches()[branch_index];
        let exchanged = &swapped.branches()[branch_index];
        let original_carrier = skew_carrier(original);
        let exchanged_carrier = skew_carrier(exchanged);
        assert_eq!(original_carrier.source(), exchanged_carrier.source());
        assert_eq!(original_carrier.range(), exchanged_carrier.range());
        assert_ne!(original_carrier.reversed(), exchanged_carrier.reversed());
        assert_eq!(
            original.faces(),
            &[exchanged.faces()[1].clone(), exchanged.faces()[0].clone()]
        );
        assert_eq!(
            original.evidence().residual_bounds(),
            [
                exchanged.evidence().residual_bounds()[1],
                exchanged.evidence().residual_bounds()[0],
            ]
        );
        for operand in 0..2 {
            let original_pcurve = skew_pcurve(original, operand);
            let exchanged_pcurve = skew_pcurve(exchanged, 1 - operand);
            assert_eq!(original_pcurve.source(), exchanged_pcurve.source());
            assert_eq!(original_pcurve.range(), exchanged_pcurve.range());
            assert_ne!(original_pcurve.reversed(), exchanged_pcurve.reversed());
        }
    }
}

fn bounded_procedural_ends(graph: &BodySectionGraph) -> Vec<&SectionBoundedProceduralFragmentEnd> {
    graph
        .curve_fragments()
        .iter()
        .flat_map(|fragment| match fragment.span() {
            SectionCurveFragmentSpan::BoundedProcedural { endpoints } => {
                endpoints.iter().collect::<Vec<_>>()
            }
            _ => Vec::new(),
        })
        .collect()
}

fn assert_bounded_root_oracle(frame: Frame, point: Point3) {
    let point = frame.to_local(point);
    assert!((point.x * point.x + point.y * point.y - RADIUS * RADIUS).abs() <= TOLERANCE);
    assert!(
        (point.y * point.y + point.z * point.z - SKEW_SECOND_RADIUS * SKEW_SECOND_RADIUS).abs()
            <= TOLERANCE
    );
    assert!(
        (point.z - BOUNDED_SKEW_LOWER).abs() <= TOLERANCE
            || (point.z - BOUNDED_SKEW_UPPER).abs() <= TOLERANCE
    );
}

fn assert_bounded_procedural_fragment(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    fragment_index: usize,
    frame: Frame,
    bound_operand: usize,
    incidence: &mut [[usize; 2]],
    points: &mut [Vec<Point3>],
) {
    let fragment = &graph.curve_fragments()[fragment_index];
    let SectionCurveFragmentSpan::BoundedProcedural { endpoints } = fragment.span() else {
        panic!("bounded skew assertion received a nonprocedural fragment")
    };
    let branch = &graph.branches()[fragment.branch()];
    let carrier = skew_carrier(branch);
    assert_eq!(branch.topology(), SectionBranchTopology::Open);
    assert_eq!(branch.endpoint_sites(), [0, 1]);
    assert_eq!(branch.fragment_sites().len(), 2);
    assert_eq!(carrier.range(), branch.range());
    assert!(branch.range().is_finite() && branch.range().width() < core::f64::consts::TAU);
    assert_eq!(fragment.source_ordinal(), 0);
    let embedding = branch
        .embedding_certificate()
        .expect("bounded procedural branch lost its pcurve embedding");
    assert_eq!(embedding.range(), branch.range());
    assert_eq!(embedding.reversed(), carrier.reversed());
    assert_eq!(embedding.guarded_cell_count(), 256);
    assert_eq!(embedding.guarded_cell_work(), 1);
    assert_eq!(embedding.all_guarded_cells_work(), 256);
    assert_eq!(embedding.root_corridor_work(), 2);
    assert_eq!(embedding.total_work(), 260);
    assert!(
        embedding
            .guarded_cell(embedding.guarded_cell_count())
            .is_none()
    );

    let mut previous_hi = None;
    for cell_index in 0..embedding.guarded_cell_count() {
        let cell = embedding
            .guarded_cell(cell_index)
            .expect("fixed guarded cell failed to reissue");
        assert_eq!(cell.work(), embedding.guarded_cell_work());
        assert!(branch.range().contains(cell.parameter().lo()));
        assert!(branch.range().contains(cell.parameter().hi()));
        if let Some(previous_hi) = previous_hi {
            assert!(previous_hi >= cell.parameter().lo());
        }
        previous_hi = Some(cell.parameter().hi());
        for operand in 0..2 {
            let proof = &cell.pcurves()[operand];
            assert!(proof.stored_is_strictly_regular());
            assert!(proof.source_is_strictly_regular());
        }
    }
    assert_eq!(
        embedding.guarded_cell(0).unwrap().parameter().lo(),
        branch.range().lo
    );
    assert_eq!(
        embedding
            .guarded_cell(embedding.guarded_cell_count() - 1)
            .unwrap()
            .parameter()
            .hi(),
        branch.range().hi
    );

    for operand in 0..2 {
        let pcurve = skew_pcurve(branch, operand);
        let parameter = branch.range().lerp(0.5);
        let point = carrier.eval(parameter);
        let uv = pcurve.eval(parameter);
        let face = part.face(branch.faces()[operand].clone()).unwrap();
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
        assert!(point.dist(lifted) <= branch.evidence().tolerance());
    }

    for (slot, end) in endpoints.iter().enumerate() {
        incidence[end.endpoint()][1] += 1;
        points[end.endpoint()].push(end.root_point());
        let physical_root = end.physical_root();
        assert_eq!(physical_root.endpoint(), end.endpoint());
        assert_eq!(physical_root.point(), end.root_point());
        let expected_parameter = if slot == 0 {
            branch.range().lo
        } else {
            branch.range().hi
        };
        assert_eq!(
            end.inside_carrier_parameter().to_bits(),
            expected_parameter.to_bits()
        );
        assert!(
            end.inside_point()
                .dist(carrier.eval(end.inside_carrier_parameter()))
                <= branch.evidence().tolerance()
        );
        assert_eq!(branch.fragment_sites()[slot].point(), end.inside_point());
        assert_bounded_root_oracle(frame, end.root_point());
        assert_on_both_faces(part, branch, end.root_point(), "procedural physical root");
        assert_on_both_faces(
            part,
            branch,
            end.inside_point(),
            "procedural guarded endpoint",
        );
        let root = embedding
            .root_corridor(slot)
            .expect("directed physical-root corridor disappeared");
        assert_eq!(root.section_end(), slot);
        assert_eq!(root.work(), embedding.root_corridor_work());
        assert_eq!(physical_root.carrier_parameter(), root.root_parameter());
        assert!(
            root.corridor()
                .parameter()
                .contains(end.inside_carrier_parameter())
        );
        if slot == 0 {
            assert!(root.root_parameter().hi() < branch.range().lo);
        } else {
            assert!(root.root_parameter().lo() > branch.range().hi);
        }
        let adjacent = embedding
            .guarded_cell(if slot == 0 {
                0
            } else {
                embedding.guarded_cell_count() - 1
            })
            .unwrap();
        for operand in 0..2 {
            let uv = skew_pcurve(branch, operand).eval(end.inside_carrier_parameter());
            let uv = [uv.x, uv.y];
            for (coordinate, &uv_value) in uv.iter().enumerate() {
                assert!(
                    root.corridor().pcurves()[operand].stored_uv()[coordinate].contains(uv_value)
                );
                assert!(adjacent.pcurves()[operand].stored_uv()[coordinate].contains(uv_value));
            }
        }

        let trim = end.trim();
        assert_eq!(trim.operand(), bound_operand);
        assert_eq!(trim.face(), branch.faces()[bound_operand]);
        assert!(trim.carrier_root().lo().is_finite());
        assert!(trim.carrier_root().hi().is_finite());
        assert!(trim.carrier_root().lo() <= trim.carrier_root().hi());
        let observed_height = frame.to_local(end.root_point()).z;
        let authored_height = if (observed_height - BOUNDED_SKEW_LOWER).abs()
            < (observed_height - BOUNDED_SKEW_UPPER).abs()
        {
            0.0
        } else {
            BOUNDED_SKEW_UPPER - BOUNDED_SKEW_LOWER
        };
        assert!(root.root_pcurves()[bound_operand].stored_uv()[1].contains(authored_height));
        assert!(root.root_pcurves()[bound_operand].source_uv()[1].contains(authored_height));
        assert!(
            trim.edge_parameter()
                .contains(trim.source_parameter().root_parameter())
        );
        let public = &graph.curve_endpoints()[end.endpoint()];
        let SectionCurveEndpointTopology::Trim {
            sites,
            source_parameters,
        } = public.topology()
        else {
            panic!("bounded physical root became a parameter seam")
        };
        assert_eq!(
            sites[bound_operand],
            SectionSite::EdgeInterior(trim.source_parameter().edge())
        );
        assert_eq!(
            sites[1 - bound_operand],
            SectionSite::FaceInterior(branch.faces()[1 - bound_operand].clone())
        );
        assert_eq!(
            source_parameters[bound_operand].as_ref(),
            Some(trim.source_parameter())
        );
        assert!(source_parameters[1 - bound_operand].is_none());
        assert!(
            public.edge_parameters()[bound_operand]
                .expect("bound-owning endpoint requires an edge enclosure")
                .contains(trim.source_parameter().root_parameter())
        );
        assert!(public.edge_parameters()[1 - bound_operand].is_none());
    }
}

fn assert_bounded_ruling_fragment(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    fragment_index: usize,
    frame: Frame,
    bound_operand: usize,
    incidence: &mut [[usize; 2]],
    points: &mut [Vec<Point3>],
) {
    let fragment = &graph.curve_fragments()[fragment_index];
    let SectionCurveFragmentSpan::LineSegment { endpoints } = fragment.span() else {
        panic!("bounded skew assertion received a nonlinear ruling")
    };
    let branch = &graph.branches()[fragment.branch()];
    let (origin, direction) = line_carrier(branch);
    assert_eq!(branch.topology(), SectionBranchTopology::Open);
    assert_eq!(fragment.source_ordinal(), 0);
    for end in endpoints.iter() {
        incidence[end.endpoint()][0] += 1;
        points[end.endpoint()].push(end.point());
        assert!(
            end.point()
                .dist(origin + direction * end.carrier_parameter())
                <= TOLERANCE
        );
        assert_bounded_root_oracle(frame, end.point());
        assert_on_both_faces(part, branch, end.point(), "cap ruling root");
        assert_eq!(end.trims().iter().flatten().count(), 1);
        let trim = end.trims()[bound_operand]
            .as_ref()
            .expect("the bounded cylinder cap must own the ruling root");
        assert_eq!(trim.operand(), bound_operand);
        assert_eq!(trim.face(), branch.faces()[bound_operand]);
        let SectionCurveEndpointTopology::Trim {
            sites,
            source_parameters,
        } = graph.curve_endpoints()[end.endpoint()].topology()
        else {
            panic!("cap ruling root became a parameter seam")
        };
        assert_eq!(
            sites[bound_operand],
            SectionSite::EdgeInterior(trim.source_parameter().edge())
        );
        assert_eq!(
            source_parameters[bound_operand].as_ref(),
            Some(trim.source_parameter())
        );
        assert!(source_parameters[1 - bound_operand].is_none());
    }
}

fn assert_bounded_skew_graph(
    part: &kernel::Part<'_>,
    graph: &BodySectionGraph,
    bodies: [BodyId; 2],
    frame: Frame,
    bound_operand: usize,
) {
    assert_eq!(graph.bodies(), &bodies);
    assert_eq!(
        graph.completion(),
        SectionCompletion::Complete,
        "{graph:#?}"
    );
    assert!(graph.gaps().is_empty(), "{graph:#?}");
    assert!(graph.vertices().is_empty());
    assert!(graph.edges().is_empty());
    assert!(graph.loops().is_empty());
    assert!(graph.rings().is_empty());
    assert_eq!(graph.branches().len(), 8, "{graph:#?}");
    assert_eq!(graph.curve_endpoints().len(), 8, "{graph:#?}");
    assert_eq!(graph.curve_fragments().len(), 8, "{graph:#?}");
    assert_eq!(graph.curve_components().len(), 2, "{graph:#?}");
    assert_eq!(
        cylinder_cylinder_branch_indices(part, graph).len(),
        4,
        "{graph:#?}"
    );

    let mut incidence = vec![[0usize; 2]; graph.curve_endpoints().len()];
    let mut points = vec![Vec::new(); graph.curve_endpoints().len()];
    let mut procedural = 0usize;
    let mut rulings = 0usize;
    for fragment_index in 0..graph.curve_fragments().len() {
        match graph.curve_fragments()[fragment_index].span() {
            SectionCurveFragmentSpan::BoundedProcedural { .. } => {
                procedural += 1;
                assert_bounded_procedural_fragment(
                    part,
                    graph,
                    fragment_index,
                    frame,
                    bound_operand,
                    &mut incidence,
                    &mut points,
                );
            }
            SectionCurveFragmentSpan::LineSegment { .. } => {
                rulings += 1;
                assert_bounded_ruling_fragment(
                    part,
                    graph,
                    fragment_index,
                    frame,
                    bound_operand,
                    &mut incidence,
                    &mut points,
                );
            }
            other => panic!("bounded skew cycle retained an unexpected fragment: {other:?}"),
        }
    }
    assert_eq!((procedural, rulings), (4, 4));
    assert!(incidence.into_iter().all(|count| count == [1, 1]));
    for occurrences in points {
        let [first, second] = occurrences.as_slice() else {
            panic!("each exact root must have one procedural and one ruling representative")
        };
        assert!(first.dist(*second) <= 1.0e-8);
    }

    let mut component_fragments = Vec::new();
    for component in graph.curve_components() {
        assert!(component.closed());
        assert_eq!(component.fragments().len(), 4);
        for index in 0..component.fragments().len() {
            let current = &graph.curve_fragments()[component.fragments()[index]];
            let next = &graph.curve_fragments()
                [component.fragments()[(index + 1) % component.fragments().len()]];
            assert_ne!(
                matches!(
                    current.span(),
                    SectionCurveFragmentSpan::BoundedProcedural { .. }
                ),
                matches!(
                    next.span(),
                    SectionCurveFragmentSpan::BoundedProcedural { .. }
                )
            );
        }
        component_fragments.extend_from_slice(component.fragments());
    }
    component_fragments.sort_unstable();
    assert_eq!(component_fragments, (0..8).collect::<Vec<_>>());

    let mut edge_roots = Vec::<(kernel::EdgeId, Vec<usize>)>::new();
    for endpoint in graph.curve_endpoints() {
        let SectionCurveEndpointTopology::Trim {
            source_parameters, ..
        } = endpoint.topology()
        else {
            panic!("bounded skew endpoint became a seam")
        };
        let root = source_parameters[bound_operand]
            .as_ref()
            .expect("bound-owning source root disappeared");
        if let Some((_, roots)) = edge_roots.iter_mut().find(|(edge, _)| *edge == root.edge()) {
            roots.push(root.root_ordinal());
        } else {
            edge_roots.push((root.edge(), vec![root.root_ordinal()]));
        }
    }
    assert_eq!(edge_roots.len(), 2);
    for (_, mut roots) in edge_roots {
        roots.sort_unstable();
        assert_eq!(roots, vec![0, 1, 2, 3]);
    }
}

fn assert_bounded_skew_swap_equivalence(forward: &BodySectionGraph, swapped: &BodySectionGraph) {
    let forward_ends = bounded_procedural_ends(forward);
    let swapped_ends = bounded_procedural_ends(swapped);
    assert_eq!(forward_ends.len(), 8);
    assert_eq!(swapped_ends.len(), 8);
    for original in forward_ends {
        let exchanged = swapped_ends
            .iter()
            .copied()
            .find(|candidate| {
                candidate.trim().source_parameter() == original.trim().source_parameter()
            })
            .expect("operand swap lost an exact source-ring root");
        assert_eq!(original.trim().operand(), 0);
        assert_eq!(exchanged.trim().operand(), 1);
        assert_eq!(original.trim().face(), exchanged.trim().face());
        assert_eq!(original.trim().loop_id(), exchanged.trim().loop_id());
        assert_eq!(original.trim().fin(), exchanged.trim().fin());
        assert_eq!(original.root_point(), exchanged.root_point());
        assert_eq!(original.inside_point(), exchanged.inside_point());
        assert_eq!(
            original.trim().carrier_root(),
            exchanged.trim().carrier_root()
        );
        assert_eq!(
            original
                .trim()
                .source_parameter()
                .root_parameter()
                .to_bits(),
            exchanged
                .trim()
                .source_parameter()
                .root_parameter()
                .to_bits()
        );
        assert_eq!(
            original
                .trim()
                .source_parameter()
                .root_parameter_enclosure(),
            exchanged
                .trim()
                .source_parameter()
                .root_parameter_enclosure()
        );
    }
}

#[test]
fn bounded_skew_spans_close_with_cap_rulings_and_topology_owned_roots() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = bounded_skew_fixture(placement);
        let forward = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(forward, replay, "serial bounded-skew replay changed");

        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        assert_bounded_skew_graph(
            &part,
            &forward,
            [fixture.first.clone(), fixture.second.clone()],
            fixture.frame,
            0,
        );
        assert_bounded_skew_graph(
            &part,
            &swapped,
            [fixture.second.clone(), fixture.first.clone()],
            fixture.frame,
            1,
        );
        assert_bounded_skew_swap_equivalence(&forward, &swapped);
        drop(part);

        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.first,
                &fixture.second,
            ),
            fixture.before,
            "bounded-skew Section mutated a source body"
        );
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
fn bounded_skew_section_work_accepts_n_and_refuses_n_minus_one_atomically() {
    let fixture = bounded_skew_fixture(Placement::World);
    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    let request = || SectionBodiesRequest::new(fixture.first.clone(), fixture.second.clone());
    let baseline = part.section_bodies(request()).unwrap();
    let expected_graph = baseline.result().unwrap().clone();
    assert_eq!(expected_graph.completion(), SectionCompletion::Complete);
    let usage = *baseline
        .report()
        .usage()
        .iter()
        .find(|usage| usage.stage == SECTION_WORK && usage.resource == ResourceKind::Work)
        .expect("bounded-skew Section must retain exact work usage");
    assert_eq!(
        usage.consumed, 67_811,
        "bounded-skew Section work drifted from its exact facade frontier"
    );

    let run_at = |allowed| {
        let plan = BudgetPlan::new([LimitSpec::new(
            SECTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap();
        part.section_bodies(
            request().with_settings(OperationSettings::new().with_budget_overrides(plan)),
        )
        .unwrap()
    };
    let admitted = run_at(usage.consumed);
    assert_eq!(admitted.result().unwrap(), &expected_graph);
    assert!(admitted.report().limit_events().is_empty());

    let denied = run_at(usage.consumed - 1);
    let expected = kernel::LimitSnapshot {
        allowed: usage.consumed - 1,
        ..usage
    };
    assert_eq!(denied.result().unwrap_err().limit(), Some(expected));
    assert_eq!(denied.report().limit_events(), &[expected]);
    drop(part);
    assert_eq!(
        source_signature(
            &fixture.session,
            &fixture.part_id,
            &fixture.first,
            &fixture.second,
        ),
        fixture.before,
        "bounded-skew budget denial mutated a source body"
    );
}

#[test]
fn contained_skew_two_sheet_section_is_complete_read_only_and_transform_stable() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture = skew_two_sheet_fixture(placement, false);
        let forward = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(forward, replay, "serial contained-skew replay changed");

        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        assert_contained_skew_graph(
            &part,
            &forward,
            [fixture.first.clone(), fixture.second.clone()],
            fixture.frame,
            false,
            &[-1, 1],
        );
        assert_contained_skew_graph(
            &part,
            &swapped,
            [fixture.second.clone(), fixture.first.clone()],
            fixture.frame,
            true,
            &[-1, 1],
        );
        assert_contained_skew_swap_equivalence(&forward, &swapped);
        drop(part);

        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.first,
                &fixture.second,
            ),
            fixture.before,
            "contained-skew Section mutated a source body"
        );
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
fn narrow_skew_height_publishes_one_complete_lower_sheet_read_only() {
    let fixture = skew_two_sheet_fixture(Placement::World, true);
    let forward = section(&fixture, fixture.first.clone(), fixture.second.clone());
    let replay = section(&fixture, fixture.first.clone(), fixture.second.clone());
    let swapped = section(&fixture, fixture.second.clone(), fixture.first.clone());
    assert_eq!(forward, replay, "serial narrow-skew replay changed");

    let part = fixture.session.part(fixture.part_id.clone()).unwrap();
    assert_contained_skew_graph(
        &part,
        &forward,
        [fixture.first.clone(), fixture.second.clone()],
        fixture.frame,
        false,
        &[-1],
    );
    assert_contained_skew_graph(
        &part,
        &swapped,
        [fixture.second.clone(), fixture.first.clone()],
        fixture.frame,
        true,
        &[-1],
    );
    assert_contained_skew_swap_equivalence(&forward, &swapped);
    drop(part);

    assert_eq!(
        source_signature(
            &fixture.session,
            &fixture.part_id,
            &fixture.first,
            &fixture.second,
        ),
        fixture.before,
        "single-sheet skew Section mutated a source body"
    );
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
fn certified_exterior_radial_miss_retains_witness_under_rigid_and_order_transforms() {
    for relation in [
        RadialRelation::ExteriorDiagonal,
        RadialRelation::ExteriorBroadPhase,
    ] {
        for placement in [Placement::World, Placement::Oblique] {
            for direction in [AxisDirection::Parallel, AxisDirection::Antiparallel] {
                let fixture = relation_fixture(placement, direction, relation);
                let forward =
                    relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
                let replay =
                    relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
                let swapped =
                    relation_section(&fixture, fixture.second.clone(), fixture.first.clone());
                assert_eq!(
                    forward, replay,
                    "serial exterior-miss replay changed for {relation:?} {placement:?} {direction:?}"
                );
                let part = fixture.session.part(fixture.part_id.clone()).unwrap();
                assert_exterior_radial_section(&part, &forward, &fixture.first, &fixture.second);
                assert_exterior_radial_section(&part, &swapped, &fixture.second, &fixture.first);
                drop(part);
                assert_eq!(
                    source_signature(
                        &fixture.session,
                        &fixture.part_id,
                        &fixture.first,
                        &fixture.second,
                    ),
                    fixture.before,
                    "exterior-miss Section mutated a source for {relation:?} {placement:?} {direction:?}"
                );
            }
        }
    }
}

#[test]
fn one_ulp_exterior_radial_witness_survives_unrelated_section_gaps() {
    for direction in [AxisDirection::Parallel, AxisDirection::Antiparallel] {
        let fixture = relation_fixture(Placement::World, direction, RadialRelation::ExteriorOneUlp);
        let forward = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = relation_section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(
            forward, replay,
            "serial one-ULP replay changed for {direction:?}"
        );
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        assert_exterior_radial_section(&part, &forward, &fixture.first, &fixture.second);
        assert_exterior_radial_section(&part, &swapped, &fixture.second, &fixture.first);
        drop(part);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.first,
                &fixture.second,
            ),
            fixture.before,
            "one-ULP Section mutated a source for {direction:?}"
        );
    }
}

#[test]
fn exact_skew_discriminant_miss_is_complete_read_only_and_swap_stable() {
    for placement in [Placement::World, Placement::Oblique] {
        let fixture =
            relation_fixture(placement, AxisDirection::Parallel, RadialRelation::SkewMiss);
        let forward = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = relation_section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(forward, replay, "serial skew-miss replay changed");

        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        for candidate in [&forward, &swapped] {
            assert_eq!(candidate.completion(), SectionCompletion::Complete);
            assert!(candidate.gaps().is_empty());
            assert!(candidate.branches().is_empty());
            assert!(candidate.curve_fragments().is_empty());
            assert!(cylinder_cylinder_branch_indices(&part, candidate).is_empty());
            let [miss] = candidate.skew_cylinder_strict_discriminant_misses() else {
                panic!("missing or duplicate propagated skew-discriminant witness");
            };
            assert_eq!(
                miss.faces(),
                &[
                    cylinder_face(&part, candidate.bodies()[0].clone()),
                    cylinder_face(&part, candidate.bodies()[1].clone()),
                ]
            );
            assert!(
                candidate
                    .cylinder_cylinder_exterior_radial_separations()
                    .is_empty(),
                "a skew miss acquired parallel radial evidence"
            );
        }
        drop(part);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.first,
                &fixture.second,
            ),
            fixture.before,
            "skew-miss Section mutated a source body"
        );
    }
}

#[test]
fn unsupported_cylinder_relations_remain_one_typed_gap_without_fallback_duplicates() {
    for relation in [
        RadialRelation::Tangent,
        RadialRelation::Internal,
        RadialRelation::Coincident,
        RadialRelation::Skew,
    ] {
        let fixture = relation_fixture(Placement::World, AxisDirection::Parallel, relation);
        let graph = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let replay = relation_section(&fixture, fixture.first.clone(), fixture.second.clone());
        let swapped = relation_section(&fixture, fixture.second.clone(), fixture.first.clone());
        assert_eq!(graph, replay, "serial replay changed for {relation:?}");
        let part = fixture.session.part(fixture.part_id.clone()).unwrap();
        for candidate in [&graph, &swapped] {
            assert_eq!(
                candidate.completion(),
                SectionCompletion::Indeterminate,
                "unsupported relation escaped as complete: {relation:?} {candidate:#?}"
            );
            assert!(cylinder_cylinder_branch_indices(&part, candidate).is_empty());
            assert!(
                candidate
                    .cylinder_cylinder_exterior_radial_separations()
                    .is_empty(),
                "unsupported relation acquired exterior radial evidence: {relation:?}"
            );
            let cylinder_pair_gaps = candidate
                .gaps()
                .iter()
                .filter(|gap| {
                    gap.faces().len() == 2
                        && gap.faces().iter().all(|face| face_is_cylinder(&part, face))
                })
                .collect::<Vec<_>>();
            assert_eq!(
                cylinder_pair_gaps.len(),
                1,
                "the curved owner and planar fallback duplicated {relation:?}: {:#?}",
                candidate.gaps()
            );
            assert_eq!(
                cylinder_pair_gaps[0].reason(),
                "a candidate face pair returned an indeterminate intersection result"
            );
        }
        drop(part);
        assert_eq!(
            source_signature(
                &fixture.session,
                &fixture.part_id,
                &fixture.first,
                &fixture.second,
            ),
            fixture.before,
            "unsupported Section mutated a source for {relation:?}"
        );
    }
}
