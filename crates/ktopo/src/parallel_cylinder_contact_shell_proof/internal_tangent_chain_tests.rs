use super::*;
use crate::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellOutput, AnalyticShellPcurve, AnalyticShellSurface,
    AnalyticShellVertex, AnalyticVertexKey,
};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::{Body, Edge, Face, FaceDomain, Fin, Loop, Region, Shell, Vertex};
use crate::transaction::{FullCommitRequirement, Transaction};
use kcore::operation::{
    ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Vec2};
use kgraph::AffineParamMap1d;

const TOLERANCE: f64 = 1.0e-12;
const LOW_TANGENT: AnalyticVertexKey = AnalyticVertexKey::new(0);
const HIGH_TANGENT: AnalyticVertexKey = AnalyticVertexKey::new(1);

const LOW_FAR: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const LOW_INNER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const LOW_OUTER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
const HIGH_OUTER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(3);
const HIGH_INNER_CONTACT: AnalyticEdgeKey = AnalyticEdgeKey::new(4);
const HIGH_FAR: AnalyticEdgeKey = AnalyticEdgeKey::new(5);

const LOW_INNER_FACE: AnalyticFaceKey = AnalyticFaceKey::new(0);
const OUTER_FACE: AnalyticFaceKey = AnalyticFaceKey::new(1);
const HIGH_INNER_FACE: AnalyticFaceKey = AnalyticFaceKey::new(2);
const LOW_CAP_FACE: AnalyticFaceKey = AnalyticFaceKey::new(3);
const HIGH_CAP_FACE: AnalyticFaceKey = AnalyticFaceKey::new(4);
const LOW_SHOULDER_FACE: AnalyticFaceKey = AnalyticFaceKey::new(5);
const HIGH_SHOULDER_FACE: AnalyticFaceKey = AnalyticFaceKey::new(6);

fn map(scale: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, 0.0).unwrap()
}

fn side_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    axial_parameter: f64,
    endpoint_free: bool,
) -> AnalyticShellFin {
    let use_ = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(
            Line2d::new(Point2::new(0.0, axial_parameter), Vec2::new(1.0, 0.0)).unwrap(),
        ),
        map(1.0),
    );
    AnalyticShellFin::new(
        edge,
        sense,
        if endpoint_free {
            use_.with_closure_winding([1, 0])
        } else {
            use_
        },
    )
}

fn plane_fin(
    edge: AnalyticEdgeKey,
    sense: Sense,
    plane: Plane,
    circle: Circle,
    endpoint_free: bool,
) -> AnalyticShellFin {
    let center = plane.frame().to_local(circle.frame().origin());
    let x = Vec2::new(
        circle.frame().x().dot(plane.frame().x()),
        circle.frame().x().dot(plane.frame().y()),
    );
    let y = Vec2::new(
        circle.frame().y().dot(plane.frame().x()),
        circle.frame().y().dot(plane.frame().y()),
    );
    let use_ = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Circle(
            Circle2d::new(Point2::new(center.x, center.y), circle.radius(), x).unwrap(),
        ),
        map(if x.perp().dot(y) > 0.0 { 1.0 } else { -1.0 }),
    );
    AnalyticShellFin::new(
        edge,
        sense,
        if endpoint_free {
            use_.with_closure_winding([0, 0])
        } else {
            use_
        },
    )
}

fn oblique_frame() -> Frame {
    Frame::new(
        Point3::new(0.5, 0.0, 0.0),
        Vec3::new(0.0, 0.28, 0.96),
        Vec3::new(1.0, 0.0, 0.0),
    )
    .unwrap()
}

fn axis_frame(frame: Frame, origin: Point3, reversed: bool) -> Frame {
    if reversed {
        Frame::new(origin, -frame.z(), frame.x()).unwrap()
    } else {
        frame.with_origin(origin)
    }
}

fn boundary_senses(first: f64, second: f64) -> (Sense, Sense) {
    if first < second {
        (Sense::Forward, Sense::Reversed)
    } else {
        (Sense::Reversed, Sense::Forward)
    }
}

#[derive(Clone, Copy)]
struct ChainFixture {
    cylinders: [Cylinder; 3],
    circles: [Circle; 6],
    planes: [Plane; 4],
    axial: [f64; 3],
    senses: [[Sense; 2]; 3],
}

fn chain_fixture(frame: Frame, reversed: [bool; 3], high_far_axial: f64) -> ChainFixture {
    let [low_reversed, outer_reversed, high_reversed] = reversed;
    let low_outer_center = frame.point_at(0.0, 0.0, -1.0);
    let high_outer_center = frame.point_at(0.0, 0.0, 1.0);
    let low_inner_center = frame.point_at(1.0, 0.0, -1.0);
    let high_inner_center = frame.point_at(1.0, 0.0, 1.0);
    let low_far_center = frame.point_at(1.0, 0.0, -2.0);
    let high_far_center = frame.point_at(1.0, 0.0, high_far_axial);

    let low_frame = axis_frame(frame, low_inner_center, low_reversed);
    let outer_frame = axis_frame(frame, low_outer_center, outer_reversed);
    let high_frame = axis_frame(frame, high_inner_center, high_reversed);
    let low_cylinder = Cylinder::new(low_frame, 1.0).unwrap();
    let outer_cylinder = Cylinder::new(outer_frame, 2.0).unwrap();
    let high_cylinder = Cylinder::new(high_frame, 1.0).unwrap();

    let low_far_circle = Circle::new(low_frame.with_origin(low_far_center), 1.0).unwrap();
    let low_inner_circle = Circle::new(low_frame, 1.0).unwrap();
    let low_outer_circle = Circle::new(outer_frame, 2.0).unwrap();
    let high_outer_circle = Circle::new(outer_frame.with_origin(high_outer_center), 2.0).unwrap();
    let high_inner_circle = Circle::new(high_frame, 1.0).unwrap();
    let high_far_circle = Circle::new(high_frame.with_origin(high_far_center), 1.0).unwrap();

    let low_plane = Plane::new(Frame::new(low_outer_center, -frame.z(), frame.x()).unwrap());
    let high_plane = Plane::new(frame.with_origin(high_outer_center));
    let low_cap_plane = Plane::new(Frame::new(low_far_center, -frame.z(), frame.x()).unwrap());
    let high_cap_axis = if high_far_axial > 1.0 {
        frame.z()
    } else {
        -frame.z()
    };
    let high_cap_plane = Plane::new(Frame::new(high_far_center, high_cap_axis, frame.x()).unwrap());

    let low_far_v: f64 = if low_reversed { 1.0 } else { -1.0 };
    let outer_high_v: f64 = if outer_reversed { -2.0 } else { 2.0 };
    let high_delta = high_far_axial - 1.0;
    let high_far_v: f64 = if high_reversed {
        -high_delta
    } else {
        high_delta
    };
    let (low_far_sense, low_contact_sense) = boundary_senses(low_far_v, 0.0);
    let (outer_low_sense, outer_high_sense) = boundary_senses(0.0, outer_high_v);
    let (high_contact_sense, high_far_sense) = boundary_senses(0.0, high_far_v);
    ChainFixture {
        cylinders: [low_cylinder, outer_cylinder, high_cylinder],
        circles: [
            low_far_circle,
            low_inner_circle,
            low_outer_circle,
            high_outer_circle,
            high_inner_circle,
            high_far_circle,
        ],
        planes: [low_cap_plane, high_cap_plane, low_plane, high_plane],
        axial: [low_far_v, outer_high_v, high_far_v],
        senses: [
            [low_far_sense, low_contact_sense],
            [outer_low_sense, outer_high_sense],
            [high_contact_sense, high_far_sense],
        ],
    }
}

fn chain_side_faces(fixture: ChainFixture) -> Vec<AnalyticShellFace> {
    let [low_cylinder, outer_cylinder, high_cylinder] = fixture.cylinders;
    let [low_far_v, outer_high_v, high_far_v] = fixture.axial;
    let [
        [low_far_sense, low_contact_sense],
        [outer_low_sense, outer_high_sense],
        [high_contact_sense, high_far_sense],
    ] = fixture.senses;
    let tau = core::f64::consts::TAU;

    vec![
        AnalyticShellFace::new(
            LOW_INNER_FACE,
            AnalyticShellSurface::Cylinder(low_cylinder),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, tau, low_far_v.min(0.0), low_far_v.max(0.0)).unwrap(),
            vec![
                AnalyticShellLoop::new(vec![side_fin(LOW_FAR, low_far_sense, low_far_v, true)]),
                AnalyticShellLoop::new(vec![side_fin(
                    LOW_INNER_CONTACT,
                    low_contact_sense,
                    0.0,
                    false,
                )]),
            ],
        ),
        AnalyticShellFace::new(
            OUTER_FACE,
            AnalyticShellSurface::Cylinder(outer_cylinder),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, tau, outer_high_v.min(0.0), outer_high_v.max(0.0))
                .unwrap(),
            vec![
                AnalyticShellLoop::new(vec![side_fin(
                    LOW_OUTER_CONTACT,
                    outer_low_sense,
                    0.0,
                    false,
                )]),
                AnalyticShellLoop::new(vec![side_fin(
                    HIGH_OUTER_CONTACT,
                    outer_high_sense,
                    outer_high_v,
                    false,
                )]),
            ],
        ),
        AnalyticShellFace::new(
            HIGH_INNER_FACE,
            AnalyticShellSurface::Cylinder(high_cylinder),
            Sense::Forward,
            FaceDomain::from_bounds(0.0, tau, high_far_v.min(0.0), high_far_v.max(0.0)).unwrap(),
            vec![
                AnalyticShellLoop::new(vec![side_fin(
                    HIGH_INNER_CONTACT,
                    high_contact_sense,
                    0.0,
                    false,
                )]),
                AnalyticShellLoop::new(vec![side_fin(HIGH_FAR, high_far_sense, high_far_v, true)]),
            ],
        ),
    ]
}

fn chain_cap_faces(fixture: ChainFixture) -> Vec<AnalyticShellFace> {
    let [low_far_circle, _, _, _, _, high_far_circle] = fixture.circles;
    let [low_cap_plane, high_cap_plane, _, _] = fixture.planes;
    let [[low_far_sense, _], _, [_, high_far_sense]] = fixture.senses;
    vec![
        AnalyticShellFace::new(
            LOW_CAP_FACE,
            AnalyticShellSurface::Plane(low_cap_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-1.1, 1.1, -1.1, 1.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![plane_fin(
                LOW_FAR,
                low_far_sense.flipped(),
                low_cap_plane,
                low_far_circle,
                true,
            )])],
        ),
        AnalyticShellFace::new(
            HIGH_CAP_FACE,
            AnalyticShellSurface::Plane(high_cap_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-1.1, 1.1, -1.1, 1.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![plane_fin(
                HIGH_FAR,
                high_far_sense.flipped(),
                high_cap_plane,
                high_far_circle,
                true,
            )])],
        ),
    ]
}

fn chain_shoulder_faces(fixture: ChainFixture) -> Vec<AnalyticShellFace> {
    let [
        _,
        low_inner_circle,
        low_outer_circle,
        high_outer_circle,
        high_inner_circle,
        _,
    ] = fixture.circles;
    let [_, _, low_plane, high_plane] = fixture.planes;
    let [
        [_, low_contact_sense],
        [outer_low_sense, outer_high_sense],
        [high_contact_sense, _],
    ] = fixture.senses;
    vec![
        AnalyticShellFace::new(
            LOW_SHOULDER_FACE,
            AnalyticShellSurface::Plane(low_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                plane_fin(
                    LOW_OUTER_CONTACT,
                    outer_low_sense.flipped(),
                    low_plane,
                    low_outer_circle,
                    false,
                ),
                plane_fin(
                    LOW_INNER_CONTACT,
                    low_contact_sense.flipped(),
                    low_plane,
                    low_inner_circle,
                    false,
                ),
            ])],
        ),
        AnalyticShellFace::new(
            HIGH_SHOULDER_FACE,
            AnalyticShellSurface::Plane(high_plane),
            Sense::Forward,
            FaceDomain::from_bounds(-2.1, 2.1, -2.1, 2.1).unwrap(),
            vec![AnalyticShellLoop::new(vec![
                plane_fin(
                    HIGH_OUTER_CONTACT,
                    outer_high_sense.flipped(),
                    high_plane,
                    high_outer_circle,
                    false,
                ),
                plane_fin(
                    HIGH_INNER_CONTACT,
                    high_contact_sense.flipped(),
                    high_plane,
                    high_inner_circle,
                    false,
                ),
            ])],
        ),
    ]
}

fn chain_contact_edges(fixture: ChainFixture) -> Vec<AnalyticShellEdge> {
    let [
        _,
        low_inner_circle,
        low_outer_circle,
        high_outer_circle,
        high_inner_circle,
        _,
    ] = fixture.circles;
    vec![
        AnalyticShellEdge::new(
            LOW_INNER_CONTACT,
            [LOW_TANGENT, LOW_TANGENT],
            AnalyticShellCurve::Circle(low_inner_circle),
            low_inner_circle.param_range(),
        ),
        AnalyticShellEdge::new(
            LOW_OUTER_CONTACT,
            [LOW_TANGENT, LOW_TANGENT],
            AnalyticShellCurve::Circle(low_outer_circle),
            low_outer_circle.param_range(),
        ),
        AnalyticShellEdge::new(
            HIGH_OUTER_CONTACT,
            [HIGH_TANGENT, HIGH_TANGENT],
            AnalyticShellCurve::Circle(high_outer_circle),
            high_outer_circle.param_range(),
        ),
        AnalyticShellEdge::new(
            HIGH_INNER_CONTACT,
            [HIGH_TANGENT, HIGH_TANGENT],
            AnalyticShellCurve::Circle(high_inner_circle),
            high_inner_circle.param_range(),
        ),
    ]
}

fn chain_closed_edges(fixture: ChainFixture) -> Vec<AnalyticShellClosedEdge> {
    let [low_far_circle, _, _, _, _, high_far_circle] = fixture.circles;
    vec![
        AnalyticShellClosedEdge::new(
            LOW_FAR,
            AnalyticShellCurve::Circle(low_far_circle),
            low_far_circle.param_range(),
        ),
        AnalyticShellClosedEdge::new(
            HIGH_FAR,
            AnalyticShellCurve::Circle(high_far_circle),
            high_far_circle.param_range(),
        ),
    ]
}

fn two_shoulder_input_with_high_far(
    frame: Frame,
    reversed: [bool; 3],
    declaration_variant: usize,
    high_far_axial: f64,
) -> AnalyticShellInput {
    let fixture = chain_fixture(frame, reversed, high_far_axial);
    let mut faces = chain_side_faces(fixture);
    faces.extend(chain_cap_faces(fixture));
    faces.extend(chain_shoulder_faces(fixture));
    let mut edges = chain_contact_edges(fixture);
    let mut closed_edges = chain_closed_edges(fixture);
    match declaration_variant % 3 {
        1 => {
            faces.reverse();
            edges.reverse();
            closed_edges.reverse();
        }
        2 => {
            faces.rotate_left(3);
            edges.rotate_left(1);
            closed_edges.rotate_left(1);
        }
        _ => {}
    }
    let [_, _, low_outer_circle, high_outer_circle, _, _] = fixture.circles;
    AnalyticShellInput::new(
        vec![
            AnalyticShellVertex::new(LOW_TANGENT, low_outer_circle.eval(0.0)),
            AnalyticShellVertex::new(HIGH_TANGENT, high_outer_circle.eval(0.0)),
        ],
        edges,
        faces,
    )
    .with_closed_edges(closed_edges)
}

fn two_shoulder_input(
    frame: Frame,
    reversed: [bool; 3],
    declaration_variant: usize,
) -> AnalyticShellInput {
    two_shoulder_input_with_high_far(frame, reversed, declaration_variant, 2.0)
}

fn face_for_key(output: &AnalyticShellOutput, key: AnalyticFaceKey) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (*candidate == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &AnalyticShellOutput, key: AnalyticEdgeKey) -> EdgeId {
    output
        .edges()
        .iter()
        .find_map(|(candidate, edge)| (*candidate == key).then_some(*edge))
        .unwrap()
}

fn permute_realized_order(
    transaction: &mut Transaction<'_>,
    output: &AnalyticShellOutput,
    variant: usize,
) {
    match variant % 3 {
        1 => transaction
            .store_mut()
            .get_mut(output.shell())
            .unwrap()
            .faces
            .reverse(),
        2 => transaction
            .store_mut()
            .get_mut(output.shell())
            .unwrap()
            .faces
            .rotate_left(2),
        _ => return,
    }
    for key in [LOW_INNER_FACE, OUTER_FACE, HIGH_INNER_FACE] {
        transaction
            .store_mut()
            .get_mut(face_for_key(output, key))
            .unwrap()
            .loops
            .reverse();
    }
    for key in [LOW_SHOULDER_FACE, HIGH_SHOULDER_FACE] {
        let face = face_for_key(output, key);
        let loop_id = transaction.store().get(face).unwrap().loops[0];
        transaction
            .store_mut()
            .get_mut(loop_id)
            .unwrap()
            .fins
            .reverse();
    }
}

fn positive() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Positive,
    })
}

fn invalid() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Invalid,
    })
}

fn assert_not_full_certified(store: &Store, output: &AnalyticShellOutput) {
    assert_ne!(
        crate::shell_proof::certify_shell(
            store,
            output.shell(),
            crate::entity::BodyKind::Solid,
            crate::entity::RegionKind::Solid,
        )
        .unwrap(),
        positive().unwrap(),
    );
    let report = check_body_report(store, output.body(), CheckLevel::Full).unwrap();
    assert_ne!(report.outcome(), CheckOutcome::Valid, "{report:#?}");
}

#[test]
fn two_shoulder_chain_is_full_valid_across_frames_axes_and_orders() {
    for frame in [Frame::world(), oblique_frame()] {
        for mask in 0_u8..8 {
            let reversed = [mask & 1 != 0, mask & 2 != 0, mask & 4 != 0];
            for variant in 0..3 {
                let mut store = Store::new();
                let mut transaction = store.transaction().unwrap();
                let output = transaction
                    .assemble_analytic_shell(
                        &two_shoulder_input(frame, reversed, variant),
                        TOLERANCE,
                    )
                    .unwrap();
                permute_realized_order(&mut transaction, &output, variant);
                assert_eq!(output.faces().len(), 7);
                assert_eq!(output.edges().len(), 6);
                assert_eq!(output.vertices().len(), 2);
                let fast = check_body_report(transaction.store(), output.body(), CheckLevel::Fast)
                    .unwrap();
                assert_eq!(fast.outcome(), CheckOutcome::Valid, "{fast:#?}");
                let full = check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                    .unwrap();
                assert_eq!(
                    full.outcome(),
                    CheckOutcome::Valid,
                    "frame={frame:?}, reversed={reversed:?}, variant={variant}: {full:#?}",
                );
                transaction
                    .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                    .unwrap();
            }
        }
    }
}

#[test]
fn two_shoulder_geometry_tampers_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &two_shoulder_input(Frame::world(), [false; 3], 0),
            TOLERANCE,
        )
        .unwrap();

    let mut shifted_circle = transaction.store().clone();
    let mut edit = shifted_circle.transaction().unwrap();
    let edge = edge_for_key(&output, HIGH_INNER_CONTACT);
    let curve_id = edit.store().get(edge).unwrap().curve.unwrap();
    let CurveGeom::Circle(circle) = *edit.store().get(curve_id).unwrap() else {
        panic!("contact edge must be circular")
    };
    let origin = circle.frame().origin();
    let shifted = Circle::new(
        circle
            .frame()
            .with_origin(Point3::new(origin.x.next_up(), origin.y, origin.z)),
        circle.radius(),
    )
    .unwrap();
    edit.store_mut()
        .replace_curve(curve_id, CurveGeom::Circle(shifted))
        .unwrap();
    assert_not_full_certified(edit.store(), &output);

    let mut changed_radius = transaction.store().clone();
    let mut edit = changed_radius.transaction().unwrap();
    let face = face_for_key(&output, OUTER_FACE);
    let surface_id = edit.store().get(face).unwrap().surface;
    let SurfaceGeom::Cylinder(cylinder) = *edit.store().get(surface_id).unwrap() else {
        panic!("middle face must be cylindrical")
    };
    let changed = Cylinder::new(*cylinder.frame(), cylinder.radius().next_up()).unwrap();
    edit.store_mut()
        .replace_surface(surface_id, SurfaceGeom::Cylinder(changed))
        .unwrap();
    assert_not_full_certified(edit.store(), &output);

    let mut shifted_axis = transaction.store().clone();
    let mut edit = shifted_axis.transaction().unwrap();
    let face = face_for_key(&output, HIGH_INNER_FACE);
    let surface_id = edit.store().get(face).unwrap().surface;
    let SurfaceGeom::Cylinder(cylinder) = *edit.store().get(surface_id).unwrap() else {
        panic!("terminal face must be cylindrical")
    };
    let origin = cylinder.frame().origin();
    let shifted_frame =
        cylinder
            .frame()
            .with_origin(Point3::new(origin.x.next_up(), origin.y, origin.z));
    let shifted = Cylinder::new(shifted_frame, cylinder.radius()).unwrap();
    edit.store_mut()
        .replace_surface(surface_id, SurfaceGeom::Cylinder(shifted))
        .unwrap();
    assert_not_full_certified(edit.store(), &output);

    let mut shifted_plane = transaction.store().clone();
    let mut edit = shifted_plane.transaction().unwrap();
    let face = face_for_key(&output, LOW_SHOULDER_FACE);
    let surface_id = edit.store().get(face).unwrap().surface;
    let SurfaceGeom::Plane(plane) = *edit.store().get(surface_id).unwrap() else {
        panic!("shoulder face must be planar")
    };
    let origin = plane.frame().origin();
    let shifted = Plane::new(plane.frame().with_origin(Point3::new(
        origin.x,
        origin.y,
        origin.z.next_up(),
    )));
    edit.store_mut()
        .replace_surface(surface_id, SurfaceGeom::Plane(shifted))
        .unwrap();
    assert_not_full_certified(edit.store(), &output);
}

#[test]
fn two_shoulder_axial_fold_fails_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &two_shoulder_input_with_high_far(Frame::world(), [false; 3], 0, 0.0),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        certify_parallel_cylinder_contact_shell(transaction.store(), output.shell(), None).unwrap(),
        None,
    );
    assert_not_full_certified(transaction.store(), &output);
}

#[test]
fn two_shoulder_coincident_but_distinct_contact_vertices_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &two_shoulder_input(Frame::world(), [false; 3], 0),
            TOLERANCE,
        )
        .unwrap();
    let high = output
        .vertices()
        .iter()
        .find_map(|(key, vertex)| (*key == HIGH_TANGENT).then_some(*vertex))
        .unwrap();
    let position = transaction.store().vertex_position(high).unwrap();

    let mut tampered = transaction.store().clone();
    let mut edit = tampered.transaction().unwrap();
    let point = edit.store_mut().insert_point(position).unwrap();
    let duplicate = edit.store_mut().add(Vertex {
        point,
        tolerance: None,
    });
    edit.store_mut()
        .get_mut(edge_for_key(&output, HIGH_INNER_CONTACT))
        .unwrap()
        .vertices = [Some(duplicate), Some(duplicate)];
    assert_not_full_certified(edit.store(), &output);
}

#[test]
fn two_shoulder_face_sense_tampers_preserve_embedding_and_invalidate_orientation() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &two_shoulder_input(Frame::world(), [false; 3], 0),
            TOLERANCE,
        )
        .unwrap();

    for key in [
        LOW_INNER_FACE,
        OUTER_FACE,
        LOW_CAP_FACE,
        LOW_SHOULDER_FACE,
        HIGH_SHOULDER_FACE,
    ] {
        let mut tampered = transaction.store().clone();
        let face = face_for_key(&output, key);
        tampered.get_mut(face).unwrap().sense = tampered.get(face).unwrap().sense.flipped();
        assert_eq!(
            certify_parallel_cylinder_contact_shell(&tampered, output.shell(), None).unwrap(),
            invalid(),
            "face key {}",
            key.value(),
        );
        assert_eq!(
            crate::shell_proof::certify_shell(
                &tampered,
                output.shell(),
                crate::entity::BodyKind::Solid,
                crate::entity::RegionKind::Solid,
            )
            .unwrap(),
            invalid().unwrap(),
        );
        let report = check_body_report(&tampered, output.body(), CheckLevel::Full).unwrap();
        assert_eq!(report.outcome(), CheckOutcome::Invalid, "{report:#?}");
    }
}

fn session_with_work(allowed: u64) -> SessionPolicy {
    let budget = BudgetPlan::new([LimitSpec::new(
        PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        allowed,
    )])
    .unwrap();
    SessionPolicy::new(
        SessionPrecision::parasolid(),
        NumericalPolicy::v1(),
        ExecutionPolicy::Serial,
        budget,
        PolicyVersion::V1,
    )
}

fn store_counts(store: &Store) -> [usize; 8] {
    [
        store.count::<Body>(),
        store.count::<Region>(),
        store.count::<Shell>(),
        store.count::<Face>(),
        store.count::<Loop>(),
        store.count::<Fin>(),
        store.count::<Edge>(),
        store.count::<Vertex>(),
    ]
}

fn consumed(scope: &OperationScope<'_, '_>) -> u64 {
    scope
        .ledger()
        .snapshots()
        .into_iter()
        .find(|snapshot| snapshot.stage == PARALLEL_CYLINDER_CONTACT_SHELL_WORK)
        .unwrap()
        .consumed
}

#[test]
fn two_shoulder_work_accepts_1860_rejects_1859_and_inapplicable_is_free() {
    // N = 1 shell + 7 faces + 10 loops + 12 uses; N^2 + 32N = 1,860.
    const SIZE: u64 = 1 + 7 + 10 + 12;
    const REQUIRED: u64 = SIZE * SIZE + 32 * SIZE;
    assert_eq!(SIZE, 30);
    assert_eq!(REQUIRED, 1_860);

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &two_shoulder_input(Frame::world(), [true, false, true], 2),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        proof_work(transaction.store(), output.shell()).unwrap(),
        Some(REQUIRED),
    );
    let before = store_counts(transaction.store());

    let policy = session_with_work(REQUIRED);
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();
    let mut scope = OperationScope::new(&context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(
            transaction.store(),
            output.shell(),
            Some(&mut scope),
        )
        .unwrap(),
        positive(),
    );
    assert_eq!(consumed(&scope), REQUIRED);
    assert_eq!(store_counts(transaction.store()), before);

    let policy = session_with_work(REQUIRED - 1);
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();
    let mut scope = OperationScope::new(&context);
    let error = certify_parallel_cylinder_contact_shell(
        transaction.store(),
        output.shell(),
        Some(&mut scope),
    )
    .unwrap_err();
    assert_eq!(
        error.limit(),
        Some(kcore::operation::LimitSnapshot {
            stage: PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
            resource: ResourceKind::Work,
            consumed: REQUIRED,
            allowed: REQUIRED - 1,
        }),
    );
    assert_eq!(consumed(&scope), 0);
    assert_eq!(store_counts(transaction.store()), before);

    let mut inapplicable = transaction.store().clone();
    inapplicable
        .get_mut(output.shell())
        .unwrap()
        .edges
        .push(edge_for_key(&output, LOW_FAR));
    let policy = session_with_work(0);
    let context = OperationContext::new(&policy, Tolerances::default()).unwrap();
    let mut scope = OperationScope::new(&context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(&inapplicable, output.shell(), Some(&mut scope),)
            .unwrap(),
        None,
    );
    assert_eq!(consumed(&scope), 0);
    assert!(scope.ledger().limit_events().is_empty());
}
