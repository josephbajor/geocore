use super::*;
use crate::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellEdge, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput,
    AnalyticShellLoop, AnalyticShellOutput, AnalyticShellPcurve, AnalyticShellSurface,
    AnalyticShellVertex, AnalyticVertexKey,
};
use crate::check::{CheckLevel, CheckOutcome, check_body_report};
use crate::entity::{Body, Edge, Face, FaceDomain, Region, Sense, Shell, Vertex};
use crate::transaction::FullCommitRequirement;
use kcore::operation::{
    ExecutionPolicy, NumericalPolicy, OperationContext, PolicyVersion, SessionPolicy,
    SessionPrecision,
};
use kcore::tolerance::{ANGULAR_RESOLUTION, Tolerances};
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::Plane;
use kgeom::vec::{Point2, Vec2};
use kgraph::AffineParamMap1d;

const TOLERANCE: f64 = 1.0e-12;
const PERIOD: f64 = core::f64::consts::TAU;
const V0: AnalyticVertexKey = AnalyticVertexKey::new(0);
const V1: AnalyticVertexKey = AnalyticVertexKey::new(1);
const I0: AnalyticEdgeKey = AnalyticEdgeKey::new(0);
const O0: AnalyticEdgeKey = AnalyticEdgeKey::new(1);
const I1: AnalyticEdgeKey = AnalyticEdgeKey::new(2);
const O1: AnalyticEdgeKey = AnalyticEdgeKey::new(3);
const R0: AnalyticEdgeKey = AnalyticEdgeKey::new(4);
const R1: AnalyticEdgeKey = AnalyticEdgeKey::new(5);

#[derive(Debug, Clone, Copy)]
struct ArcPiece {
    key: AnalyticEdgeKey,
    circle: Circle,
    range: ParamRange,
    vertices: [AnalyticVertexKey; 2],
    inside: bool,
}

#[derive(Debug, Clone, Copy)]
struct BandFixture {
    cylinder: Cylinder,
    far_circle: Circle,
    far_parameter: f64,
    contact_sense: Sense,
    pieces: [ArcPiece; 2],
}

fn map(scale: f64) -> AffineParamMap1d {
    AffineParamMap1d::new(scale, 0.0).unwrap()
}

fn oblique_frame() -> Frame {
    Frame::new(
        Point3::new(2.5, -1.75, 0.625),
        Vec3::new(0.48, 0.64, 0.6),
        Vec3::new(0.8, -0.6, 0.0),
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

fn parameter(circle: Circle, point: Point3) -> f64 {
    let local = circle.frame().to_local(point);
    kcore::math::atan2(local.y, local.x)
}

fn vertex_key(point: Point3, vertices: [Point3; 2]) -> AnalyticVertexKey {
    let distances = vertices.map(|vertex| (point - vertex).norm_sq());
    if distances[0] <= distances[1] { V0 } else { V1 }
}

fn circle_pieces(
    circle: Circle,
    other_center: Point3,
    other_radius: f64,
    vertices: [Point3; 2],
    inside_key: AnalyticEdgeKey,
    outside_key: AnalyticEdgeKey,
) -> [ArcPiece; 2] {
    let mut roots = vertices.map(|vertex| parameter(circle, vertex));
    roots.sort_by(f64::total_cmp);
    let ranges = [
        ParamRange::new(roots[0], roots[1]),
        ParamRange::new(roots[1], roots[0] + PERIOD),
    ];
    let pieces = ranges.map(|range| {
        let midpoint = range.lo / 2.0 + range.hi / 2.0;
        let inside = (circle.eval(midpoint) - other_center).norm_sq() < other_radius * other_radius;
        ArcPiece {
            key: if inside { inside_key } else { outside_key },
            circle,
            range,
            vertices: [
                vertex_key(circle.eval(range.lo), vertices),
                vertex_key(circle.eval(range.hi), vertices),
            ],
            inside,
        }
    });
    assert_ne!(pieces[0].inside, pieces[1].inside);
    pieces
}

fn cylinder_arc(piece: ArcPiece, sense: Sense, height: f64) -> AnalyticShellFin {
    AnalyticShellFin::new(
        piece.key,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            map(1.0),
        ),
    )
}

fn projected_arc(piece: ArcPiece, sense: Sense, plane: Plane) -> AnalyticShellFin {
    let center = plane.frame().to_local(piece.circle.frame().origin());
    let local_x = Vec2::new(
        piece.circle.frame().x().dot(plane.frame().x()),
        piece.circle.frame().x().dot(plane.frame().y()),
    );
    let local_y = Vec2::new(
        piece.circle.frame().y().dot(plane.frame().x()),
        piece.circle.frame().y().dot(plane.frame().y()),
    );
    let scale = if local_x.perp().dot(local_y) > 0.0 {
        1.0
    } else {
        -1.0
    };
    AnalyticShellFin::new(
        piece.key,
        sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Circle(
                Circle2d::new(
                    Point2::new(center.x, center.y),
                    piece.circle.radius(),
                    local_x,
                )
                .unwrap(),
            ),
            map(scale),
        ),
    )
}

fn directed_tail(piece: ArcPiece, sense: Sense) -> AnalyticVertexKey {
    if sense.is_forward() {
        piece.vertices[0]
    } else {
        piece.vertices[1]
    }
}

fn directed_head(piece: ArcPiece, sense: Sense) -> AnalyticVertexKey {
    if sense.is_forward() {
        piece.vertices[1]
    } else {
        piece.vertices[0]
    }
}

fn arc_loop(
    first: (ArcPiece, Sense),
    second: (ArcPiece, Sense),
    plane: Option<Plane>,
    rotated: bool,
) -> AnalyticShellLoop {
    let mut uses = [first, second];
    if directed_head(uses[0].0, uses[0].1) != directed_tail(uses[1].0, uses[1].1) {
        uses.swap(0, 1);
    }
    assert_eq!(
        directed_head(uses[0].0, uses[0].1),
        directed_tail(uses[1].0, uses[1].1)
    );
    assert_eq!(
        directed_head(uses[1].0, uses[1].1),
        directed_tail(uses[0].0, uses[0].1)
    );
    if rotated {
        uses.rotate_left(1);
    }
    AnalyticShellLoop::new(
        uses.map(|(piece, sense)| match plane {
            Some(plane) => projected_arc(piece, sense, plane),
            None => cylinder_arc(piece, sense, 0.0),
        })
        .to_vec(),
    )
}

fn ring_uses(
    key: AnalyticEdgeKey,
    circle: Circle,
    parameter: f64,
    side_sense: Sense,
    cap_plane: Plane,
) -> (AnalyticShellFin, AnalyticShellFin) {
    let side = AnalyticShellFin::new(
        key,
        side_sense,
        AnalyticPcurveUse::new(
            AnalyticShellPcurve::Line(
                Line2d::new(Point2::new(0.0, parameter), Vec2::new(1.0, 0.0)).unwrap(),
            ),
            map(1.0),
        )
        .with_closure_winding([1, 0]),
    );
    let synthetic = ArcPiece {
        key,
        circle,
        range: circle.param_range(),
        vertices: [V0, V1],
        inside: false,
    };
    let cap = projected_arc(synthetic, side_sense.flipped(), cap_plane);
    (
        side,
        AnalyticShellFin::new(
            cap.edge(),
            cap.sense(),
            cap.pcurve().with_closure_winding([0, 0]),
        ),
    )
}

fn side_contact_loop(band: BandFixture, rotated: bool) -> AnalyticShellLoop {
    let ordered = if band.contact_sense.is_forward() {
        [band.pieces[0], band.pieces[1]]
    } else {
        [band.pieces[1], band.pieces[0]]
    };
    arc_loop(
        (ordered[0], band.contact_sense),
        (ordered[1], band.contact_sense),
        None,
        rotated,
    )
}

fn piece(band: BandFixture, inside: bool) -> ArcPiece {
    band.pieces
        .into_iter()
        .find(|piece| piece.inside == inside)
        .unwrap()
}

fn strict_secant_input(
    frame: Frame,
    first_reversed: bool,
    second_reversed: bool,
    permuted: bool,
) -> AnalyticShellInput {
    strict_secant_input_with_geometry(
        frame,
        first_reversed,
        second_reversed,
        permuted,
        [1.0, 1.0],
        1.0,
    )
}

fn strict_secant_input_with_geometry(
    frame: Frame,
    first_reversed: bool,
    second_reversed: bool,
    permuted: bool,
    radii: [f64; 2],
    center_distance: f64,
) -> AnalyticShellInput {
    let intersection_x = (radii[0] * radii[0] - radii[1] * radii[1]
        + center_distance * center_distance)
        / (2.0 * center_distance);
    let intersection_y = (radii[0] * radii[0] - intersection_x * intersection_x).sqrt();
    assert!(intersection_y.is_finite() && intersection_y > 0.0);
    let first_origin = frame.origin();
    let second_origin = frame.point_at(center_distance, 0.0, 0.0);
    let first_frame = axis_frame(frame, first_origin, first_reversed);
    let second_frame = axis_frame(frame, second_origin, second_reversed);
    let contact_circles = [
        Circle::new(first_frame, radii[0]).unwrap(),
        Circle::new(second_frame, radii[1]).unwrap(),
    ];
    let ideal_vertices = [
        frame.point_at(intersection_x, -intersection_y, 0.0),
        frame.point_at(intersection_x, intersection_y, 0.0),
    ];
    let vertices = ideal_vertices.map(|point| {
        let parameter = parameter(contact_circles[0], point);
        contact_circles[0].eval(parameter)
    });
    let far_origins = [
        frame.point_at(0.0, 0.0, -2.0),
        frame.point_at(center_distance, 0.0, 2.0),
    ];
    let cylinders = [
        Cylinder::new(first_frame, radii[0]).unwrap(),
        Cylinder::new(second_frame, radii[1]).unwrap(),
    ];
    let far_circles = [
        Circle::new(first_frame.with_origin(far_origins[0]), radii[0]).unwrap(),
        Circle::new(second_frame.with_origin(far_origins[1]), radii[1]).unwrap(),
    ];
    let far_parameters = [
        (far_origins[0] - first_origin).dot(first_frame.z()),
        (far_origins[1] - second_origin).dot(second_frame.z()),
    ];
    let pieces = [
        circle_pieces(
            contact_circles[0],
            second_origin,
            radii[1],
            vertices,
            I0,
            O0,
        ),
        circle_pieces(contact_circles[1], first_origin, radii[0], vertices, I1, O1),
    ];
    let bands: [BandFixture; 2] = core::array::from_fn(|index| BandFixture {
        cylinder: cylinders[index],
        far_circle: far_circles[index],
        far_parameter: far_parameters[index],
        contact_sense: if 0.0 < far_parameters[index] {
            Sense::Forward
        } else {
            Sense::Reversed
        },
        pieces: pieces[index],
    });

    let negative_plane = |origin| Plane::new(Frame::new(origin, -frame.z(), frame.x()).unwrap());
    let positive_plane = |origin| Plane::new(frame.with_origin(origin));
    let far_planes = [
        negative_plane(far_origins[0]),
        positive_plane(far_origins[1]),
    ];
    let contact_planes = [
        positive_plane(frame.origin()),
        negative_plane(frame.origin()),
    ];
    let mut edges = bands
        .iter()
        .flat_map(|band| band.pieces)
        .map(|piece| {
            AnalyticShellEdge::new(
                piece.key,
                piece.vertices,
                AnalyticShellCurve::Circle(piece.circle),
                piece.range,
            )
        })
        .collect::<Vec<_>>();
    let mut closed_edges = [0, 1]
        .map(|index| {
            let lo = bands[index].pieces[0].range.lo;
            AnalyticShellClosedEdge::new(
                if index == 0 { R0 } else { R1 },
                AnalyticShellCurve::Circle(bands[index].far_circle),
                ParamRange::new(lo, lo + PERIOD),
            )
        })
        .to_vec();

    let mut faces = Vec::new();
    for index in 0..2 {
        let ring_key = if index == 0 { R0 } else { R1 };
        let far_sense = bands[index].contact_sense.flipped();
        let (side_ring, cap_ring) = ring_uses(
            ring_key,
            bands[index].far_circle,
            bands[index].far_parameter,
            far_sense,
            far_planes[index],
        );
        let far_loop = AnalyticShellLoop::new(vec![side_ring]);
        let contact_loop = side_contact_loop(bands[index], permuted);
        let loops = if permuted {
            vec![contact_loop, far_loop]
        } else {
            vec![far_loop, contact_loop]
        };
        let u_lo = bands[index].pieces[0].range.lo;
        faces.push(AnalyticShellFace::new(
            AnalyticFaceKey::new(index as u64),
            AnalyticShellSurface::Cylinder(bands[index].cylinder),
            Sense::Forward,
            FaceDomain::from_bounds(
                u_lo,
                u_lo + PERIOD,
                bands[index].far_parameter.min(0.0),
                bands[index].far_parameter.max(0.0),
            )
            .unwrap(),
            loops,
        ));
        faces.push(AnalyticShellFace::new(
            AnalyticFaceKey::new((2 + index) as u64),
            AnalyticShellSurface::Plane(far_planes[index]),
            Sense::Forward,
            FaceDomain::from_bounds(
                -radii[index] - 0.1,
                radii[index] + 0.1,
                -radii[index] - 0.1,
                radii[index] + 0.1,
            )
            .unwrap(),
            vec![AnalyticShellLoop::new(vec![cap_ring])],
        ));
    }

    let contact_extent = center_distance + radii[0].max(radii[1]) + 0.1;
    let outside0 = piece(bands[0], false);
    let inside0 = piece(bands[0], true);
    let outside1 = piece(bands[1], false);
    let inside1 = piece(bands[1], true);
    faces.push(AnalyticShellFace::new(
        AnalyticFaceKey::new(4),
        AnalyticShellSurface::Plane(contact_planes[0]),
        Sense::Forward,
        FaceDomain::from_bounds(
            -contact_extent,
            contact_extent,
            -contact_extent,
            contact_extent,
        )
        .unwrap(),
        vec![arc_loop(
            (outside0, bands[0].contact_sense.flipped()),
            (inside1, bands[1].contact_sense.flipped()),
            Some(contact_planes[0]),
            permuted,
        )],
    ));
    faces.push(AnalyticShellFace::new(
        AnalyticFaceKey::new(5),
        AnalyticShellSurface::Plane(contact_planes[1]),
        Sense::Forward,
        FaceDomain::from_bounds(
            -contact_extent,
            contact_extent,
            -contact_extent,
            contact_extent,
        )
        .unwrap(),
        vec![arc_loop(
            (outside1, bands[1].contact_sense.flipped()),
            (inside0, bands[0].contact_sense.flipped()),
            Some(contact_planes[1]),
            permuted,
        )],
    ));

    let mut analytic_vertices = vec![
        AnalyticShellVertex::new(V0, vertices[0]),
        AnalyticShellVertex::new(V1, vertices[1]),
    ];
    if permuted {
        analytic_vertices.reverse();
        edges.reverse();
        closed_edges.reverse();
        faces.reverse();
    }
    AnalyticShellInput::new(analytic_vertices, edges, faces).with_closed_edges(closed_edges)
}

fn positive() -> Option<ShellCertification> {
    Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: ShellOrientation::Positive,
    })
}

fn face_for_key(output: &AnalyticShellOutput, key: u64) -> FaceId {
    output
        .faces()
        .iter()
        .find_map(|(candidate, face)| (candidate.value() == key).then_some(*face))
        .unwrap()
}

fn edge_for_key(output: &AnalyticShellOutput, key: u64) -> EdgeId {
    output
        .edges()
        .iter()
        .find_map(|(candidate, edge)| (candidate.value() == key).then_some(*edge))
        .unwrap()
}

#[test]
fn strict_secant_contact_is_full_valid_across_frames_axis_directions_and_permutations() {
    for (radii, center_distance) in [([1.0, 1.0], 1.0), ([2.0, 1.0], 2.0)] {
        for frame in [Frame::world(), oblique_frame()] {
            for first_reversed in [false, true] {
                for second_reversed in [false, true] {
                    for permuted in [false, true] {
                        let mut store = Store::new();
                        let mut transaction = store.transaction().unwrap();
                        let output = transaction
                            .assemble_analytic_shell(
                                &strict_secant_input_with_geometry(
                                    frame,
                                    first_reversed,
                                    second_reversed,
                                    permuted,
                                    radii,
                                    center_distance,
                                ),
                                TOLERANCE,
                            )
                            .unwrap();
                        assert_eq!(output.faces().len(), 6);
                        assert_eq!(output.edges().len(), 6);
                        assert_eq!(output.vertices().len(), 2);
                        let direct = certify_parallel_cylinder_contact_shell(
                            transaction.store(),
                            output.shell(),
                            None,
                        )
                        .unwrap();
                        assert_eq!(
                            direct,
                            positive(),
                            "radii={radii:?}, center_distance={center_distance}, frame={frame:?}, first_reversed={first_reversed}, second_reversed={second_reversed}, permuted={permuted}",
                        );
                        let first =
                            check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                                .unwrap();
                        let replayed =
                            check_body_report(transaction.store(), output.body(), CheckLevel::Full)
                                .unwrap();
                        assert_eq!(first, replayed);
                        assert_eq!(
                            first.outcome(),
                            CheckOutcome::Valid,
                            "radii={radii:?}, center_distance={center_distance}, frame={frame:?}, first_reversed={first_reversed}, second_reversed={second_reversed}, permuted={permuted}: {first:#?}",
                        );
                        transaction
                            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
                            .unwrap();
                    }
                }
            }
        }
    }
}

#[test]
fn strict_secant_contact_degree_four_links_and_tampers_fail_closed() {
    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &strict_secant_input(Frame::world(), false, false, false),
            TOLERANCE,
        )
        .unwrap();
    let vertices = output
        .vertices()
        .iter()
        .map(|(_, vertex)| *vertex)
        .collect::<Vec<_>>();
    for vertex in vertices {
        let incident = output
            .edges()
            .iter()
            .filter(|(key, edge)| {
                key.value() < 4
                    && transaction
                        .store()
                        .get(*edge)
                        .unwrap()
                        .vertices
                        .contains(&Some(vertex))
            })
            .count();
        assert_eq!(incident, 4);
    }

    let second_side = face_for_key(&output, 1);
    let mut wrong_sense = transaction.store().clone();
    wrong_sense.get_mut(second_side).unwrap().sense = Sense::Reversed;
    assert_eq!(
        certify_parallel_cylinder_contact_shell(&wrong_sense, output.shell(), None).unwrap(),
        Some(ShellCertification {
            embedding: ShellEmbedding::Certified,
            orientation: ShellOrientation::Invalid,
        })
    );

    let mut wrong_link = transaction.store().clone();
    let edge = edge_for_key(&output, I0.value());
    let first_vertex = wrong_link.get(edge).unwrap().vertices[0];
    wrong_link.get_mut(edge).unwrap().vertices[1] = first_vertex;
    assert_eq!(
        certify_parallel_cylinder_contact_shell(&wrong_link, output.shell(), None).unwrap(),
        None
    );
    assert_ne!(
        check_body_report(&wrong_link, output.body(), CheckLevel::Full)
            .unwrap()
            .outcome(),
        CheckOutcome::Valid
    );

    let surface = transaction.store().get(second_side).unwrap().surface;
    let SurfaceGeom::Cylinder(cylinder) = *transaction.store().get(surface).unwrap() else {
        panic!("second side must be cylindrical")
    };
    let tilted = Frame::new(
        cylinder.frame().origin(),
        cylinder.frame().z() + cylinder.frame().y() * (0.25 * ANGULAR_RESOLUTION),
        cylinder.frame().x(),
    )
    .unwrap();
    transaction
        .store_mut()
        .replace_surface(
            surface,
            SurfaceGeom::Cylinder(Cylinder::new(tilted, cylinder.radius()).unwrap()),
        )
        .unwrap();
    assert_ne!(
        certify_parallel_cylinder_contact_shell(transaction.store(), output.shell(), None).unwrap(),
        positive()
    );
}

#[test]
fn cross_product_radial_distance_encloses_ill_conditioned_harmonic_oracle() {
    let axis = Vec3::new(1.0, 1.0, 1.0).normalized().unwrap();
    let frame = Frame::new(
        Point3::new(0.0, 0.0, 0.0),
        axis,
        axis + Vec3::new(1.0e-6, -1.0e-6, 0.0),
    )
    .unwrap();
    let radial = Vec3::new(1.0, -1.0, 0.0).normalized().unwrap();
    let plane_center = frame.origin() + frame.z() * 400.0;
    let cylinder = Cylinder::new(frame, 1.0).unwrap();
    let portal = Circle::new(frame.with_origin(plane_center), 1.0).unwrap();
    let profile = Circle::new(frame.with_origin(plane_center + radial), 1.0).unwrap();
    let range = ParamRange::new(-core::f64::consts::PI / 3.0, core::f64::consts::PI / 3.0);
    let midpoint = range.lo / 2.0 + range.hi / 2.0;
    let point = profile.eval(midpoint);
    let displacement = point - frame.origin();
    let oracle = displacement.cross(frame.z()).norm_sq() / frame.z().norm_sq();
    let legacy = displacement.dot(frame.x()).powi(2) + displacement.dot(frame.y()).powi(2);
    let enclosed = interval_axis_distance_squared(
        interval_circle_point(profile, midpoint).unwrap(),
        interval_point(frame.origin().to_array()),
        interval_point(frame.z().to_array()),
    )
    .unwrap();

    assert!(frame.x().dot(frame.z()).abs() > 1.0e-10);
    assert!((oracle - legacy).abs() > 1.0e-8);
    assert!(enclosed.lo() <= oracle && oracle <= enclosed.hi());
    assert_eq!(
        strict_secant_span_side(cylinder, profile, range, portal, true),
        Some(SecantRadialSide::Outside)
    );

    let tangent = Circle::new(frame.with_origin(plane_center + radial * 2.0), 1.0).unwrap();
    assert_eq!(
        strict_secant_span_side(cylinder, tangent, range, portal, true),
        None
    );
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
        store.count::<crate::entity::Loop>(),
        store.count::<crate::entity::Fin>(),
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
fn strict_secant_contact_work_accepts_1593_rejects_1592_and_inapplicable_is_free() {
    const SIZE: u64 = 1 + 6 + 8 + 12;
    const REQUIRED: u64 = SIZE * SIZE + 32 * SIZE;
    assert_eq!(SIZE, 27);
    assert_eq!(REQUIRED, 1_593);

    let mut store = Store::new();
    let mut transaction = store.transaction().unwrap();
    let output = transaction
        .assemble_analytic_shell(
            &strict_secant_input(Frame::world(), true, false, true),
            TOLERANCE,
        )
        .unwrap();
    assert_eq!(
        proof_work(transaction.store(), output.shell()).unwrap(),
        Some(REQUIRED)
    );
    let before = store_counts(transaction.store());

    let accepted_policy = session_with_work(REQUIRED);
    let accepted_context = OperationContext::new(&accepted_policy, Tolerances::default()).unwrap();
    let mut accepted_scope = OperationScope::new(&accepted_context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(
            transaction.store(),
            output.shell(),
            Some(&mut accepted_scope),
        )
        .unwrap(),
        positive()
    );
    assert_eq!(consumed(&accepted_scope), REQUIRED);
    assert_eq!(store_counts(transaction.store()), before);

    let denied_policy = session_with_work(REQUIRED - 1);
    let denied_context = OperationContext::new(&denied_policy, Tolerances::default()).unwrap();
    let mut denied_scope = OperationScope::new(&denied_context);
    let error = certify_parallel_cylinder_contact_shell(
        transaction.store(),
        output.shell(),
        Some(&mut denied_scope),
    )
    .unwrap_err();
    assert_eq!(
        error.limit(),
        Some(kcore::operation::LimitSnapshot {
            stage: PARALLEL_CYLINDER_CONTACT_SHELL_WORK,
            resource: ResourceKind::Work,
            consumed: REQUIRED,
            allowed: REQUIRED - 1,
        })
    );
    assert_eq!(consumed(&denied_scope), 0);
    assert_eq!(store_counts(transaction.store()), before);

    let mut inapplicable = transaction.store().clone();
    inapplicable
        .get_mut(output.shell())
        .unwrap()
        .edges
        .push(edge_for_key(&output, I0.value()));
    let zero_policy = session_with_work(0);
    let zero_context = OperationContext::new(&zero_policy, Tolerances::default()).unwrap();
    let mut zero_scope = OperationScope::new(&zero_context);
    assert_eq!(
        certify_parallel_cylinder_contact_shell(
            &inapplicable,
            output.shell(),
            Some(&mut zero_scope),
        )
        .unwrap(),
        None
    );
    assert_eq!(consumed(&zero_scope), 0);
    assert!(zero_scope.ledger().limit_events().is_empty());
}
