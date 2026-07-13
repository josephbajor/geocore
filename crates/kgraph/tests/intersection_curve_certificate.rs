//! Whole-interval certification and persistent intersection-descriptor tests.

use kcore::tolerance::Tolerances;
use kgeom::curve::{Circle, Curve, Line};
use kgeom::curve2d::NurbsCurve2d;
use kgeom::curve2d::{Circle2d, Curve2d, Line2d};
use kgeom::frame::Frame;
use kgeom::nurbs::{NurbsCurve, NurbsSurface};
use kgeom::param::ParamRange;
use kgeom::surface::{Plane, Sphere, Surface};
use kgeom::vec::{Vec2, Vec3};
use kgraph::{
    AffineParamMap1d, Curve2dDescriptor, CurveClass, EvalContext, EvalError, EvalLimits,
    GeometryGraph, GeometryGraphError, GeometryRef, IntersectionCertificateError,
    OffsetSurfaceDescriptor, PairedTrace, PlaneCircleTrace, PlaneSphereCircleTrace,
    SphereLatitudeTrace, TransmittedIntersectionChartMetadata, TransmittedOffsetNurbsTrace,
    TransmittedPlaneNurbsTrace, VerifiedIntersectionCarrier, VerifiedIntersectionCertificate,
    certify_paired_plane_line_residuals, certify_paired_plane_sphere_circle_residuals,
    certify_paired_plane_sphere_oblique_circle_residuals,
    certify_transmitted_nurbs_nurbs_intersection_residuals,
    certify_transmitted_offset_nurbs_intersection_residuals,
    certify_transmitted_plane_intersection_residuals,
    certify_transmitted_plane_nurbs_intersection_residuals,
};

fn nonplanar_trace_surface(rational: bool) -> NurbsSurface {
    let points = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.5, 0.4),
        Vec3::new(0.0, 1.0, 0.1),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.5, -0.3),
        Vec3::new(1.0, 1.0, 0.2),
    ];
    let weights = rational.then(|| vec![1.0, 0.75, 1.4, 1.0, 1.25, 0.8]);
    NurbsSurface::new(
        1,
        2,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap()
}

fn second_nonplanar_trace_surface(rational: bool) -> NurbsSurface {
    let points = vec![
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.5, -0.2),
        Vec3::new(0.0, 1.0, 0.6),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(1.0, 0.5, 0.5),
        Vec3::new(1.0, 1.0, -0.4),
    ];
    let weights = rational.then(|| vec![1.0, 0.9, 1.3, 1.0, 1.1, 0.7]);
    NurbsSurface::new(
        1,
        2,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        points,
        weights,
    )
    .unwrap()
}

fn carrier() -> Line {
    Line::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap()
}

fn planes() -> [Plane; 2] {
    [
        Plane::new(Frame::world()),
        Plane::new(
            Frame::new(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            )
            .unwrap(),
        ),
    ]
}

fn identity_pcurves() -> [Line2d; 2] {
    [
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ]
}

fn identity_maps() -> [AffineParamMap1d; 2] {
    [
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    ]
}

fn aligned_circle_fixture(
    sphere_radius: f64,
    height: f64,
) -> (
    Circle,
    Plane,
    Sphere,
    Circle2d,
    Line2d,
    [PlaneSphereCircleTrace; 2],
) {
    let frame = Frame::world();
    let plane = Plane::new(frame.with_origin(Vec3::new(0.0, 0.0, height)));
    let sphere = Sphere::new(frame, sphere_radius).unwrap();
    let radius = (sphere_radius * sphere_radius - height * height).sqrt();
    let carrier = Circle::new(frame.with_origin(Vec3::new(0.0, 0.0, height)), radius).unwrap();
    let plane_pcurve = Circle2d::new(Vec2::new(0.0, 0.0), radius, Vec2::new(1.0, 0.0)).unwrap();
    let latitude = kcore::math::atan2(height, radius);
    let sphere_pcurve = Line2d::new(Vec2::new(0.0, latitude), Vec2::new(1.0, 0.0)).unwrap();
    let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
    let traces = [
        PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, identity)),
        PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(sphere, sphere_pcurve, identity)),
    ];
    (carrier, plane, sphere, plane_pcurve, sphere_pcurve, traces)
}

fn oblique_circle_fixture() -> (Circle, Plane, Sphere, Circle2d) {
    let sphere = Sphere::new(Frame::world(), 2.5).unwrap();
    let normal = Vec3::new(0.0, 0.6, 0.8);
    let center = normal * 0.5;
    let plane = Plane::new(Frame::new(center, normal, Vec3::new(1.0, 0.0, 0.0)).unwrap());
    let radius = (sphere.radius() * sphere.radius() - 0.25).sqrt();
    let carrier = Circle::new(
        Frame::new(center, normal, Vec3::new(1.0, 0.0, 0.0)).unwrap(),
        radius,
    )
    .unwrap();
    let plane_pcurve = Circle2d::new(Vec2::new(0.0, 0.0), radius, Vec2::new(1.0, 0.0)).unwrap();
    (carrier, plane, sphere, plane_pcurve)
}

fn sphere_longitude(carrier: Circle, sphere: Sphere, parameter: f64) -> f64 {
    let local = sphere.frame().to_local(carrier.eval(parameter));
    kcore::math::atan2(local.y, local.x)
}

#[test]
fn exact_paired_plane_traces_are_certified_over_the_whole_range() {
    let range = ParamRange::new(-3.0, 7.0);
    let certificate = certify_paired_plane_line_residuals(
        carrier(),
        range,
        planes(),
        identity_pcurves(),
        identity_maps(),
        1.0e-12,
    )
    .unwrap();

    assert_eq!(certificate.carrier(), carrier());
    assert_eq!(certificate.carrier_range(), range);
    assert_eq!(certificate.surfaces(), planes());
    assert_eq!(certificate.pcurves(), identity_pcurves());
    assert_eq!(certificate.parameter_maps(), identity_maps());
    assert_eq!(certificate.tolerance(), 1.0e-12);
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
}

#[test]
fn reversed_and_nonidentity_parameter_maps_certify_and_roundtrip() {
    let pcurves = [
        Line2d::new(Vec2::new(5.0, 0.0), Vec2::new(-1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(-2.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ];
    // First lift: u = 5 - (5 - t) = t. Second lift:
    // u = -2 + (2 + t) = t.
    let maps = [
        AffineParamMap1d::new(-1.0, 5.0).unwrap(),
        AffineParamMap1d::new(1.0, 2.0).unwrap(),
    ];
    let range = ParamRange::new(-4.0, 3.0);

    let certificate =
        certify_paired_plane_line_residuals(carrier(), range, planes(), pcurves, maps, 1.0e-12)
            .unwrap();

    assert_eq!(maps[0].map_range(range), ParamRange::new(2.0, 9.0));
    for map in maps {
        for parameter in [range.lo, 0.25, range.hi] {
            assert_eq!(map.inverse(map.map(parameter)), parameter);
        }
    }
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= 1.0e-12)
    );
}

#[test]
fn perturbed_pcurve_is_rejected_with_the_failing_trace() {
    let mut pcurves = identity_pcurves();
    pcurves[1] = Line2d::new(Vec2::new(0.0, 0.01), Vec2::new(1.0, 0.0)).unwrap();

    let error = certify_paired_plane_line_residuals(
        carrier(),
        ParamRange::new(-1.0, 1.0),
        planes(),
        pcurves,
        identity_maps(),
        1.0e-6,
    )
    .unwrap_err();

    assert!(matches!(
        error,
        IntersectionCertificateError::ResidualExceedsTolerance {
            trace: PairedTrace::Second,
            residual_bound,
            tolerance: 1.0e-6,
        } if residual_bound >= 0.01
    ));
}

#[test]
fn nonfinite_reversed_ranges_maps_and_tolerances_are_rejected() {
    assert!(matches!(
        AffineParamMap1d::new(0.0, 1.0),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));
    assert!(matches!(
        AffineParamMap1d::new(f64::NAN, 1.0),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));
    assert!(matches!(
        AffineParamMap1d::new(1.0, f64::INFINITY),
        Err(IntersectionCertificateError::InvalidParameterMap { .. })
    ));

    for range in [
        ParamRange::unbounded(),
        ParamRange { lo: 2.0, hi: 1.0 },
        ParamRange {
            lo: f64::NAN,
            hi: 1.0,
        },
    ] {
        assert_eq!(
            certify_paired_plane_line_residuals(
                carrier(),
                range,
                planes(),
                identity_pcurves(),
                identity_maps(),
                1.0e-6,
            ),
            Err(IntersectionCertificateError::InvalidCarrierRange)
        );
    }

    for tolerance in [-1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            certify_paired_plane_line_residuals(
                carrier(),
                ParamRange::new(-1.0, 1.0),
                planes(),
                identity_pcurves(),
                identity_maps(),
                tolerance,
            ),
            Err(IntersectionCertificateError::InvalidTolerance)
        );
    }

    let nonfinite_carrier =
        Line::new(Vec3::new(f64::NAN, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    assert_eq!(
        certify_paired_plane_line_residuals(
            nonfinite_carrier,
            ParamRange::new(-1.0, 1.0),
            planes(),
            identity_pcurves(),
            identity_maps(),
            1.0e-6,
        ),
        Err(IntersectionCertificateError::NonFiniteGeometry)
    );
}

#[test]
fn certificate_minting_is_deterministic() {
    let mint = || {
        certify_paired_plane_line_residuals(
            carrier(),
            ParamRange::new(-17.25, 23.5),
            planes(),
            identity_pcurves(),
            identity_maps(),
            1.0e-12,
        )
        .unwrap()
    };

    assert_eq!(mint(), mint());
    assert_eq!(format!("{:?}", mint()), format!("{:?}", mint()));
}

#[test]
fn persistent_descriptor_retains_ordered_identity_proof_and_bounded_evaluation() {
    let pcurves = [
        Line2d::new(Vec2::new(5.0, 0.0), Vec2::new(-1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(-2.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
    ];
    let maps = [
        AffineParamMap1d::new(-1.0, 5.0).unwrap(),
        AffineParamMap1d::new(1.0, 2.0).unwrap(),
    ];
    let range = ParamRange::new(-4.0, 3.0);
    let certificate =
        certify_paired_plane_line_residuals(carrier(), range, planes(), pcurves, maps, 1.0e-12)
            .unwrap();
    let mut graph = GeometryGraph::new();
    let surfaces = [
        graph.insert_surface(planes()[0]).unwrap(),
        graph.insert_surface(planes()[1]).unwrap(),
    ];
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0]).unwrap(),
        graph.insert_curve2d(pcurves[1]).unwrap(),
    ];
    let curve = graph
        .insert_verified_plane_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();

    let descriptor = graph.curve(curve).unwrap();
    assert_eq!(descriptor.class(), CurveClass::Intersection);
    assert_eq!(descriptor.class_key(), CurveClass::Intersection.key());
    let intersection = descriptor.as_intersection().copied().unwrap();
    assert_eq!(intersection.source_surfaces(), surfaces);
    assert_eq!(intersection.pcurves(), pcurve_handles);
    assert_eq!(
        intersection.certificate(),
        VerifiedIntersectionCertificate::PlaneLine(certificate)
    );
    assert_eq!(
        intersection.carrier(),
        VerifiedIntersectionCarrier::Line(carrier())
    );
    assert_eq!(intersection.carrier_range(), range);
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(surfaces[0]),
            GeometryRef::Surface(surfaces[1]),
            GeometryRef::Curve2d(pcurve_handles[0]),
            GeometryRef::Curve2d(pcurve_handles[1]),
        ]
    );
    assert_eq!(
        graph.dependency_closure(GeometryRef::Curve(curve)).unwrap(),
        vec![
            GeometryRef::Surface(surfaces[0]),
            GeometryRef::Surface(surfaces[1]),
            GeometryRef::Curve2d(pcurve_handles[0]),
            GeometryRef::Curve2d(pcurve_handles[1]),
            GeometryRef::Curve(curve),
        ]
    );

    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(eval.curve_param_range(curve), Ok(range));
    for parameter in [range.lo, range.lerp(0.37), range.hi] {
        assert_eq!(
            eval.eval_curve(curve, parameter, 1).unwrap().d[0],
            carrier().eval(parameter)
        );
    }
    assert_eq!(
        eval.eval_curve(curve, range.hi + 1.0, 0),
        Err(EvalError::ParameterOutsideDomain)
    );
    graph.validate().unwrap();
}

#[test]
fn persistent_descriptor_rejects_mismatches_atomically_and_protects_proof_sources() {
    let range = ParamRange::new(-1.0, 1.0);
    let pcurves = identity_pcurves();
    let certificate = certify_paired_plane_line_residuals(
        carrier(),
        range,
        planes(),
        pcurves,
        identity_maps(),
        1.0e-12,
    )
    .unwrap();
    let mut graph = GeometryGraph::new();
    let surfaces = [
        graph.insert_surface(planes()[0]).unwrap(),
        graph.insert_surface(planes()[1]).unwrap(),
    ];
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0]).unwrap(),
        graph.insert_curve2d(pcurves[1]).unwrap(),
    ];
    let wrong_pcurve = graph
        .insert_curve2d(Line2d::new(Vec2::new(0.0, 0.25), Vec2::new(1.0, 0.0)).unwrap())
        .unwrap();
    let counts = (
        graph.curve_count(),
        graph.surface_count(),
        graph.curve2d_count(),
    );

    assert!(matches!(
        graph.insert_verified_plane_intersection_curve(
            surfaces,
            [pcurve_handles[0], wrong_pcurve],
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { class, .. })
            if class == CurveClass::Intersection.key()
    ));
    assert_eq!(
        (
            graph.curve_count(),
            graph.surface_count(),
            graph.curve2d_count()
        ),
        counts
    );

    let curve = graph
        .insert_verified_plane_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let dependent = vec![GeometryRef::Curve(curve)];
    assert_eq!(
        graph.replace_surface(surfaces[0], planes()[0]),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(surfaces[0]),
            dependents: dependent.clone(),
        })
    );
    assert_eq!(
        graph.replace_curve2d(pcurve_handles[1], pcurves[1]),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Curve2d(pcurve_handles[1]),
            dependents: dependent,
        })
    );
    graph.validate().unwrap();
}

#[test]
fn transmitted_plane_chart_binds_exact_owned_geometry_and_protects_ordered_sources() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurves = [
        NurbsCurve2d::new(
            1,
            knots.clone(),
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
        NurbsCurve2d::new(
            1,
            knots,
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
    ];
    let metadata =
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap();
    let certificate = certify_transmitted_plane_intersection_residuals(
        carrier.clone(),
        planes(),
        pcurves.clone(),
        metadata,
        1.0e-12,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let surfaces = [
        graph.insert_surface(planes()[0]).unwrap(),
        graph.insert_surface(planes()[1]).unwrap(),
    ];
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0].clone()).unwrap(),
        graph.insert_curve2d(pcurves[1].clone()).unwrap(),
    ];
    let wrong_pcurve = graph
        .insert_curve2d(
            NurbsCurve2d::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![Vec2::new(0.0, 0.25), Vec2::new(1.0, 0.25)],
                None,
            )
            .unwrap(),
        )
        .unwrap();
    let wrong_surface = graph
        .insert_surface(Plane::new(
            Frame::world().with_origin(Vec3::new(0.0, 0.0, 0.25)),
        ))
        .unwrap();
    let stale_surface = graph.insert_surface(planes()[0]).unwrap();
    graph.remove_surface(stale_surface).unwrap();
    let curve_count = graph.curve_count();
    assert_eq!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            [stale_surface, surfaces[1]],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(stale_surface),
        })
    );
    assert_eq!(graph.curve_count(), curve_count);
    assert!(matches!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            surfaces,
            [pcurve_handles[0], wrong_pcurve],
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { class, .. })
            if class == CurveClass::Intersection.key()
    ));
    assert_eq!(graph.curve_count(), curve_count);
    assert!(matches!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            [surfaces[0], wrong_surface],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { class, .. })
            if class == CurveClass::Intersection.key()
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_transmitted_plane_intersection_curve(surfaces, pcurve_handles, certificate)
        .unwrap();
    let descriptor = graph
        .curve(curve)
        .unwrap()
        .as_transmitted_intersection()
        .unwrap();
    assert_eq!(descriptor.source_surfaces(), surfaces);
    assert_eq!(descriptor.pcurves(), pcurve_handles);
    assert_eq!(descriptor.certificate().carrier(), &carrier);
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(surfaces[0]),
            GeometryRef::Surface(surfaces[1]),
            GeometryRef::Curve2d(pcurve_handles[0]),
            GeometryRef::Curve2d(pcurve_handles[1]),
        ]
    );
    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(
        eval.eval_curve(curve, 0.5, 1).unwrap().d[0],
        carrier.eval(0.5)
    );

    let dependent = vec![GeometryRef::Curve(curve)];
    assert_eq!(
        graph.replace_surface(surfaces[0], planes()[0]),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(surfaces[0]),
            dependents: dependent.clone(),
        })
    );
    assert_eq!(
        graph.replace_curve2d(pcurve_handles[1], pcurves[1].clone()),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Curve2d(pcurve_handles[1]),
            dependents: dependent,
        })
    );
    graph.validate().unwrap();
}

#[test]
fn transmitted_plane_chart_binds_nested_offset_identity_and_protects_its_basis_chain() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurves = [
        NurbsCurve2d::new(
            1,
            knots.clone(),
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
        NurbsCurve2d::new(
            1,
            knots,
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
    ];
    let metadata =
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap();
    let certificate = certify_transmitted_plane_intersection_residuals(
        carrier,
        planes(),
        pcurves.clone(),
        metadata,
        1.0e-12,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let basis_plane = Plane::new(Frame::world().with_origin(Vec3::new(0.0, 0.0, -0.5)));
    let basis = graph.insert_surface(basis_plane).unwrap();
    let inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
        .unwrap();
    let outer = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner, 0.25))
        .unwrap();
    let vertical = graph.insert_surface(planes()[1]).unwrap();
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0].clone()).unwrap(),
        graph.insert_curve2d(pcurves[1].clone()).unwrap(),
    ];

    let wrong_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
        .unwrap();
    let curve_count = graph.curve_count();
    assert!(matches!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            [wrong_offset, vertical],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { class, .. })
            if class == CurveClass::Intersection.key()
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_transmitted_plane_intersection_curve(
            [outer, vertical],
            pcurve_handles,
            certificate,
        )
        .unwrap();
    let intersection = graph
        .curve(curve)
        .unwrap()
        .as_transmitted_intersection()
        .unwrap();
    assert_eq!(intersection.source_surfaces(), [outer, vertical]);
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(outer),
            GeometryRef::Surface(vertical),
            GeometryRef::Curve2d(pcurve_handles[0]),
            GeometryRef::Curve2d(pcurve_handles[1]),
        ]
    );
    assert_eq!(
        graph.replace_surface(basis, basis_plane),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(basis),
            dependents: vec![GeometryRef::Surface(inner)],
        })
    );
    graph.validate().unwrap();
}

#[test]
fn transmitted_plane_chart_binds_two_independent_offset_roots_and_both_basis_chains() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurves = [
        NurbsCurve2d::new(
            1,
            knots.clone(),
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
        NurbsCurve2d::new(
            1,
            knots,
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
    ];
    let certificate = certify_transmitted_plane_intersection_residuals(
        carrier,
        planes(),
        pcurves.clone(),
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap(),
        1.0e-12,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let basis_a_plane = Plane::new(Frame::world().with_origin(Vec3::new(0.0, 0.0, -0.5)));
    let basis_a = graph.insert_surface(basis_a_plane).unwrap();
    let inner_a = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_a, 0.25))
        .unwrap();
    let root_a = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner_a, 0.25))
        .unwrap();

    let vertical_frame = *planes()[1].frame();
    let basis_b_plane =
        Plane::new(vertical_frame.with_origin(vertical_frame.origin() - vertical_frame.z() * 0.75));
    let basis_b = graph.insert_surface(basis_b_plane).unwrap();
    let inner_b = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis_b, 0.25))
        .unwrap();
    let root_b = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner_b, 0.5))
        .unwrap();
    let altered_b = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner_b, 0.625))
        .unwrap();
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0].clone()).unwrap(),
        graph.insert_curve2d(pcurves[1].clone()).unwrap(),
    ];

    let curve_count = graph.curve_count();
    assert!(matches!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            [root_a, altered_b],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { class, .. })
            if class == CurveClass::Intersection.key()
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let stale_basis = graph.insert_surface(planes()[0]).unwrap();
    let stale = graph
        .insert_surface(OffsetSurfaceDescriptor::new(stale_basis, 0.0))
        .unwrap();
    graph.remove_surface(stale).unwrap();
    assert!(matches!(
        graph.insert_verified_transmitted_plane_intersection_curve(
            [stale, root_b],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(handle)
        }) if handle == stale
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_transmitted_plane_intersection_curve(
            [root_a, root_b],
            pcurve_handles,
            certificate,
        )
        .unwrap();
    let intersection = graph
        .curve(curve)
        .unwrap()
        .as_transmitted_intersection()
        .unwrap();
    assert_eq!(intersection.source_surfaces(), [root_a, root_b]);
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(root_a),
            GeometryRef::Surface(root_b),
            GeometryRef::Curve2d(pcurve_handles[0]),
            GeometryRef::Curve2d(pcurve_handles[1]),
        ]
    );
    for (basis, inner, plane) in [
        (basis_a, inner_a, basis_a_plane),
        (basis_b, inner_b, basis_b_plane),
    ] {
        assert_eq!(
            graph.replace_surface(basis, plane),
            Err(GeometryGraphError::HasDependents {
                geometry: GeometryRef::Surface(basis),
                dependents: vec![GeometryRef::Surface(inner)],
            })
        );
    }
    graph.validate().unwrap();
}

#[test]
fn transmitted_plane_nurbs_chart_certifies_nonplanar_polynomial_and_rational_sources() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let plane_pcurve = NurbsCurve2d::new(
        1,
        knots.clone(),
        vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        None,
    )
    .unwrap();
    let surface_pcurve = plane_pcurve.clone();
    let metadata =
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap();

    for (rational, swapped) in [(false, false), (true, false), (true, true)] {
        let surface = nonplanar_trace_surface(rational);
        let (traces, pcurves) = if swapped {
            (
                [
                    TransmittedPlaneNurbsTrace::Nurbs(surface.clone()),
                    TransmittedPlaneNurbsTrace::Plane(Plane::new(Frame::world())),
                ],
                [surface_pcurve.clone(), plane_pcurve.clone()],
            )
        } else {
            (
                [
                    TransmittedPlaneNurbsTrace::Plane(Plane::new(Frame::world())),
                    TransmittedPlaneNurbsTrace::Nurbs(surface.clone()),
                ],
                [plane_pcurve.clone(), surface_pcurve.clone()],
            )
        };
        let certificate = certify_transmitted_plane_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            metadata,
            1.0e-6,
        )
        .unwrap();
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= 1.0e-6)
        );

        let mut graph = GeometryGraph::new();
        let plane = graph.insert_surface(Plane::new(Frame::world())).unwrap();
        let nurbs = graph.insert_surface(surface.clone()).unwrap();
        let sources = if swapped {
            [nurbs, plane]
        } else {
            [plane, nurbs]
        };
        let pcurve_handles = [
            graph.insert_curve2d(pcurves[0].clone()).unwrap(),
            graph.insert_curve2d(pcurves[1].clone()).unwrap(),
        ];
        let wrong_surface = graph
            .insert_surface(nonplanar_trace_surface(!rational))
            .unwrap();
        let wrong_sources = if swapped {
            [wrong_surface, plane]
        } else {
            [plane, wrong_surface]
        };
        let curve_count = graph.curve_count();
        assert!(matches!(
            graph.insert_verified_transmitted_nurbs_intersection_curve(
                wrong_sources,
                pcurve_handles,
                certificate.clone(),
            ),
            Err(GeometryGraphError::InvalidDescriptor { .. })
        ));
        assert_eq!(graph.curve_count(), curve_count);

        let curve = graph
            .insert_verified_transmitted_plane_nurbs_intersection_curve(
                sources,
                pcurve_handles,
                certificate,
            )
            .unwrap();
        let intersection = graph
            .curve(curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        assert_eq!(intersection.source_surfaces(), sources);
        assert_eq!(intersection.pcurves(), pcurve_handles);
        assert_eq!(intersection.certificate().carrier(), &carrier);
        assert_eq!(
            graph.replace_surface(nurbs, surface),
            Err(GeometryGraphError::HasDependents {
                geometry: GeometryRef::Surface(nurbs),
                dependents: vec![GeometryRef::Curve(curve)],
            })
        );
        graph.validate().unwrap();
    }
}

#[test]
fn transmitted_nurbs_nurbs_chart_binds_two_original_sources_in_both_orders() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurve = NurbsCurve2d::new(
        1,
        knots,
        vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        None,
    )
    .unwrap();
    let metadata =
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap();

    for (rational_a, rational_b, swapped) in [
        (false, false, false),
        (true, false, false),
        (false, true, true),
        (true, true, true),
    ] {
        let surface_a = nonplanar_trace_surface(rational_a);
        let surface_b = second_nonplanar_trace_surface(rational_b);
        let traces = if swapped {
            [
                TransmittedPlaneNurbsTrace::Nurbs(surface_b.clone()),
                TransmittedPlaneNurbsTrace::Nurbs(surface_a.clone()),
            ]
        } else {
            [
                TransmittedPlaneNurbsTrace::Nurbs(surface_a.clone()),
                TransmittedPlaneNurbsTrace::Nurbs(surface_b.clone()),
            ]
        };
        let pcurves = [pcurve.clone(), pcurve.clone()];
        let certificate = certify_transmitted_nurbs_nurbs_intersection_residuals(
            carrier.clone(),
            traces,
            pcurves.clone(),
            metadata,
            1.0e-6,
        )
        .unwrap();
        assert_eq!(certificate.proof_depth(), 10);
        assert!(
            certificate
                .residual_bounds()
                .into_iter()
                .all(|bound| bound <= certificate.tolerance())
        );

        let mut graph = GeometryGraph::new();
        let source_a = graph.insert_surface(surface_a.clone()).unwrap();
        let source_b = graph.insert_surface(surface_b.clone()).unwrap();
        let sources = if swapped {
            [source_b, source_a]
        } else {
            [source_a, source_b]
        };
        let pcurve_handles = [
            graph.insert_curve2d(pcurves[0].clone()).unwrap(),
            graph.insert_curve2d(pcurves[1].clone()).unwrap(),
        ];
        let altered = graph
            .insert_surface(second_nonplanar_trace_surface(!rational_b))
            .unwrap();
        let altered_sources = if swapped {
            [altered, source_a]
        } else {
            [source_a, altered]
        };
        let curve_count = graph.curve_count();
        assert!(matches!(
            graph.insert_verified_transmitted_nurbs_intersection_curve(
                altered_sources,
                pcurve_handles,
                certificate.clone(),
            ),
            Err(GeometryGraphError::InvalidDescriptor { class, .. })
                if class == CurveClass::Intersection.key()
        ));
        assert_eq!(graph.curve_count(), curve_count);

        let stale = graph
            .insert_surface(second_nonplanar_trace_surface(rational_b))
            .unwrap();
        graph.remove_surface(stale).unwrap();
        let stale_sources = if swapped {
            [stale, source_a]
        } else {
            [source_a, stale]
        };
        assert!(matches!(
            graph.insert_verified_transmitted_plane_nurbs_intersection_curve(
                stale_sources,
                pcurve_handles,
                certificate.clone(),
            ),
            Err(GeometryGraphError::StaleGeometryHandle {
                geometry: GeometryRef::Surface(handle)
            }) if handle == stale
        ));
        assert_eq!(graph.curve_count(), curve_count);

        let curve = graph
            .insert_verified_transmitted_nurbs_intersection_curve(
                sources,
                pcurve_handles,
                certificate,
            )
            .unwrap();
        let intersection = graph
            .curve(curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap();
        assert_eq!(intersection.source_surfaces(), sources);
        assert_eq!(intersection.pcurves(), pcurve_handles);
        assert_eq!(intersection.certificate().carrier(), &carrier);
        assert_eq!(
            graph
                .direct_dependencies(GeometryRef::Curve(curve))
                .unwrap(),
            vec![
                GeometryRef::Surface(sources[0]),
                GeometryRef::Surface(sources[1]),
                GeometryRef::Curve2d(pcurve_handles[0]),
                GeometryRef::Curve2d(pcurve_handles[1]),
            ]
        );
        for source in sources {
            assert_eq!(
                graph.replace_surface(
                    source,
                    if source == source_a {
                        surface_a.clone()
                    } else {
                        surface_b.clone()
                    },
                ),
                Err(GeometryGraphError::HasDependents {
                    geometry: GeometryRef::Surface(source),
                    dependents: vec![GeometryRef::Curve(curve)],
                })
            );
        }
        graph.validate().unwrap();
    }
}

#[test]
fn transmitted_offset_nurbs_trace_proves_normal_field_and_binds_root_and_basis() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurve = NurbsCurve2d::new(
        1,
        knots.clone(),
        vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
        None,
    )
    .unwrap();
    let surface_knots = vec![0.0, 0.0, 1.0, 1.0];
    let offset_basis = NurbsSurface::new(
        1,
        1,
        surface_knots.clone(),
        surface_knots.clone(),
        vec![
            Vec3::new(0.0, 0.0, -0.25),
            Vec3::new(0.0, 1.0, -0.25),
            Vec3::new(1.0, 0.0, -0.25),
            Vec3::new(1.0, 1.0, -0.25),
        ],
        None,
    )
    .unwrap();
    let direct = NurbsSurface::new(
        1,
        1,
        surface_knots.clone(),
        surface_knots,
        vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 1.0),
        ],
        None,
    )
    .unwrap();
    let traces = [
        TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
            offset_basis.clone(),
            0.25,
        )),
        TransmittedPlaneNurbsTrace::Nurbs(direct.clone()),
    ];
    let certificate = certify_transmitted_offset_nurbs_intersection_residuals(
        carrier.clone(),
        traces,
        [pcurve.clone(), pcurve.clone()],
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap(),
        1.0e-10,
    )
    .unwrap();
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound < 1.0e-10)
    );

    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(offset_basis.clone()).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.25))
        .unwrap();
    let altered_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.5))
        .unwrap();
    let altered_basis = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![
            Vec3::new(0.0, 0.0, -0.375),
            Vec3::new(0.0, 1.0, -0.375),
            Vec3::new(1.0, 0.0, -0.375),
            Vec3::new(1.0, 1.0, -0.375),
        ],
        None,
    )
    .unwrap();
    let altered_basis = graph.insert_surface(altered_basis).unwrap();
    let same_distance_altered_basis = graph
        .insert_surface(OffsetSurfaceDescriptor::new(altered_basis, 0.25))
        .unwrap();
    let direct_handle = graph.insert_surface(direct).unwrap();
    let pcurves = [
        graph.insert_curve2d(pcurve.clone()).unwrap(),
        graph.insert_curve2d(pcurve).unwrap(),
    ];
    assert!(matches!(
        graph.insert_verified_transmitted_nurbs_intersection_curve(
            [altered_offset, direct_handle],
            pcurves,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert!(matches!(
        graph.insert_verified_transmitted_nurbs_intersection_curve(
            [same_distance_altered_basis, direct_handle],
            pcurves,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    let curve = graph
        .insert_verified_transmitted_nurbs_intersection_curve(
            [offset, direct_handle],
            pcurves,
            certificate,
        )
        .unwrap();
    assert_eq!(
        graph.replace_surface(basis, offset_basis),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(basis),
            dependents: vec![GeometryRef::Surface(offset)],
        })
    );
    assert!(
        graph
            .curve(curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .is_some()
    );
    graph.validate().unwrap();

    let singular = NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Vec3::new(0.0, 0.0, 0.0); 4],
        None,
    )
    .unwrap();
    assert!(matches!(
        certify_transmitted_offset_nurbs_intersection_residuals(
            carrier,
            [
                TransmittedPlaneNurbsTrace::OffsetNurbs(TransmittedOffsetNurbsTrace::new(
                    singular, 0.25,
                )),
                TransmittedPlaneNurbsTrace::Nurbs(nonplanar_trace_surface(false)),
            ],
            [
                NurbsCurve2d::new(
                    1,
                    knots.clone(),
                    vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
                    None,
                )
                .unwrap(),
                NurbsCurve2d::new(
                    1,
                    knots,
                    vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
                    None,
                )
                .unwrap(),
            ],
            TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap(),
            1.0e-6,
        ),
        Err(IntersectionCertificateError::SingularOffsetNormal { .. })
    ));
}

#[test]
fn transmitted_nurbs_plane_trace_binds_nested_offset_root_and_rejects_altered_or_stale_sources() {
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let carrier = NurbsCurve::new(
        1,
        knots.clone(),
        vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    let pcurves = [
        NurbsCurve2d::new(
            1,
            knots.clone(),
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
        NurbsCurve2d::new(
            1,
            knots,
            vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)],
            None,
        )
        .unwrap(),
    ];
    let surface = nonplanar_trace_surface(true);
    let certificate = certify_transmitted_plane_nurbs_intersection_residuals(
        carrier,
        [
            TransmittedPlaneNurbsTrace::Plane(Plane::new(Frame::world())),
            TransmittedPlaneNurbsTrace::Nurbs(surface.clone()),
        ],
        pcurves.clone(),
        TransmittedIntersectionChartMetadata::new(0.0, 1.0, 0.0, 0.0, [None, None]).unwrap(),
        1.0e-6,
    )
    .unwrap();

    let mut graph = GeometryGraph::new();
    let shifted_basis = Plane::new(Frame::world().with_origin(Vec3::new(0.0, 0.0, -0.25)));
    let basis = graph.insert_surface(shifted_basis).unwrap();
    let inner = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.125))
        .unwrap();
    let root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner, 0.125))
        .unwrap();
    let altered_root = graph
        .insert_surface(OffsetSurfaceDescriptor::new(inner, 0.25))
        .unwrap();
    let nurbs = graph.insert_surface(surface).unwrap();
    let pcurve_handles = [
        graph.insert_curve2d(pcurves[0].clone()).unwrap(),
        graph.insert_curve2d(pcurves[1].clone()).unwrap(),
    ];

    let curve_count = graph.curve_count();
    assert!(matches!(
        graph.insert_verified_transmitted_plane_nurbs_intersection_curve(
            [altered_root, nurbs],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let stale = graph.insert_surface(Plane::new(Frame::world())).unwrap();
    graph.remove_surface(stale).unwrap();
    assert!(matches!(
        graph.insert_verified_transmitted_plane_nurbs_intersection_curve(
            [stale, nurbs],
            pcurve_handles,
            certificate.clone(),
        ),
        Err(GeometryGraphError::StaleGeometryHandle {
            geometry: GeometryRef::Surface(handle)
        }) if handle == stale
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_transmitted_plane_nurbs_intersection_curve(
            [root, nurbs],
            pcurve_handles,
            certificate,
        )
        .unwrap();
    assert_eq!(
        graph
            .curve(curve)
            .unwrap()
            .as_transmitted_nurbs_intersection()
            .unwrap()
            .source_surfaces(),
        [root, nurbs]
    );
    assert!(matches!(
        graph.remove_surface(root),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(handle),
            ..
        }) if handle == root
    ));
    assert!(matches!(
        graph.remove_surface(basis),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(handle),
            ..
        }) if handle == basis
    ));
    assert!(matches!(
        graph.replace_surface(basis, shifted_basis),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(handle),
            dependents,
        }) if handle == basis && dependents == vec![GeometryRef::Surface(inner)]
    ));
    graph.validate().unwrap();
}

#[test]
fn exact_offset_plane_fields_are_accounted_and_bindable_to_persistent_proof() {
    let basis_plane = planes()[0];
    let effective_plane = Plane::new(
        Frame::new(
            Vec3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap(),
    );
    let vertical_plane = planes()[1];
    let shifted_carrier = Line::new(Vec3::new(0.0, 0.0, 0.5), Vec3::new(1.0, 0.0, 0.0)).unwrap();
    let shifted_pcurves = [
        Line2d::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap(),
        Line2d::new(Vec2::new(0.0, -0.5), Vec2::new(1.0, 0.0)).unwrap(),
    ];
    let range = ParamRange::new(-2.0, 2.0);
    let certificate = certify_paired_plane_line_residuals(
        shifted_carrier,
        range,
        [effective_plane, vertical_plane],
        shifted_pcurves,
        identity_maps(),
        1.0e-12,
    )
    .unwrap();
    let mut graph = GeometryGraph::new();
    let basis = graph.insert_surface(basis_plane).unwrap();
    let offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(basis, 0.5))
        .unwrap();
    let vertical = graph.insert_surface(vertical_plane).unwrap();
    let pcurve_handles = [
        graph.insert_curve2d(shifted_pcurves[0]).unwrap(),
        graph.insert_curve2d(shifted_pcurves[1]).unwrap(),
    ];

    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(eval.surface_exact_plane(offset), Ok(Some(effective_plane)));
    assert_eq!(eval.last_query_usage().node_visits(), 2);
    assert_eq!(eval.last_query_usage().dependency_depth(), 2);
    let curve = graph
        .insert_verified_plane_intersection_curve([offset, vertical], pcurve_handles, certificate)
        .unwrap();
    assert_eq!(
        graph
            .curve(curve)
            .unwrap()
            .as_intersection()
            .unwrap()
            .source_surfaces(),
        [offset, vertical]
    );
    assert_eq!(
        graph.replace_surface(basis, basis_plane),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(basis),
            dependents: vec![GeometryRef::Surface(offset)],
        })
    );
    graph.validate().unwrap();

    let tilted_frame = Frame::new(
        Vec3::new(2.0, -3.0, 5.0),
        Vec3::new(0.3, 0.4, 0.5),
        Vec3::new(1.0, 0.2, 0.0),
    )
    .unwrap();
    let tilted_plane = Plane::new(tilted_frame);
    let expected_frame = tilted_frame.with_origin(tilted_frame.origin() + tilted_frame.z() * 0.125);
    let mut tilted_graph = GeometryGraph::new();
    let tilted_basis = tilted_graph.insert_surface(tilted_plane).unwrap();
    let tilted_offset = tilted_graph
        .insert_surface(OffsetSurfaceDescriptor::new(tilted_basis, 0.125))
        .unwrap();
    let mut tilted_eval =
        EvalContext::new(&tilted_graph, EvalLimits::default(), Tolerances::default());
    let exact = tilted_eval
        .surface_exact_plane(tilted_offset)
        .unwrap()
        .unwrap();
    assert_eq!(exact, Plane::new(expected_frame));
    assert_eq!(exact.frame().x(), tilted_frame.x());
    assert_eq!(exact.frame().y(), tilted_frame.y());
    assert_eq!(exact.frame().z(), tilted_frame.z());
}

#[test]
fn frame_aligned_plane_sphere_circle_traces_certify_the_complete_range_in_both_orders() {
    let (carrier, plane, sphere, plane_pcurve, sphere_pcurve, traces) =
        aligned_circle_fixture(2.0, 0.5);
    let range = ParamRange::new(0.25, 5.75);
    let certificate =
        certify_paired_plane_sphere_circle_residuals(carrier, range, traces, 1.0e-12).unwrap();
    assert_eq!(certificate.carrier(), carrier);
    assert_eq!(certificate.carrier_range(), range);
    assert_eq!(certificate.traces(), traces);
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
    for parameter in [range.lo, range.lerp(0.37), range.hi] {
        let point = carrier.eval(parameter);
        let plane_uv = plane_pcurve.eval(parameter);
        let sphere_uv = sphere_pcurve.eval(parameter);
        assert!(point.dist(plane.eval([plane_uv.x, plane_uv.y])) <= 1.0e-12);
        assert!(point.dist(sphere.eval([sphere_uv.x, sphere_uv.y])) <= 1.0e-12);
    }

    let reversed = certify_paired_plane_sphere_circle_residuals(
        carrier,
        range,
        [traces[1], traces[0]],
        1.0e-12,
    )
    .unwrap();
    assert_eq!(reversed.traces(), [traces[1], traces[0]]);
    assert_eq!(
        reversed.residual_bounds(),
        [
            certificate.residual_bounds()[1],
            certificate.residual_bounds()[0]
        ]
    );
}

#[test]
fn oblique_spherical_pcurve_certifies_derivatives_persistence_and_source_binding() {
    let (carrier, plane, sphere, plane_pcurve) = oblique_circle_fixture();
    let range = ParamRange::new(0.2, 2.8);
    let sphere_window = [
        ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI),
        ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2),
    ];
    let (sphere_pcurve, certificate) = certify_paired_plane_sphere_oblique_circle_residuals(
        carrier,
        range,
        plane,
        plane_pcurve,
        sphere,
        sphere_window,
        [
            sphere_longitude(carrier, sphere, range.lo),
            sphere_longitude(carrier, sphere, range.hi),
        ],
        PairedTrace::First,
        1.0e-10,
    )
    .unwrap();
    assert_eq!(sphere_pcurve.carrier(), carrier);
    assert_eq!(sphere_pcurve.sphere(), sphere);
    assert_eq!(sphere_pcurve.carrier_range(), range);
    assert_eq!(sphere_pcurve.chart_window(), sphere_window);
    assert!(matches!(
        certificate.traces(),
        [
            PlaneSphereCircleTrace::Plane(_),
            PlaneSphereCircleTrace::SphereOblique(_)
        ]
    ));
    assert!(
        certificate
            .residual_bounds()
            .into_iter()
            .all(|bound| bound <= certificate.tolerance())
    );
    let bounds = sphere_pcurve.bounding_box(range);
    for parameter in [range.lo, range.lerp(0.37), range.lerp(0.81), range.hi] {
        let derivatives = sphere_pcurve.eval_derivs(parameter, 3);
        assert!(bounds.contains(derivatives.d[0]));
        assert!(
            derivatives
                .d
                .iter()
                .all(|value| value.x.is_finite() && value.y.is_finite())
        );
        assert!(
            carrier
                .eval(parameter)
                .dist(sphere.eval([derivatives.d[0].x, derivatives.d[0].y]))
                <= certificate.tolerance()
        );
    }

    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let sphere_handle = graph.insert_surface(sphere).unwrap();
    let pcurves = [
        graph.insert_curve2d(plane_pcurve).unwrap(),
        graph
            .insert_curve2d(Curve2dDescriptor::SphericalCircle(sphere_pcurve))
            .unwrap(),
    ];
    let curve = graph
        .insert_verified_plane_sphere_intersection_curve(
            [plane_handle, sphere_handle],
            pcurves,
            certificate,
        )
        .unwrap();
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(plane_handle),
            GeometryRef::Surface(sphere_handle),
            GeometryRef::Curve2d(pcurves[0]),
            GeometryRef::Curve2d(pcurves[1]),
        ]
    );
    let mut evaluator = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(evaluator.curve_param_range(curve), Ok(range));
    for parameter in [range.lo, range.lerp(0.43), range.hi] {
        assert_eq!(
            evaluator.eval_curve(curve, parameter, 3).unwrap().d[0],
            carrier.eval(parameter)
        );
    }
    assert_eq!(
        evaluator.curve_bounds(curve, range).unwrap(),
        carrier.bounding_box(range)
    );
    graph.validate().unwrap();

    let mut wrong_graph = GeometryGraph::new();
    let wrong_plane = wrong_graph.insert_surface(plane).unwrap();
    let wrong_sphere = wrong_graph
        .insert_surface(Sphere::new(Frame::world(), 2.75).unwrap())
        .unwrap();
    let wrong_pcurves = [
        wrong_graph.insert_curve2d(plane_pcurve).unwrap(),
        wrong_graph
            .insert_curve2d(Curve2dDescriptor::SphericalCircle(sphere_pcurve))
            .unwrap(),
    ];
    assert!(matches!(
        wrong_graph.insert_verified_plane_sphere_intersection_curve(
            [wrong_plane, wrong_sphere],
            wrong_pcurves,
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert_eq!(wrong_graph.curve_count(), 0);
    wrong_graph.validate().unwrap();
}

#[test]
fn oblique_spherical_pcurve_fails_closed_at_poles_and_outside_chart_windows() {
    let (carrier, plane, sphere, plane_pcurve) = oblique_circle_fixture();
    let range = ParamRange::new(0.2, 2.8);
    let endpoint_longitudes = [
        sphere_longitude(carrier, sphere, range.lo),
        sphere_longitude(carrier, sphere, range.hi),
    ];
    assert!(matches!(
        certify_paired_plane_sphere_oblique_circle_residuals(
            carrier,
            range,
            plane,
            plane_pcurve,
            sphere,
            [ParamRange::new(-3.0, 3.0), ParamRange::new(0.0, 0.01)],
            endpoint_longitudes,
            PairedTrace::First,
            1.0e-10,
        ),
        Err(IntersectionCertificateError::SphereTraceOutsideWindow {
            coordinate: "latitude"
        })
    ));

    let pole_plane = Plane::new(
        Frame::new(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .unwrap(),
    );
    let pole_carrier = Circle::new(*pole_plane.frame(), sphere.radius()).unwrap();
    let pole_pcurve =
        Circle2d::new(Vec2::new(0.0, 0.0), sphere.radius(), Vec2::new(1.0, 0.0)).unwrap();
    let pole_range = ParamRange::new(0.0, core::f64::consts::PI);
    assert!(matches!(
        certify_paired_plane_sphere_oblique_circle_residuals(
            pole_carrier,
            pole_range,
            pole_plane,
            pole_pcurve,
            sphere,
            [
                ParamRange::new(-core::f64::consts::PI, core::f64::consts::PI),
                ParamRange::new(-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2,),
            ],
            [
                sphere_longitude(pole_carrier, sphere, pole_range.lo),
                sphere_longitude(pole_carrier, sphere, pole_range.hi),
            ],
            PairedTrace::First,
            1.0e-10,
        ),
        Err(IntersectionCertificateError::SingularSphereChart { .. })
    ));
}

#[test]
fn rotated_and_antialigned_plane_charts_certify_shifted_longitude_ranges() {
    let sphere = Sphere::new(Frame::world(), 2.0).unwrap();
    let height = 0.5;
    let radius = (sphere.radius() * sphere.radius() - height * height).sqrt();
    let carrier = Circle::new(
        sphere.frame().with_origin(Vec3::new(0.0, 0.0, height)),
        radius,
    )
    .unwrap();
    let latitude = kcore::math::atan2(height, radius);
    let sphere_pcurve = Line2d::new(Vec2::new(0.0, latitude), Vec2::new(1.0, 0.0)).unwrap();
    let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
    let range = ParamRange::new(
        4.0 * core::f64::consts::TAU + 0.25,
        4.0 * core::f64::consts::TAU + 5.75,
    );
    let rotated_x = Vec3::new(0.6, 0.8, 0.0);

    for orientation in [1.0, -1.0] {
        let plane = Plane::new(
            Frame::new(
                Vec3::new(0.0, 0.0, height),
                Vec3::new(0.0, 0.0, orientation),
                rotated_x,
            )
            .unwrap(),
        );
        let plane_x = Vec2::new(
            sphere.frame().x().dot(plane.frame().x()),
            sphere.frame().x().dot(plane.frame().y()),
        );
        let plane_pcurve = Circle2d::new(Vec2::new(0.0, 0.0), radius, plane_x).unwrap();
        let plane_map = AffineParamMap1d::new(orientation, 0.0).unwrap();
        let traces = [
            PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, plane_map)),
            PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(
                sphere,
                sphere_pcurve,
                identity,
            )),
        ];
        let certificate =
            certify_paired_plane_sphere_circle_residuals(carrier, range, traces, 1.0e-12).unwrap();
        assert_eq!(certificate.parameter_maps(), [plane_map, identity]);
        for parameter in [range.lo, range.lerp(0.37), range.hi] {
            let point = carrier.eval(parameter);
            let plane_uv = plane_pcurve.eval(plane_map.map(parameter));
            let sphere_uv = sphere_pcurve.eval(parameter);
            assert!(point.dist(plane.eval([plane_uv.x, plane_uv.y])) <= 1.0e-12);
            assert!(point.dist(sphere.eval([sphere_uv.x, sphere_uv.y])) <= 1.0e-12);
        }
    }
}

#[test]
fn nonlinear_certificate_rejects_wrong_families_charts_and_perturbed_latitudes() {
    let (carrier, plane, sphere, plane_pcurve, sphere_pcurve, traces) =
        aligned_circle_fixture(2.0, 0.5);
    let range = ParamRange::new(0.0, core::f64::consts::TAU);
    assert_eq!(
        certify_paired_plane_sphere_circle_residuals(
            carrier,
            range,
            [traces[0], traces[0]],
            1.0e-12,
        ),
        Err(IntersectionCertificateError::InvalidTraceFamily)
    );

    let shifted_longitude = Line2d::new(
        Vec2::new(0.25, sphere_pcurve.origin().y),
        Vec2::new(1.0, 0.0),
    )
    .unwrap();
    let identity = AffineParamMap1d::new(1.0, 0.0).unwrap();
    let shifted_trace = PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(
        sphere,
        shifted_longitude,
        identity,
    ));
    assert!(matches!(
        certify_paired_plane_sphere_circle_residuals(
            carrier,
            range,
            [traces[0], shifted_trace],
            1.0e-12,
        ),
        Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::Second,
                ..
            }
        )
    ));

    let perturbed = Line2d::new(
        Vec2::new(0.0, sphere_pcurve.origin().y + 0.01),
        Vec2::new(1.0, 0.0),
    )
    .unwrap();
    let perturbed_trace =
        PlaneSphereCircleTrace::Sphere(SphereLatitudeTrace::new(sphere, perturbed, identity));
    assert!(matches!(
        certify_paired_plane_sphere_circle_residuals(
            carrier,
            range,
            [
                PlaneSphereCircleTrace::Plane(
                    PlaneCircleTrace::new(plane, plane_pcurve, identity,)
                ),
                perturbed_trace,
            ],
            1.0e-6,
        ),
        Err(IntersectionCertificateError::ResidualExceedsTolerance {
            trace: PairedTrace::Second,
            ..
        })
    ));

    let nonidentity = AffineParamMap1d::new(2.0, 0.0).unwrap();
    let mapped_plane =
        PlaneSphereCircleTrace::Plane(PlaneCircleTrace::new(plane, plane_pcurve, nonidentity));
    assert!(matches!(
        certify_paired_plane_sphere_circle_residuals(
            carrier,
            range,
            [mapped_plane, traces[1]],
            1.0e-12,
        ),
        Err(
            IntersectionCertificateError::UnsupportedTraceParameterization {
                trace: PairedTrace::First,
                ..
            }
        )
    ));
}

#[test]
fn persistent_circle_binds_safe_offset_sphere_and_rejects_invalid_radius_or_mutation() {
    let (carrier, plane, effective_sphere, plane_pcurve, sphere_pcurve, traces) =
        aligned_circle_fixture(2.5, 0.5);
    let range = ParamRange::new(0.2, 5.8);
    let certificate =
        certify_paired_plane_sphere_circle_residuals(carrier, range, traces, 1.0e-12).unwrap();
    let mut graph = GeometryGraph::new();
    let plane_handle = graph.insert_surface(plane).unwrap();
    let sphere_basis = graph
        .insert_surface(Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let sphere_offset = graph
        .insert_surface(OffsetSurfaceDescriptor::new(sphere_basis, 0.5))
        .unwrap();
    let pcurves = [
        graph.insert_curve2d(plane_pcurve).unwrap(),
        graph.insert_curve2d(sphere_pcurve).unwrap(),
    ];
    let wrong_pcurve = graph
        .insert_curve2d(
            Circle2d::new(
                Vec2::new(0.1, 0.0),
                plane_pcurve.radius(),
                Vec2::new(1.0, 0.0),
            )
            .unwrap(),
        )
        .unwrap();
    let curve_count = graph.curve_count();
    assert!(matches!(
        graph.insert_verified_plane_sphere_intersection_curve(
            [plane_handle, sphere_offset],
            [wrong_pcurve, pcurves[1]],
            certificate,
        ),
        Err(GeometryGraphError::InvalidDescriptor { .. })
    ));
    assert_eq!(graph.curve_count(), curve_count);

    let curve = graph
        .insert_verified_plane_sphere_intersection_curve(
            [plane_handle, sphere_offset],
            pcurves,
            certificate,
        )
        .unwrap();
    let descriptor = graph.curve(curve).unwrap().as_intersection().unwrap();
    assert_eq!(
        descriptor.carrier(),
        VerifiedIntersectionCarrier::Circle(carrier)
    );
    assert_eq!(
        descriptor.certificate(),
        VerifiedIntersectionCertificate::PlaneSphereCircle(certificate)
    );
    assert_eq!(descriptor.source_surfaces(), [plane_handle, sphere_offset]);
    assert_eq!(
        graph
            .direct_dependencies(GeometryRef::Curve(curve))
            .unwrap(),
        vec![
            GeometryRef::Surface(plane_handle),
            GeometryRef::Surface(sphere_offset),
            GeometryRef::Curve2d(pcurves[0]),
            GeometryRef::Curve2d(pcurves[1]),
        ]
    );
    let mut eval = EvalContext::new(&graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(
        eval.surface_exact_field(sphere_offset),
        Ok(Some(kgraph::ExactSurfaceField::Sphere(effective_sphere)))
    );
    assert_eq!(eval.last_query_usage().node_visits(), 2);
    assert_eq!(eval.curve_param_range(curve), Ok(range));
    for parameter in [range.lo, range.lerp(0.41), range.hi] {
        assert_eq!(
            eval.eval_curve(curve, parameter, 1).unwrap().d[0],
            carrier.eval(parameter)
        );
    }
    assert_eq!(
        eval.curve_bounds(curve, range).unwrap(),
        carrier.bounding_box(range)
    );
    assert_eq!(
        graph.replace_surface(sphere_basis, Sphere::new(Frame::world(), 2.0).unwrap()),
        Err(GeometryGraphError::HasDependents {
            geometry: GeometryRef::Surface(sphere_basis),
            dependents: vec![GeometryRef::Surface(sphere_offset)],
        })
    );

    let mut invalid_graph = GeometryGraph::new();
    let invalid_basis = invalid_graph
        .insert_surface(Sphere::new(Frame::world(), 2.0).unwrap())
        .unwrap();
    let invalid_offset = invalid_graph
        .insert_surface(OffsetSurfaceDescriptor::new(invalid_basis, -2.0))
        .unwrap();
    let mut invalid_eval =
        EvalContext::new(&invalid_graph, EvalLimits::default(), Tolerances::default());
    assert_eq!(invalid_eval.surface_exact_field(invalid_offset), Ok(None));
    assert_eq!(invalid_eval.last_query_usage().node_visits(), 2);
    graph.validate().unwrap();
}
