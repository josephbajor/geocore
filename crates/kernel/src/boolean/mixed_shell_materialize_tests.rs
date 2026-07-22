use super::super::{
    MixedBoundedSourceRoot, MixedShellEdgeUse, MixedShellLoopPlan, ProjectedSourceCircleOnPlane,
};
use super::*;

struct ProjectedPcurveFixture {
    store: Store,
    plan: MixedShellProofPlan,
    physical: PhysicalEdge,
}

fn projected_pcurve_fixture(upper: bool) -> ProjectedPcurveFixture {
    projected_pcurve_fixture_with_target_x(upper, kgeom::vec::Vec3::new(1.0, 0.0, 0.0))
}

fn projected_pcurve_fixture_with_target_x(
    upper: bool,
    target_x: kgeom::vec::Vec3,
) -> ProjectedPcurveFixture {
    let mut store = Store::new();
    let source_body = ktopo::make::cylinder(
        &mut store,
        &Frame::world().with_origin(Point3::new(-0.5, 0.0, -1.0)),
        1.0,
        2.0,
    )
    .unwrap();
    let target_body = ktopo::make::cylinder(
        &mut store,
        &Frame::new(
            Point3::new(0.5, 0.0, -1.0),
            kgeom::vec::Vec3::new(0.0, 0.0, 1.0),
            target_x,
        )
        .unwrap(),
        1.0,
        2.0,
    )
    .unwrap();
    let source_raw_face = store
        .faces_of_body(source_body)
        .unwrap()
        .into_iter()
        .find(|face| {
            matches!(
                store.surface(store.get(*face).unwrap().surface()).unwrap(),
                SurfaceGeom::Cylinder(_)
            )
        })
        .unwrap();
    let target_raw_face = store
        .faces_of_body(target_body)
        .unwrap()
        .into_iter()
        .find(|face| {
            let Ok(SurfaceGeom::Plane(plane)) = store.surface(store.get(*face).unwrap().surface())
            else {
                return false;
            };
            (plane.frame().z().z > 0.0) == upper
        })
        .unwrap();
    let target_z = if upper { 1.0 } else { -1.0 };
    let (source_loop, source_fin, source_edge) = store
        .get(source_raw_face)
        .unwrap()
        .loops()
        .iter()
        .find_map(|loop_id| {
            let loop_ = store.get(*loop_id).unwrap();
            let [fin_id] = loop_.fins() else {
                return None;
            };
            let edge_id = store.get(*fin_id).unwrap().edge();
            let curve_id = store.get(edge_id).unwrap().curve()?;
            let CurveGeom::Circle(circle) = store.curve(curve_id).unwrap() else {
                return None;
            };
            (circle.frame().origin().z == target_z).then_some((*loop_id, *fin_id, edge_id))
        })
        .unwrap();

    let mut session = crate::Kernel::new().create_session();
    let part = session.create_part();
    let source_face = crate::FaceId::new(part.clone(), source_raw_face);
    let target_face = crate::FaceId::new(part, target_raw_face);
    let source = MixedSourceFaceKey {
        operand: 0,
        topology_ordinal: 0,
    };
    let target = MixedSourceFaceKey {
        operand: 1,
        topology_ordinal: usize::from(upper),
    };
    let span = MixedSourceSpanKey {
        fin_loop_ordinal: usize::from(upper),
        traversal_ordinal: 0,
    };
    let root = |endpoint, parameter: f64| MixedBoundedSourceRoot {
        endpoint,
        root_ordinal: endpoint,
        parameter_bits: parameter.to_bits(),
        enclosure_bits: [parameter.to_bits(), parameter.to_bits()],
        period_shift: 0,
    };
    let bounded = MixedBoundedSourceSpanPlan {
        source,
        span: span.clone(),
        loop_id: source_loop,
        fin: source_fin,
        edge: source_edge,
        roots: [root(0, 0.25), root(1, 2.25)],
    };
    let proof = ProjectedSourceCircleOnPlane::certify(
        &store,
        &source_face,
        &bounded,
        target,
        &target_face,
        kcore::tolerance::LINEAR_RESOLUTION,
    )
    .unwrap();
    let edge = MixedShellEdgeKey::PlanarSource {
        source,
        span: span.clone(),
    };
    let vertices = vec![
        MixedShellVertexKey::SectionEndpoint(0),
        MixedShellVertexKey::SectionEndpoint(1),
    ];
    let source_loop_plan = MixedShellLoopPlan {
        uses: vec![MixedShellEdgeUse {
            edge: edge.clone(),
            direction: ArrangementDirection::Forward,
            pcurve: MixedPcurveLineage::SourceTopology,
        }],
        vertices: vertices.clone(),
    };
    let target_loop_plan = MixedShellLoopPlan {
        uses: vec![MixedShellEdgeUse {
            edge,
            direction: ArrangementDirection::Reverse,
            pcurve: MixedPcurveLineage::ProjectedSourceCircleOnPlane(proof),
        }],
        vertices,
    };
    let retained = RetainedPlanarSpan {
        source,
        span,
        loop_id: source_loop,
        fin: source_fin,
        edge: source_edge,
        range: [
            RetainedSpanParameter::SectionRoot {
                endpoint: 0,
                enclosure_bits: [0.25_f64.to_bits(), 0.25_f64.to_bits()],
                parameter_bits: 0.25_f64.to_bits(),
                period_shift: 0,
            },
            RetainedSpanParameter::SectionRoot {
                endpoint: 1,
                enclosure_bits: [2.25_f64.to_bits(), 2.25_f64.to_bits()],
                parameter_bits: 2.25_f64.to_bits(),
                period_shift: 0,
            },
        ],
    };
    let plan = MixedShellProofPlan {
        faces: vec![
            MixedShellFacePlan {
                source,
                source_face,
                selected_orientation: SelectedOrientation::Preserved,
                loops: vec![source_loop_plan],
            },
            MixedShellFacePlan {
                source: target,
                source_face: target_face,
                selected_orientation: SelectedOrientation::Preserved,
                loops: vec![target_loop_plan],
            },
        ],
        section_edges: Vec::new(),
        bounded_source_spans: vec![bounded],
        cap_rings: Vec::new(),
        materialization: RetainedMaterializationEvidence {
            source_spans: vec![retained],
            section_trims: Vec::new(),
        },
        materialization_gaps: Vec::new(),
    };
    let physical = PhysicalEdge {
        carrier: PhysicalCarrier::Source(source_edge),
        endpoints: Some([PhysicalVertex::Section(0), PhysicalVertex::Section(1)]),
        uses: vec![
            PhysicalUse {
                face: 0,
                loop_index: 0,
                use_index: 0,
                forward: true,
            },
            PhysicalUse {
                face: 1,
                loop_index: 0,
                use_index: 0,
                forward: false,
            },
        ],
    };
    ProjectedPcurveFixture {
        store,
        plan,
        physical,
    }
}

fn emit_projected_pcurve(
    fixture: &ProjectedPcurveFixture,
) -> Result<AnalyticPcurveUse, MixedShellMaterializationError> {
    let use_ = &fixture.plan.faces[1].loops[0].uses[0];
    materialized_pcurve_for_use(
        &fixture.plan,
        &fixture.store,
        1,
        0,
        0,
        use_,
        &fixture.physical,
    )
}

#[test]
fn projected_source_circle_emits_exact_plane_coefficients_in_both_orientations() {
    for (upper, expected_scale) in [(false, -1.0), (true, 1.0)] {
        let fixture = projected_pcurve_fixture(upper);
        let emitted = emit_projected_pcurve(&fixture).unwrap();
        let AnalyticShellPcurve::Circle(pcurve_circle) = emitted.curve() else {
            panic!("projected source ring must emit a Circle2d");
        };
        assert_eq!(pcurve_circle.center(), Point2::new(-1.0, 0.0));
        assert_eq!(pcurve_circle.radius(), 1.0);
        assert_eq!(pcurve_circle.x_dir(), Vec2::new(1.0, 0.0));
        assert_eq!(emitted.edge_to_pcurve().scale(), expected_scale);
        assert_eq!(
            emitted.edge_to_pcurve().offset().to_bits(),
            0.0_f64.to_bits()
        );
        assert_eq!(emitted.chart(), PcurveChart::identity());
        assert_eq!(emitted.closure_winding(), None);
        let MixedPcurveLineage::ProjectedSourceCircleOnPlane(proof) =
            fixture.plan.faces[1].loops[0].uses[0].pcurve()
        else {
            panic!("fixture must retain projected-circle proof");
        };
        assert_eq!(proof.center(), Point2::new(-1.0, 0.0));
        assert_eq!(proof.x_direction().unwrap(), Vec2::new(1.0, 0.0));
        assert_eq!(proof.parameter_scale(), expected_scale);
        assert_eq!(proof.parameter_offset().to_bits(), 0.0_f64.to_bits());

        let target = fixture
            .store
            .get(fixture.plan.faces[1].source_face.raw())
            .unwrap();
        let SurfaceGeom::Plane(plane) = fixture.store.surface(target.surface()).unwrap() else {
            panic!("fixture target must remain planar");
        };
        let PhysicalCarrier::Source(edge) = fixture.physical.carrier else {
            panic!("fixture carrier must remain the source ring");
        };
        let curve = fixture.store.get(edge).unwrap().curve().unwrap();
        let CurveGeom::Circle(source_circle) = fixture.store.curve(curve).unwrap() else {
            panic!("fixture carrier must remain circular");
        };
        for parameter in [0.0, core::f64::consts::FRAC_PI_2, core::f64::consts::PI] {
            let mapped = emitted.edge_to_pcurve().map(parameter);
            let uv = pcurve_circle.eval(mapped);
            let on_plane = plane.eval([uv.x, uv.y]);
            assert!((on_plane - source_circle.eval(parameter)).norm() <= 2.0e-15);
        }
    }
}

#[test]
fn projected_source_circle_uses_the_target_planes_authored_in_plane_chart() {
    let fixture =
        projected_pcurve_fixture_with_target_x(true, kgeom::vec::Vec3::new(0.0, 1.0, 0.0));
    let emitted = emit_projected_pcurve(&fixture).unwrap();
    let AnalyticShellPcurve::Circle(circle) = emitted.curve() else {
        panic!("projected source ring must emit a Circle2d");
    };
    assert_eq!(circle.center(), Point2::new(0.0, 1.0));
    assert_eq!(circle.x_dir(), Vec2::new(0.0, -1.0));
    assert_eq!(emitted.edge_to_pcurve().scale(), 1.0);
    assert_eq!(emitted.edge_to_pcurve().offset(), 0.0);
}

#[test]
fn projected_source_circle_certifies_oblique_co_and_antiparallel_cap_supports() {
    let common = Frame::new(
        Point3::new(2.5, -1.75, 0.625),
        kgeom::vec::Vec3::new(0.48, 0.64, 0.6),
        kgeom::vec::Vec3::new(0.8, -0.6, 0.0),
    )
    .unwrap();
    for reverse_target in [false, true] {
        let mut store = Store::new();
        let source_body = ktopo::make::cylinder(
            &mut store,
            &common.with_origin(common.point_at(-0.5, 0.0, -1.0)),
            1.0,
            2.0,
        )
        .unwrap();
        let target_frame = if reverse_target {
            Frame::new(common.point_at(0.5, 0.0, 1.0), -common.z(), common.x()).unwrap()
        } else {
            common.with_origin(common.point_at(0.5, 0.0, -1.0))
        };
        let target_body = ktopo::make::cylinder(&mut store, &target_frame, 1.0, 2.0).unwrap();
        let source_raw_face = store
            .faces_of_body(source_body)
            .unwrap()
            .into_iter()
            .find(|face| {
                matches!(
                    store.surface(store.get(*face).unwrap().surface()).unwrap(),
                    SurfaceGeom::Cylinder(_)
                )
            })
            .unwrap();
        for (height, expected_scale) in [(-1.0, -1.0), (1.0, 1.0)] {
            let target_raw_face = store
                .faces_of_body(target_body)
                .unwrap()
                .into_iter()
                .find(|face| {
                    let Ok(SurfaceGeom::Plane(plane)) =
                        store.surface(store.get(*face).unwrap().surface())
                    else {
                        return false;
                    };
                    (common.to_local(plane.frame().origin()).z - height).abs() <= 1.0e-15
                })
                .unwrap();
            let (source_loop, source_fin, source_edge) = store
                .get(source_raw_face)
                .unwrap()
                .loops()
                .iter()
                .find_map(|loop_id| {
                    let [fin_id] = store.get(*loop_id).unwrap().fins() else {
                        return None;
                    };
                    let edge = store.get(*fin_id).unwrap().edge();
                    let curve = store.get(edge).unwrap().curve()?;
                    let CurveGeom::Circle(circle) = store.curve(curve).unwrap() else {
                        return None;
                    };
                    ((common.to_local(circle.frame().origin()).z - height).abs() <= 1.0e-15)
                        .then_some((*loop_id, *fin_id, edge))
                })
                .unwrap();
            let mut session = crate::Kernel::new().create_session();
            let part = session.create_part();
            let source_face = crate::FaceId::new(part.clone(), source_raw_face);
            let target_face = crate::FaceId::new(part, target_raw_face);
            let bounded = MixedBoundedSourceSpanPlan {
                source: source(),
                span: MixedSourceSpanKey {
                    fin_loop_ordinal: usize::from(height > 0.0),
                    traversal_ordinal: 0,
                },
                loop_id: source_loop,
                fin: source_fin,
                edge: source_edge,
                roots: [
                    MixedBoundedSourceRoot {
                        endpoint: 0,
                        root_ordinal: 0,
                        parameter_bits: 0.25_f64.to_bits(),
                        enclosure_bits: [0.25_f64.to_bits(); 2],
                        period_shift: 0,
                    },
                    MixedBoundedSourceRoot {
                        endpoint: 1,
                        root_ordinal: 1,
                        parameter_bits: 2.25_f64.to_bits(),
                        enclosure_bits: [2.25_f64.to_bits(); 2],
                        period_shift: 0,
                    },
                ],
            };
            let proof = ProjectedSourceCircleOnPlane::certify(
                &store,
                &source_face,
                &bounded,
                MixedSourceFaceKey {
                    operand: 1,
                    topology_ordinal: usize::from(height > 0.0),
                },
                &target_face,
                kcore::tolerance::LINEAR_RESOLUTION,
            )
            .unwrap();
            assert_eq!(proof.parameter_scale(), expected_scale);
            assert_eq!(proof.parameter_offset(), 0.0);
            assert!((proof.center() - Point2::new(-1.0, 0.0)).norm() <= 2.0e-15);
            assert_eq!(proof.tolerance(), kcore::tolerance::LINEAR_RESOLUTION);
        }
    }
}

#[test]
fn projected_source_circle_recertification_rejects_every_rebound_identity() {
    let assert_binding_refusal = |fixture: &ProjectedPcurveFixture| {
        assert_eq!(
            emit_projected_pcurve(fixture),
            Err(
                MixedShellMaterializationError::ProjectedSourceCircleOnPlane(
                    ProjectedSourceCircleOnPlaneError::SourceTopologyMismatch,
                )
            )
        );
    };

    let mut rebound_target = projected_pcurve_fixture(true);
    rebound_target.plan.faces[1].source = source();
    assert_binding_refusal(&rebound_target);

    let mut rebound_peer = projected_pcurve_fixture(true);
    rebound_peer.plan.faces[0].loops[0].uses[0].pcurve = MixedPcurveLineage::Section {
        branch: 0,
        operand: 0,
        cylinder_period_shift: 0,
    };
    assert_binding_refusal(&rebound_peer);

    let mut rebound_span = projected_pcurve_fixture(true);
    let source_raw = rebound_span.plan.faces[0].source_face.raw();
    let other_loop = rebound_span
        .store
        .get(source_raw)
        .unwrap()
        .loops()
        .iter()
        .copied()
        .find(|loop_id| *loop_id != rebound_span.plan.materialization.source_spans[0].loop_id)
        .unwrap();
    rebound_span.plan.materialization.source_spans[0].loop_id = other_loop;
    assert_binding_refusal(&rebound_span);
}

#[test]
fn projected_source_circle_certification_rejects_foreign_parts_and_other_planes() {
    let fixture = projected_pcurve_fixture(true);
    let source_face = &fixture.plan.faces[0].source_face;
    let target_face = &fixture.plan.faces[1].source_face;
    let source_span = &fixture.plan.bounded_source_spans[0];
    let target_key = fixture.plan.faces[1].source;

    let mut session = crate::Kernel::new().create_session();
    let foreign_part = session.create_part();
    let foreign_target = crate::FaceId::new(foreign_part, target_face.raw());
    assert_eq!(
        ProjectedSourceCircleOnPlane::certify(
            &fixture.store,
            source_face,
            source_span,
            target_key,
            &foreign_target,
            kcore::tolerance::LINEAR_RESOLUTION,
        ),
        Err(ProjectedSourceCircleOnPlaneError::FacePartMismatch)
    );

    let other_plane = fixture
        .store
        .iter::<ktopo::entity::Face>()
        .find_map(|(face_id, face)| {
            let SurfaceGeom::Plane(plane) = fixture.store.surface(face.surface()).ok()? else {
                return None;
            };
            (plane.frame().z().z < 0.0).then_some(face_id)
        })
        .unwrap();
    let other_target = crate::FaceId::new(source_face.part().clone(), other_plane);
    assert_eq!(
        ProjectedSourceCircleOnPlane::certify(
            &fixture.store,
            source_face,
            source_span,
            target_key,
            &other_target,
            kcore::tolerance::LINEAR_RESOLUTION,
        ),
        Err(ProjectedSourceCircleOnPlaneError::CircleNotOnPlane)
    );
}

fn source() -> MixedSourceFaceKey {
    MixedSourceFaceKey {
        operand: 0,
        topology_ordinal: 0,
    }
}

fn root_parameter(canonical: f64, period_shift: i32) -> RetainedSpanParameter {
    RetainedSpanParameter::SectionRoot {
        endpoint: 7,
        enclosure_bits: [canonical.to_bits(), canonical.to_bits()],
        parameter_bits: canonical.to_bits(),
        period_shift,
    }
}

#[test]
fn source_root_period_lift_preserves_canonical_scalar_authority() {
    let canonical = 0.25;
    let period = core::f64::consts::TAU;
    let mut candidates = BTreeMap::from([(SourceRootScalarKey::new(0, 7), canonical)]);
    let lifted = source_parameter(
        source(),
        &root_parameter(canonical, 1),
        &mut candidates,
        Some(period),
    )
    .unwrap();
    assert_eq!(lifted.to_bits(), (canonical + period).to_bits());
    assert!(candidates.is_empty());
}

#[test]
fn source_root_period_lift_refuses_a_nonperiodic_carrier() {
    assert_eq!(
        source_parameter(
            source(),
            &root_parameter(0.25, -1),
            &mut BTreeMap::new(),
            None,
        ),
        Err(MixedShellMaterializationError::InvalidSourcePeriodLift)
    );
}

#[test]
fn bounded_source_span_drops_whole_loop_closure_winding() {
    let curve =
        AnalyticShellPcurve::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap());
    let map = AffineParamMap1d::new(1.0, 0.0).unwrap();
    let source = AnalyticPcurveUse::new(curve, map);
    assert_eq!(
        apply_source_closure_winding(source, Some([1, 0]), false).closure_winding(),
        None
    );
    assert_eq!(
        apply_source_closure_winding(source, Some([1, 0]), true).closure_winding(),
        Some([1, 0])
    );
}

#[test]
fn periodic_window_keeps_the_authored_chart_when_every_interval_fits() {
    let period = core::f64::consts::TAU;
    let authored = ParamRange::new(0.0, period);
    let selected = select_common_periodic_window(period, authored, &[(0.25, 2.0)]).unwrap();
    assert_eq!(selected.lo.to_bits(), authored.lo.to_bits());
    assert_eq!(selected.hi.to_bits(), authored.hi.to_bits());
}

#[test]
fn periodic_window_rotates_into_an_open_gap_for_a_seam_crossing_arc() {
    let pi = core::f64::consts::PI;
    let period = core::f64::consts::TAU;
    let authored = ParamRange::new(0.0, period);
    let crossing = (5.0 * pi / 3.0, 7.0 * pi / 3.0);
    let selected = select_common_periodic_window(period, authored, &[crossing]).unwrap();
    assert_ne!(selected, authored);
    assert_eq!(selected.width(), period);
    let shift = periodic_interval_shift(period, selected, crossing).unwrap();
    let shifted = (
        crossing.0 + f64::from(shift) * period,
        crossing.1 + f64::from(shift) * period,
    );
    assert!(shifted.0 > selected.lo);
    assert!(shifted.1 < selected.hi);
}

#[test]
fn periodic_window_fails_closed_when_bounded_intervals_cover_every_seam() {
    let period = core::f64::consts::TAU;
    assert_eq!(
        select_common_periodic_window(
            period,
            ParamRange::new(0.0, period),
            &[(0.0, 4.0), (3.0, 7.0)],
        ),
        Err(MixedShellMaterializationError::NoCommonPeriodicWindow)
    );
}

fn bounded_periodic_line(origin: Point2, direction: Vec2) -> AnalyticPcurveUse {
    AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(Line2d::new(origin, direction).unwrap()),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    )
}

fn normalize_bounded_periodic_face(
    surface: AnalyticShellSurface,
    authored: ParamRange,
    uses: &[(AnalyticPcurveUse, ParamRange)],
) -> Result<(ParamRange, ktopo::entity::FaceDomain), MixedShellMaterializationError> {
    let period = surface_periodicity(surface)[0]
        .ok_or(MixedShellMaterializationError::InvalidAnalyticGeometry)?;
    let intervals = uses
        .iter()
        .map(|(pcurve, range)| {
            pcurve_bounds(surface, *pcurve, *range).map(|(min, max)| (min.x, max.x))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let window = select_common_periodic_window(period, authored, &intervals)?;
    let mut bounds = None;
    for (pcurve, range) in uses {
        let normalized = normalize_periodic_pcurve_chart(surface, window, *pcurve, *range)?;
        include_pcurve_bounds(&mut bounds, pcurve_bounds(surface, normalized, *range)?);
    }
    let (min, max) = bounds.ok_or(MixedShellMaterializationError::InvalidAnalyticGeometry)?;
    let domain = ktopo::entity::FaceDomain::from_bounds(min.x, max.x, min.y, max.y)
        .map_err(|_| MixedShellMaterializationError::InvalidAnalyticGeometry)?;
    Ok((window, domain))
}

#[test]
fn bounded_only_periodic_face_normalization_is_permutation_stable_and_fails_closed() {
    let period = core::f64::consts::TAU;
    let surface =
        AnalyticShellSurface::Cylinder(kgeom::surface::Cylinder::new(Frame::world(), 1.0).unwrap());
    let authored = ParamRange::new(0.0, period);
    let lower = period / 6.0;
    let split = 5.0 * period / 6.0;
    let upper = lower + period;
    assert_eq!(upper - lower, period);
    // A bounded four-edge Cylinder loop with two horizontal arcs authored on
    // opposite lifts. The arcs are complementary and the two axial rulings
    // join their shared fibers, so the noncontractible loop spans one period.
    let uses = vec![
        (
            bounded_periodic_line(Point2::new(0.0, -1.0), Vec2::new(1.0, 0.0)),
            ParamRange::new(split, upper),
        ),
        (
            bounded_periodic_line(Point2::new(upper, -1.0), Vec2::new(0.0, 1.0)),
            ParamRange::new(0.0, 2.0),
        ),
        (
            bounded_periodic_line(Point2::new(0.0, 1.0), Vec2::new(1.0, 0.0)),
            ParamRange::new(lower, split),
        ),
        (
            bounded_periodic_line(Point2::new(split, -1.0), Vec2::new(0.0, 1.0)),
            ParamRange::new(0.0, 2.0),
        ),
    ];
    let expected = normalize_bounded_periodic_face(surface, authored, &uses).unwrap();
    assert_eq!(expected.1.u, expected.0);
    assert_eq!(expected.0.width(), period);
    assert_eq!(expected.1.u.width(), period);

    let mut permuted = uses.clone();
    permuted.reverse();
    assert_eq!(
        normalize_bounded_periodic_face(surface, authored, &permuted).unwrap(),
        expected
    );

    let shifts = [-2, 1, 3, -1];
    let mut shifted_permutation = uses
        .iter()
        .zip(shifts)
        .map(|((pcurve, range), shift)| {
            (pcurve.with_chart(PcurveChart::shifted([shift, 0])), *range)
        })
        .collect::<Vec<_>>();
    shifted_permutation.rotate_left(1);
    shifted_permutation.reverse();
    let actual = normalize_bounded_periodic_face(surface, authored, &shifted_permutation).unwrap();
    assert_eq!(actual.1, expected.1);
    assert_eq!(actual.0.width(), period);
    assert_eq!(actual.1.u.width(), period);
    let chart_epsilon = 256.0 * f64::EPSILON * actual.0.hi.abs().max(1.0);
    assert!(actual.1.u.lo >= actual.0.lo - chart_epsilon);
    assert!(actual.1.u.hi <= actual.0.hi + chart_epsilon);

    let mut tampered = uses;
    tampered[0].1 = ParamRange::new(split, split + period + 0.25);
    assert_eq!(
        normalize_bounded_periodic_face(surface, authored, &tampered),
        Err(MixedShellMaterializationError::NoCommonPeriodicWindow)
    );
}

fn horizontal_ring_use(scale: f64, winding: [i32; 2]) -> AnalyticPcurveUse {
    AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(1.0, 0.0)).unwrap()),
        AffineParamMap1d::new(scale, 0.0).unwrap(),
    )
    .with_closure_winding(winding)
}

#[test]
fn endpoint_free_periodic_use_requires_exact_horizontal_full_winding() {
    let pi = core::f64::consts::PI;
    let period = core::f64::consts::TAU;
    let surface =
        AnalyticShellSurface::Cylinder(kgeom::surface::Cylinder::new(Frame::world(), 1.0).unwrap());
    let shifted = ParamRange::new(pi / 3.0, 7.0 * pi / 3.0);
    assert_eq!(shifted.width(), period);
    validate_endpoint_free_periodic_use(surface, horizontal_ring_use(1.0, [1, 0]), shifted)
        .unwrap();

    assert_eq!(
        validate_endpoint_free_periodic_use(surface, horizontal_ring_use(1.0, [0, 0]), shifted,),
        Err(MixedShellMaterializationError::InvalidEndpointFreePeriodicUse)
    );
    assert_eq!(
        validate_endpoint_free_periodic_use(surface, horizontal_ring_use(0.5, [1, 0]), shifted,),
        Err(MixedShellMaterializationError::InvalidEndpointFreePeriodicUse)
    );
    let vertical = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(Line2d::new(Point2::new(0.0, 0.0), Vec2::new(0.0, 1.0)).unwrap()),
        AffineParamMap1d::new(1.0, 0.0).unwrap(),
    )
    .with_closure_winding([1, 0]);
    assert_eq!(
        validate_endpoint_free_periodic_use(surface, vertical, shifted),
        Err(MixedShellMaterializationError::InvalidEndpointFreePeriodicUse)
    );
    let inexact = ParamRange::new(5.0 * pi / 3.0, 5.0 * pi / 3.0 + period);
    assert_ne!(inexact.width(), period);
    assert_eq!(
        validate_endpoint_free_periodic_use(surface, horizontal_ring_use(1.0, [1, 0]), inexact,),
        Err(MixedShellMaterializationError::InvalidEndpointFreePeriodicUse)
    );
}

#[test]
fn periodic_window_work_preserves_source_store_errors() {
    let mut source_store = Store::new();
    let body = ktopo::make::block(&mut source_store, &Frame::world(), [1.0, 1.0, 1.0]).unwrap();
    let raw_face = source_store.faces_of_body(body).unwrap()[0];
    let mut session = crate::Kernel::new().create_session();
    let part = session.create_part();
    let plan = MixedShellProofPlan {
        faces: vec![MixedShellFacePlan {
            source: source(),
            source_face: crate::FaceId::new(part, raw_face),
            selected_orientation: SelectedOrientation::Preserved,
            loops: Vec::new(),
        }],
        section_edges: Vec::new(),
        bounded_source_spans: Vec::new(),
        cap_rings: Vec::new(),
        materialization: RetainedMaterializationEvidence {
            source_spans: Vec::new(),
            section_trims: Vec::new(),
        },
        materialization_gaps: Vec::new(),
    };

    assert_eq!(
        prepare_mixed_shell_materialization(&plan, &Store::new()),
        Err(MixedShellMaterializationError::StoreRead)
    );
}

#[test]
fn periodic_window_ceiling_accepts_n_and_refuses_n_minus_one_or_one_more_candidate() {
    use crate::boolean::pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationPolicyError, ResourceKind,
        WorkLedger,
    };

    let exact = periodic_face_window_work(5, 3, 2).unwrap();
    let bounded_only = periodic_face_window_work(4, 4, 0).unwrap();
    let one_more_candidate = periodic_face_window_work(6, 4, 2).unwrap();
    assert!(one_more_candidate > exact);
    assert_eq!(bounded_only, 289);
    assert_eq!(periodic_window_application_work(10, 3), Some(19));
    assert_eq!(periodic_window_application_work(10, 4), Some(26));
    let ledger_at = |allowed| {
        WorkLedger::new(
            BudgetPlan::new([LimitSpec::new(
                PLANAR_BOOLEAN_REALIZATION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap(),
        )
    };

    let mut admitted = ledger_at(exact);
    admitted
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, exact)
        .unwrap();
    let mut bounded_admitted = ledger_at(bounded_only);
    bounded_admitted
        .charge(PLANAR_BOOLEAN_REALIZATION_WORK, bounded_only)
        .unwrap();

    let assert_refusal = |allowed, consumed| {
        let mut ledger = ledger_at(allowed);
        assert_eq!(
            ledger.charge(PLANAR_BOOLEAN_REALIZATION_WORK, consumed),
            Err(OperationPolicyError::LimitReached(LimitSnapshot {
                stage: PLANAR_BOOLEAN_REALIZATION_WORK,
                resource: ResourceKind::Work,
                consumed,
                allowed,
            }))
        );
    };
    assert_refusal(exact - 1, exact);
    assert_refusal(bounded_only - 1, bounded_only);
    assert_refusal(exact, one_more_candidate);
}
