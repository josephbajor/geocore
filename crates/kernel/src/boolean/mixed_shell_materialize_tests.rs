use super::*;

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
fn periodic_window_ceiling_accepts_n_and_refuses_n_minus_one_or_one_more_candidate() {
    use crate::boolean::pipeline::PLANAR_BOOLEAN_REALIZATION_WORK;
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSnapshot, LimitSpec, OperationPolicyError, ResourceKind,
        WorkLedger,
    };

    let exact = periodic_face_window_work(5, 3, 2).unwrap();
    let one_more_candidate = periodic_face_window_work(6, 4, 2).unwrap();
    assert!(one_more_candidate > exact);
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
    assert_refusal(exact, one_more_candidate);
}
