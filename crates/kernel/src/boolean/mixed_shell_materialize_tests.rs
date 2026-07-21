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
