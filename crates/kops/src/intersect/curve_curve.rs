use super::circle_ellipse::intersect_bounded_circle_ellipse;
use super::circle_nurbs::intersect_bounded_circle_nurbs;
use super::ellipse_ellipse::intersect_bounded_ellipses_in_scope;
use super::ellipse_nurbs::intersect_bounded_ellipse_nurbs;
use super::error::{IntersectionError, IntersectionResult};
use super::geometry_class::CurveDispatch;
use super::line_circle::intersect_bounded_line_circle;
use super::line_ellipse::intersect_bounded_line_ellipse;
use super::line_line::intersect_bounded_lines;
use super::line_nurbs::intersect_bounded_line_nurbs;
use super::nurbs_nurbs::NurbsCurvePairSolveBudgetProfile;
use super::nurbs_nurbs::intersect_bounded_nurbs_nurbs_in_scope;
use super::result::CurveCurveIntersections;
use kcore::operation::{
    BudgetPlan, OperationContext, OperationOutcome, OperationScope, SessionPolicy,
};
use kgeom::curve::Curve;
use kgeom::param::ParamRange;
use kgeom::project::ProjectionBudgetProfile;

/// Version-1 composed budget for generic bounded curve/curve dispatch.
#[derive(Debug, Clone, Copy, Default)]
pub struct CurveCurveBudgetProfile;

impl CurveCurveBudgetProfile {
    /// Curve projection plus exact NURBS pair isolation and seed defaults.
    pub fn v1_defaults() -> BudgetPlan {
        let projection = ProjectionBudgetProfile::curve_aggregate_compatibility();
        let nurbs_pair = NurbsCurvePairSolveBudgetProfile::v1_defaults();
        BudgetPlan::new(
            projection
                .limits()
                .iter()
                .chain(nurbs_pair.limits())
                .copied(),
        )
        .expect("built-in curve/curve family budget is valid")
    }
}

/// Intersect two curves restricted to finite parameter ranges where needed.
///
/// This dispatches the currently supported analytic curve classes plus the
/// initial NURBS bridges. Unsupported curve classes fail explicitly; the
/// broader subdivision/Newton curve-curve solver remains later M4 work.
pub fn intersect_bounded_curves(
    a: &dyn Curve,
    range_a: ParamRange,
    b: &dyn Curve,
    range_b: ParamRange,
    tolerances: kcore::tolerance::Tolerances,
) -> IntersectionResult<CurveCurveIntersections> {
    let session = SessionPolicy::v1();
    let context = OperationContext::new(&session, tolerances)
        .expect("validated Tolerances always satisfy v1 session precision");
    intersect_bounded_curves_with_context(a, range_a, b, range_b, &context).into_result()
}

/// Intersect two bounded curves with caller-owned numerical policy and work accounting.
///
/// The complete curve-projection compatibility profile is composed before one
/// operation scope is created, so specialized ellipse projection can borrow
/// the same report. Analytic paths that require no iterative work leave those
/// counters at zero. NURBS/NURBS consumes the caller's numerical policy and
/// accounts exact isolation plus bounded cell-local seed attempts.
pub fn intersect_bounded_curves_with_context(
    a: &dyn Curve,
    range_a: ParamRange,
    b: &dyn Curve,
    range_b: ParamRange,
    context: &OperationContext<'_>,
) -> OperationOutcome<CurveCurveIntersections, IntersectionError> {
    let context = context
        .clone()
        .with_family_budget_defaults(CurveCurveBudgetProfile::v1_defaults());
    let mut scope = OperationScope::new(&context);
    let result = intersect_bounded_curves_in_scope(a, range_a, b, range_b, &mut scope);
    scope.finish_typed(result)
}

/// Intersect two bounded curves inside an existing owner operation scope.
///
/// Owners must compose [`CurveCurveBudgetProfile::v1_defaults`] before creating
/// `scope` so projection, certified NURBS pair isolation, and local seed
/// attempts share one report.
/// The function never creates or finishes a nested scope.
pub fn intersect_bounded_curves_in_scope(
    a: &dyn Curve,
    range_a: ParamRange,
    b: &dyn Curve,
    range_b: ParamRange,
    scope: &mut OperationScope<'_, '_>,
) -> IntersectionResult<CurveCurveIntersections> {
    let original_a = CurveDispatch::inspect(a);
    let original_b = CurveDispatch::inspect(b);
    let (Some(mut class_a), Some(mut class_b)) = (original_a, original_b) else {
        return Err(IntersectionError::UnsupportedCurvePair {
            class_a: original_a.map(|class| class.class().key()),
            class_b: original_b.map(|class| class.class().key()),
        });
    };
    let (mut range_a, mut range_b) = (range_a, range_b);
    let original_classes = [class_a.class().key(), class_b.class().key()];
    let swapped = class_a.class() > class_b.class();
    if swapped {
        core::mem::swap(&mut class_a, &mut class_b);
        core::mem::swap(&mut range_a, &mut range_b);
    }
    let tolerances = scope.context().tolerances();

    let result = match (class_a, class_b) {
        (CurveDispatch::Line(a), CurveDispatch::Line(b)) => {
            intersect_bounded_lines(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Circle(b)) => {
            intersect_bounded_line_circle(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Ellipse(b)) => {
            intersect_bounded_line_ellipse(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Line(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_line_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Circle(b)) => {
            super::circle_circle::intersect_bounded_circles(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Ellipse(b)) => {
            intersect_bounded_circle_ellipse(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Circle(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_circle_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Ellipse(a), CurveDispatch::Ellipse(b)) => {
            return intersect_bounded_ellipses_in_scope(a, range_a, b, range_b, scope)
                .map(|result| if swapped { result.swapped() } else { result });
        }
        (CurveDispatch::Ellipse(a), CurveDispatch::Nurbs(b)) => {
            intersect_bounded_ellipse_nurbs(a, range_a, b, range_b, tolerances)
        }
        (CurveDispatch::Nurbs(a), CurveDispatch::Nurbs(b)) => {
            return intersect_bounded_nurbs_nurbs_in_scope(a, range_a, b, range_b, scope)
                .map(|result| if swapped { result.swapped() } else { result });
        }
        _ => {
            return Err(IntersectionError::UnsupportedCurvePair {
                class_a: Some(original_classes[0]),
                class_b: Some(original_classes[1]),
            });
        }
    };
    result
        .map(|result| if swapped { result.swapped() } else { result })
        .map_err(IntersectionError::from)
}
