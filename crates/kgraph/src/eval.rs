//! Bounded, fallible graph evaluation.

use kcore::tolerance::Tolerances;
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::{Curve, CurveDerivs};
use kgeom::curve2d::{Curve2d, Curve2dDerivs};
use kgeom::param::ParamRange;
use kgeom::surface::{Degeneracy, Surface, SurfaceDerivs};

use crate::descriptor::{Curve2dDescriptor, CurveDescriptor, SurfaceDescriptor};
use crate::error::{EvalError, EvalResult};
use crate::graph::{Curve2dHandle, CurveHandle, GeometryGraph, GeometryRef, SurfaceHandle};

/// Work reserved for one public graph query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EvalLimits {
    /// Maximum active dependency stack depth, including the root.
    pub max_dependency_depth: usize,
    /// Maximum descriptor visits made by one public query.
    pub max_node_visits_per_query: usize,
}

impl Default for EvalLimits {
    fn default() -> Self {
        Self {
            max_dependency_depth: 64,
            max_node_visits_per_query: 4_096,
        }
    }
}

/// Requested exact surface derivative order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceDerivativeOrder {
    /// Position only.
    Position,
    /// Position and first partial derivatives.
    First,
    /// Position through second partial derivatives.
    Second,
}

impl SurfaceDerivativeOrder {
    const fn as_usize(self) -> usize {
        match self {
            Self::Position => 0,
            Self::First => 1,
            Self::Second => 2,
        }
    }
}

/// Cheap per-thread evaluator over an immutable graph.
///
/// Every public method starts a fresh query ledger. The context owns no cache,
/// session policy, topology state, executor, or diagnostic sink.
pub struct EvalContext<'g> {
    graph: &'g GeometryGraph,
    limits: EvalLimits,
    tolerances: Tolerances,
    active: Vec<GeometryRef>,
    node_visits: usize,
}

impl<'g> EvalContext<'g> {
    /// Construct an evaluator with explicit graph-recursion limits and model tolerances.
    pub fn new(graph: &'g GeometryGraph, limits: EvalLimits, tolerances: Tolerances) -> Self {
        Self {
            graph,
            limits,
            tolerances,
            active: Vec::new(),
            node_visits: 0,
        }
    }

    /// Graph being evaluated.
    pub const fn graph(&self) -> &'g GeometryGraph {
        self.graph
    }
    /// Per-query graph-recursion limits.
    pub const fn limits(&self) -> EvalLimits {
        self.limits
    }
    /// Model acceptance tolerances supplied by the caller.
    pub const fn tolerances(&self) -> Tolerances {
        self.tolerances
    }

    /// Evaluate a 3D curve through exact derivative `order` (0 through 3).
    pub fn eval_curve(
        &mut self,
        curve: CurveHandle,
        t: f64,
        order: usize,
    ) -> EvalResult<CurveDerivs> {
        self.begin_query();
        if !t.is_finite() {
            return Err(EvalError::InvalidParameter);
        }
        let geometry = GeometryRef::Curve(curve);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .curve(curve)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?
                .descriptor();
            if order > 3 {
                return Err(EvalError::DerivativeUnavailable {
                    class: descriptor.class_key(),
                    requested: order,
                });
            }
            let leaf = curve_leaf(descriptor);
            validate_parameter(t, leaf.param_range(), leaf.periodicity())?;
            let value = leaf.eval_derivs(t, order);
            if curve_derivs_finite(value, order) {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })();
        self.leave(geometry);
        result
    }

    /// Query a 3D curve's natural parameter range.
    pub fn curve_param_range(&mut self, curve: CurveHandle) -> EvalResult<ParamRange> {
        self.with_curve(curve, |descriptor| Ok(curve_leaf(descriptor).param_range()))
    }

    /// Query a 3D curve's period, if periodic.
    pub fn curve_periodicity(&mut self, curve: CurveHandle) -> EvalResult<Option<f64>> {
        self.with_curve(curve, |descriptor| Ok(curve_leaf(descriptor).periodicity()))
    }

    /// Bound a 3D curve over a finite in-domain range.
    pub fn curve_bounds(&mut self, curve: CurveHandle, range: ParamRange) -> EvalResult<Aabb3> {
        self.with_curve(curve, |descriptor| {
            let leaf = curve_leaf(descriptor);
            validate_range(range, leaf.param_range(), leaf.periodicity())?;
            let value = leaf.bounding_box(range);
            if value.is_finite() {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })
    }

    /// Evaluate a parameter-space curve through exact derivative `order` (0 through 3).
    pub fn eval_curve2d(
        &mut self,
        curve: Curve2dHandle,
        t: f64,
        order: usize,
    ) -> EvalResult<Curve2dDerivs> {
        self.begin_query();
        if !t.is_finite() {
            return Err(EvalError::InvalidParameter);
        }
        let geometry = GeometryRef::Curve2d(curve);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .curve2d(curve)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?
                .descriptor();
            if order > 3 {
                return Err(EvalError::DerivativeUnavailable {
                    class: descriptor.class_key(),
                    requested: order,
                });
            }
            let leaf = curve2d_leaf(descriptor);
            validate_parameter(t, leaf.param_range(), leaf.periodicity())?;
            let value = leaf.eval_derivs(t, order);
            if curve2d_derivs_finite(value, order) {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })();
        self.leave(geometry);
        result
    }

    /// Query a 2D curve's natural parameter range.
    pub fn curve2d_param_range(&mut self, curve: Curve2dHandle) -> EvalResult<ParamRange> {
        self.with_curve2d(curve, |descriptor| {
            Ok(curve2d_leaf(descriptor).param_range())
        })
    }

    /// Query a 2D curve's period, if periodic.
    pub fn curve2d_periodicity(&mut self, curve: Curve2dHandle) -> EvalResult<Option<f64>> {
        self.with_curve2d(curve, |descriptor| {
            Ok(curve2d_leaf(descriptor).periodicity())
        })
    }

    /// Bound a 2D curve over a finite in-domain range.
    pub fn curve2d_bounds(&mut self, curve: Curve2dHandle, range: ParamRange) -> EvalResult<Aabb2> {
        self.with_curve2d(curve, |descriptor| {
            let leaf = curve2d_leaf(descriptor);
            validate_range(range, leaf.param_range(), leaf.periodicity())?;
            let value = leaf.bounding_box(range);
            if aabb2_finite(value) {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })
    }

    /// Evaluate a surface through the requested exact derivative order.
    pub fn eval_surface(
        &mut self,
        surface: SurfaceHandle,
        uv: [f64; 2],
        order: SurfaceDerivativeOrder,
    ) -> EvalResult<SurfaceDerivs> {
        self.begin_query();
        if !uv.into_iter().all(f64::is_finite) {
            return Err(EvalError::InvalidParameter);
        }
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?
                .descriptor();
            let leaf = surface_leaf(descriptor);
            let domain = leaf.param_range();
            let periodicity = leaf.periodicity();
            validate_parameter(uv[0], domain[0], periodicity[0])?;
            validate_parameter(uv[1], domain[1], periodicity[1])?;
            let order = order.as_usize();
            let value = leaf.eval_derivs(uv, order);
            if surface_derivs_finite(value, order) {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })();
        self.leave(geometry);
        result
    }

    /// Query a surface's natural parameter ranges.
    pub fn surface_param_range(&mut self, surface: SurfaceHandle) -> EvalResult<[ParamRange; 2]> {
        self.with_surface(surface, |descriptor| {
            Ok(surface_leaf(descriptor).param_range())
        })
    }

    /// Query a surface's periods by parameter direction.
    pub fn surface_periodicity(&mut self, surface: SurfaceHandle) -> EvalResult<[Option<f64>; 2]> {
        self.with_surface(surface, |descriptor| {
            Ok(surface_leaf(descriptor).periodicity())
        })
    }

    /// Query exact degenerate iso-parameter lines advertised by the leaf.
    pub fn surface_degeneracies(&mut self, surface: SurfaceHandle) -> EvalResult<Vec<Degeneracy>> {
        self.with_surface(surface, |descriptor| {
            Ok(surface_leaf(descriptor).degeneracies())
        })
    }

    /// Bound a surface over finite in-domain parameter ranges.
    pub fn surface_bounds(
        &mut self,
        surface: SurfaceHandle,
        range: [ParamRange; 2],
    ) -> EvalResult<Aabb3> {
        self.with_surface(surface, |descriptor| {
            let leaf = surface_leaf(descriptor);
            let domain = leaf.param_range();
            let periodicity = leaf.periodicity();
            validate_range(range[0], domain[0], periodicity[0])?;
            validate_range(range[1], domain[1], periodicity[1])?;
            let value = leaf.bounding_box(range);
            if value.is_finite() {
                Ok(value)
            } else {
                Err(EvalError::NonFiniteResult {
                    class: descriptor.class_key(),
                })
            }
        })
    }

    fn with_curve<T>(
        &mut self,
        handle: CurveHandle,
        query: impl FnOnce(&CurveDescriptor) -> EvalResult<T>,
    ) -> EvalResult<T> {
        self.begin_query();
        let geometry = GeometryRef::Curve(handle);
        self.enter(geometry)?;
        let result = self
            .graph
            .curve(handle)
            .ok_or(EvalError::StaleGeometryHandle { geometry })
            .and_then(|node| query(node.descriptor()));
        self.leave(geometry);
        result
    }

    fn with_curve2d<T>(
        &mut self,
        handle: Curve2dHandle,
        query: impl FnOnce(&Curve2dDescriptor) -> EvalResult<T>,
    ) -> EvalResult<T> {
        self.begin_query();
        let geometry = GeometryRef::Curve2d(handle);
        self.enter(geometry)?;
        let result = self
            .graph
            .curve2d(handle)
            .ok_or(EvalError::StaleGeometryHandle { geometry })
            .and_then(|node| query(node.descriptor()));
        self.leave(geometry);
        result
    }

    fn with_surface<T>(
        &mut self,
        handle: SurfaceHandle,
        query: impl FnOnce(&SurfaceDescriptor) -> EvalResult<T>,
    ) -> EvalResult<T> {
        self.begin_query();
        let geometry = GeometryRef::Surface(handle);
        self.enter(geometry)?;
        let result = self
            .graph
            .surface(handle)
            .ok_or(EvalError::StaleGeometryHandle { geometry })
            .and_then(|node| query(node.descriptor()));
        self.leave(geometry);
        result
    }

    fn begin_query(&mut self) {
        self.active.clear();
        self.node_visits = 0;
    }

    fn enter(&mut self, geometry: GeometryRef) -> EvalResult<()> {
        self.node_visits += 1;
        if self.node_visits > self.limits.max_node_visits_per_query {
            return Err(EvalError::NodeVisitLimitExceeded {
                consumed: self.node_visits,
                limit: self.limits.max_node_visits_per_query,
            });
        }
        if let Some(start) = self
            .active
            .iter()
            .position(|candidate| *candidate == geometry)
        {
            let mut path = self.active[start..].to_vec();
            path.push(geometry);
            return Err(EvalError::DependencyCycle { path });
        }
        let consumed = self.active.len() + 1;
        if consumed > self.limits.max_dependency_depth {
            return Err(EvalError::DependencyDepthExceeded {
                consumed,
                limit: self.limits.max_dependency_depth,
            });
        }
        self.active.push(geometry);
        Ok(())
    }

    fn leave(&mut self, geometry: GeometryRef) {
        let popped = self.active.pop();
        debug_assert_eq!(popped, Some(geometry));
    }
}

fn curve_leaf(descriptor: &CurveDescriptor) -> &dyn Curve {
    match descriptor {
        CurveDescriptor::Line(v) => v,
        CurveDescriptor::Circle(v) => v,
        CurveDescriptor::Ellipse(v) => v,
        CurveDescriptor::Nurbs(v) => v,
    }
}

fn curve2d_leaf(descriptor: &Curve2dDescriptor) -> &dyn Curve2d {
    match descriptor {
        Curve2dDescriptor::Line(v) => v,
        Curve2dDescriptor::Circle(v) => v,
        Curve2dDescriptor::Nurbs(v) => v,
    }
}

fn surface_leaf(descriptor: &SurfaceDescriptor) -> &dyn Surface {
    match descriptor {
        SurfaceDescriptor::Plane(v) => v,
        SurfaceDescriptor::Cylinder(v) => v,
        SurfaceDescriptor::Cone(v) => v,
        SurfaceDescriptor::Sphere(v) => v,
        SurfaceDescriptor::Torus(v) => v,
        SurfaceDescriptor::Nurbs(v) => v,
    }
}

fn validate_parameter(value: f64, domain: ParamRange, periodicity: Option<f64>) -> EvalResult<()> {
    if !value.is_finite() {
        return Err(EvalError::InvalidParameter);
    }
    if periodicity.is_none() && !domain.contains(value) {
        return Err(EvalError::ParameterOutsideDomain);
    }
    Ok(())
}

fn validate_range(
    range: ParamRange,
    domain: ParamRange,
    periodicity: Option<f64>,
) -> EvalResult<()> {
    if !range.is_finite() || range.lo > range.hi {
        return Err(EvalError::InvalidRange);
    }
    if periodicity.is_none() && (!domain.contains(range.lo) || !domain.contains(range.hi)) {
        return Err(EvalError::InvalidRange);
    }
    Ok(())
}

fn finite2(v: kgeom::vec::Vec2) -> bool {
    v.x.is_finite() && v.y.is_finite()
}
fn finite3(v: kgeom::vec::Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn curve_derivs_finite(value: CurveDerivs, order: usize) -> bool {
    value.d[..=order].iter().copied().all(finite3)
}
fn curve2d_derivs_finite(value: Curve2dDerivs, order: usize) -> bool {
    value.d[..=order].iter().copied().all(finite2)
}
fn surface_derivs_finite(value: SurfaceDerivs, order: usize) -> bool {
    finite3(value.p)
        && (order < 1 || (finite3(value.du) && finite3(value.dv)))
        && (order < 2 || (finite3(value.duu) && finite3(value.duv) && finite3(value.dvv)))
}
fn aabb2_finite(value: Aabb2) -> bool {
    finite2(value.min) && finite2(value.max)
}
