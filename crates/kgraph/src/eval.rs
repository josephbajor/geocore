//! Bounded, fallible graph evaluation.

use kcore::tolerance::Tolerances;
use kgeom::aabb::{Aabb2, Aabb3};
use kgeom::curve::{Curve, CurveDerivs};
use kgeom::curve2d::{Curve2d, Curve2dDerivs};
use kgeom::param::ParamRange;
use kgeom::surface::{Degeneracy, Surface, SurfaceDerivs};

use crate::SurfaceClass;
use crate::descriptor::{Curve2dDescriptor, CurveDescriptor, SurfaceDescriptor};
use crate::error::{EvalError, EvalResult};
use crate::graph::{Curve2dHandle, CurveHandle, GeometryGraph, GeometryRef, SurfaceHandle};

/// Reason a surface regularity query could not certify regularity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidityGap {
    /// The normalized Jacobian is at or below angular conditioning tolerance.
    IllConditioned,
}

/// Pointwise surface regularity result.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SurfaceValidity {
    /// The parameterization is pointwise regular.
    Regular {
        /// `|du × dv| / (|du| |dv|)`.
        normalized_jacobian: f64,
    },
    /// The surface Jacobian is exactly singular.
    Singular,
    /// The Jacobian is nonzero but cannot be robustly certified regular.
    Indeterminate {
        /// Named reason for the proof gap.
        reason: ValidityGap,
    },
}

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
        self.eval_surface_inner(surface, uv, order, true)
    }

    /// Query a surface's natural parameter ranges.
    pub fn surface_param_range(&mut self, surface: SurfaceHandle) -> EvalResult<[ParamRange; 2]> {
        self.begin_query();
        self.surface_param_range_inner(surface)
    }

    /// Query a surface's periods by parameter direction.
    pub fn surface_periodicity(&mut self, surface: SurfaceHandle) -> EvalResult<[Option<f64>; 2]> {
        self.begin_query();
        self.surface_periodicity_inner(surface)
    }

    /// Query exact degenerate iso-parameter lines advertised by the leaf.
    pub fn surface_degeneracies(&mut self, surface: SurfaceHandle) -> EvalResult<Vec<Degeneracy>> {
        self.begin_query();
        self.surface_degeneracies_inner(surface)
    }

    /// Bound a surface over finite in-domain parameter ranges.
    pub fn surface_bounds(
        &mut self,
        surface: SurfaceHandle,
        range: [ParamRange; 2],
    ) -> EvalResult<Aabb3> {
        self.begin_query();
        self.surface_bounds_inner(surface, range)
    }

    /// Classify pointwise surface regularity without guessing across tolerance gaps.
    pub fn surface_validity(
        &mut self,
        surface: SurfaceHandle,
        uv: [f64; 2],
    ) -> EvalResult<SurfaceValidity> {
        self.begin_query();
        if !uv.into_iter().all(f64::is_finite) {
            return Err(EvalError::InvalidParameter);
        }
        let derivatives =
            self.eval_surface_inner(surface, uv, SurfaceDerivativeOrder::First, false)?;
        Ok(classify_jacobian(derivatives, self.tolerances))
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

    fn eval_surface_inner(
        &mut self,
        surface: SurfaceHandle,
        uv: [f64; 2],
        order: SurfaceDerivativeOrder,
        require_regular: bool,
    ) -> EvalResult<SurfaceDerivs> {
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?;
            match descriptor {
                SurfaceDescriptor::Offset(offset) => {
                    if order == SurfaceDerivativeOrder::Second {
                        return Err(EvalError::DerivativeUnavailable {
                            class: descriptor.class_key(),
                            requested: 2,
                        });
                    }
                    self.eval_offset_chain(surface, *offset, uv, require_regular)
                }
                _ => {
                    let leaf = surface_leaf(descriptor);
                    let domain = leaf.param_range();
                    let periodicity = leaf.periodicity();
                    validate_parameter(uv[0], domain[0], periodicity[0])?;
                    validate_parameter(uv[1], domain[1], periodicity[1])?;
                    let exact_order = order.as_usize();
                    let value = leaf.eval_derivs(uv, exact_order);
                    if surface_derivs_finite(value, exact_order) {
                        Ok(value)
                    } else {
                        Err(EvalError::NonFiniteResult {
                            class: descriptor.class_key(),
                        })
                    }
                }
            }
        })();
        self.leave(geometry);
        result
    }

    fn surface_param_range_inner(&mut self, surface: SurfaceHandle) -> EvalResult<[ParamRange; 2]> {
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?;
            if let SurfaceDescriptor::Offset(offset) = descriptor {
                let basis = offset.basis();
                self.surface_param_range_inner(basis)
            } else {
                Ok(surface_leaf(descriptor).param_range())
            }
        })();
        self.leave(geometry);
        result
    }

    fn eval_offset_chain(
        &mut self,
        root: SurfaceHandle,
        root_offset: crate::OffsetSurfaceDescriptor,
        uv: [f64; 2],
        require_final_regular: bool,
    ) -> EvalResult<SurfaceDerivs> {
        let mut chain = vec![(root, root_offset.signed_distance())];
        let mut entered = Vec::new();
        let result = (|| {
            let mut basis_handle = root_offset.basis();
            loop {
                let geometry = GeometryRef::Surface(basis_handle);
                self.enter(geometry)?;
                entered.push(geometry);
                let descriptor = self
                    .graph
                    .surface(basis_handle)
                    .ok_or(EvalError::StaleGeometryHandle { geometry })?;
                if let SurfaceDescriptor::Offset(offset) = descriptor {
                    chain.push((basis_handle, offset.signed_distance()));
                    basis_handle = offset.basis();
                } else {
                    let leaf = surface_leaf(descriptor);
                    let domain = leaf.param_range();
                    let periodicity = leaf.periodicity();
                    validate_parameter(uv[0], domain[0], periodicity[0])?;
                    validate_parameter(uv[1], domain[1], periodicity[1])?;
                    let basis = leaf.eval_derivs(uv, 2);
                    if !surface_derivs_finite(basis, 2) {
                        return Err(EvalError::NonFiniteResult {
                            class: descriptor.class_key(),
                        });
                    }
                    enforce_regular(basis, self.tolerances, basis_handle, uv)?;
                    let basis_normal = basis.du.cross(basis.dv);
                    let mut current = basis;
                    let mut effective_distance = 0.0;
                    let chain_len = chain.len();
                    for (index, &(node, distance)) in chain.iter().rev().enumerate() {
                        let orientation = if current
                            .du
                            .cross(current.dv)
                            .dot(basis_normal)
                            .is_sign_negative()
                        {
                            -1.0
                        } else {
                            1.0
                        };
                        effective_distance += orientation * distance;
                        current = offset_derivatives(basis, effective_distance)?;
                        let final_node = index + 1 == chain_len;
                        if !final_node || require_final_regular {
                            enforce_regular(current, self.tolerances, node, uv)?;
                        }
                    }
                    return Ok(current);
                }
            }
        })();
        for geometry in entered.into_iter().rev() {
            self.leave(geometry);
        }
        result
    }

    fn surface_periodicity_inner(
        &mut self,
        surface: SurfaceHandle,
    ) -> EvalResult<[Option<f64>; 2]> {
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?;
            if let SurfaceDescriptor::Offset(offset) = descriptor {
                let basis = offset.basis();
                self.surface_periodicity_inner(basis)
            } else {
                Ok(surface_leaf(descriptor).periodicity())
            }
        })();
        self.leave(geometry);
        result
    }

    fn surface_degeneracies_inner(
        &mut self,
        surface: SurfaceHandle,
    ) -> EvalResult<Vec<Degeneracy>> {
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?;
            if let SurfaceDescriptor::Offset(offset) = descriptor {
                let basis = offset.basis();
                self.surface_degeneracies_inner(basis)
            } else {
                Ok(surface_leaf(descriptor).degeneracies())
            }
        })();
        self.leave(geometry);
        result
    }

    fn surface_bounds_inner(
        &mut self,
        surface: SurfaceHandle,
        range: [ParamRange; 2],
    ) -> EvalResult<Aabb3> {
        let geometry = GeometryRef::Surface(surface);
        self.enter(geometry)?;
        let result = (|| {
            let descriptor = self
                .graph
                .surface(surface)
                .ok_or(EvalError::StaleGeometryHandle { geometry })?;
            if let SurfaceDescriptor::Offset(offset) = descriptor {
                let basis = offset.basis();
                let distance = offset.signed_distance();
                Ok(self
                    .surface_bounds_inner(basis, range)?
                    .inflated(distance.abs()))
            } else {
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
            }
        })();
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
        SurfaceDescriptor::Offset(_) => unreachable!("offsets are evaluated recursively"),
    }
}

fn offset_derivatives(basis: SurfaceDerivs, distance: f64) -> EvalResult<SurfaceDerivs> {
    let w = basis.du.cross(basis.dv);
    let q = w.norm();
    if q == 0.0 || !q.is_finite() {
        return Err(EvalError::NonFiniteResult {
            class: SurfaceClass::Offset.key(),
        });
    }
    let n = w / q;
    let w_u = basis.duu.cross(basis.dv) + basis.du.cross(basis.duv);
    let w_v = basis.duv.cross(basis.dv) + basis.du.cross(basis.dvv);
    let n_u = (w_u - n * n.dot(w_u)) / q;
    let n_v = (w_v - n * n.dot(w_v)) / q;
    let value = SurfaceDerivs {
        p: basis.p + n * distance,
        du: basis.du + n_u * distance,
        dv: basis.dv + n_v * distance,
        ..SurfaceDerivs::default()
    };
    if surface_derivs_finite(value, 1) {
        Ok(value)
    } else {
        Err(EvalError::NonFiniteResult {
            class: SurfaceClass::Offset.key(),
        })
    }
}

fn classify_jacobian(value: SurfaceDerivs, tolerances: Tolerances) -> SurfaceValidity {
    let cross = value.du.cross(value.dv).norm();
    let scale = value.du.norm() * value.dv.norm();
    if cross == 0.0 || scale == 0.0 || !cross.is_finite() || !scale.is_finite() {
        return SurfaceValidity::Singular;
    }
    let normalized = cross / scale;
    if !normalized.is_finite() || normalized <= tolerances.angular() {
        SurfaceValidity::Indeterminate {
            reason: ValidityGap::IllConditioned,
        }
    } else {
        SurfaceValidity::Regular {
            normalized_jacobian: normalized,
        }
    }
}

fn enforce_regular(
    value: SurfaceDerivs,
    tolerances: Tolerances,
    surface: SurfaceHandle,
    uv: [f64; 2],
) -> EvalResult<()> {
    match classify_jacobian(value, tolerances) {
        SurfaceValidity::Regular { .. } => Ok(()),
        SurfaceValidity::Singular => Err(EvalError::SingularSurface { surface, uv }),
        SurfaceValidity::Indeterminate { .. } => {
            Err(EvalError::IllConditionedSurface { surface, uv })
        }
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
