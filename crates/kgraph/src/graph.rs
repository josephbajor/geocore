//! Geometry graph storage and deterministic dependency traversal.

use crate::descriptor::{
    Curve2dDescriptor, CurveDescriptor, GeometryDependencies, SurfaceDescriptor,
};
use crate::error::{GeometryGraphError, GeometryGraphResult};
use kcore::arena::{Arena, Handle};

/// Immutable 3D curve node.
#[derive(Debug, Clone, PartialEq)]
pub struct CurveNode {
    descriptor: CurveDescriptor,
}

impl CurveNode {
    /// Borrow the immutable descriptor.
    pub const fn descriptor(&self) -> &CurveDescriptor {
        &self.descriptor
    }
}

/// Immutable surface node.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceNode {
    descriptor: SurfaceDescriptor,
}

impl SurfaceNode {
    /// Borrow the immutable descriptor.
    pub const fn descriptor(&self) -> &SurfaceDescriptor {
        &self.descriptor
    }
}

/// Immutable parameter-space curve node.
#[derive(Debug, Clone, PartialEq)]
pub struct Curve2dNode {
    descriptor: Curve2dDescriptor,
}

impl Curve2dNode {
    /// Borrow the immutable descriptor.
    pub const fn descriptor(&self) -> &Curve2dDescriptor {
        &self.descriptor
    }
}

/// Typed identity of a 3D curve node.
pub type CurveHandle = Handle<CurveNode>;
/// Typed identity of a surface node.
pub type SurfaceHandle = Handle<SurfaceNode>;
/// Typed identity of a parameter-space curve node.
pub type Curve2dHandle = Handle<Curve2dNode>;

/// A type-erased geometry-graph reference used only for graph relationships.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GeometryRef {
    /// 3D curve.
    Curve(CurveHandle),
    /// Surface.
    Surface(SurfaceHandle),
    /// Parameter-space curve.
    Curve2d(Curve2dHandle),
}

#[derive(Debug, Clone, Default)]
struct ReverseDependencyIndex {
    entries: Vec<(GeometryRef, Vec<GeometryRef>)>,
}

impl ReverseDependencyIndex {
    fn register(&mut self, geometry: GeometryRef) {
        self.entries.push((geometry, Vec::new()));
    }

    fn unregister(&mut self, geometry: GeometryRef) {
        self.entries.retain(|(candidate, _)| *candidate != geometry);
        for (_, dependents) in &mut self.entries {
            dependents.retain(|candidate| *candidate != geometry);
        }
    }

    fn add(&mut self, dependency: GeometryRef, dependent: GeometryRef) {
        if let Some((_, values)) = self.entries.iter_mut().find(|(key, _)| *key == dependency) {
            values.push(dependent);
        }
    }

    fn dependents(&self, geometry: GeometryRef) -> &[GeometryRef] {
        self.entries
            .iter()
            .find(|(key, _)| *key == geometry)
            .map_or(&[], |(_, values)| values.as_slice())
    }

    fn key_count(&self, geometry: GeometryRef) -> usize {
        self.entries
            .iter()
            .filter(|(candidate, _)| *candidate == geometry)
            .count()
    }
}

/// Three typed immutable-node arenas and their dependency index.
#[derive(Clone, Default)]
pub struct GeometryGraph {
    curves: Arena<CurveNode>,
    surfaces: Arena<SurfaceNode>,
    curves_2d: Arena<Curve2dNode>,
    reverse_dependencies: ReverseDependencyIndex,
}

impl GeometryGraph {
    /// Construct an empty graph.
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of live 3D curve nodes.
    pub fn curve_count(&self) -> usize {
        self.curves.len()
    }
    /// Number of live surface nodes.
    pub fn surface_count(&self) -> usize {
        self.surfaces.len()
    }
    /// Number of live parameter-space curve nodes.
    pub fn curve2d_count(&self) -> usize {
        self.curves_2d.len()
    }
    /// Total number of live geometry nodes.
    pub fn len(&self) -> usize {
        self.curve_count() + self.surface_count() + self.curve2d_count()
    }
    /// Whether the graph contains no live nodes.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Insert a validated immutable 3D curve descriptor.
    pub fn insert_curve(
        &mut self,
        descriptor: impl Into<CurveDescriptor>,
    ) -> GeometryGraphResult<CurveHandle> {
        let descriptor = descriptor.into();
        validate_curve(&descriptor)?;
        let dependencies = dependencies_of(&descriptor);
        self.validate_dependencies(&dependencies)?;
        let handle = self.curves.insert(CurveNode { descriptor });
        self.register(GeometryRef::Curve(handle), &dependencies);
        Ok(handle)
    }

    /// Insert a validated immutable surface descriptor.
    pub fn insert_surface(
        &mut self,
        descriptor: impl Into<SurfaceDescriptor>,
    ) -> GeometryGraphResult<SurfaceHandle> {
        let descriptor = descriptor.into();
        validate_surface(&descriptor)?;
        let dependencies = dependencies_of(&descriptor);
        self.validate_dependencies(&dependencies)?;
        let handle = self.surfaces.insert(SurfaceNode { descriptor });
        self.register(GeometryRef::Surface(handle), &dependencies);
        Ok(handle)
    }

    /// Insert a validated immutable parameter-space curve descriptor.
    pub fn insert_curve2d(
        &mut self,
        descriptor: impl Into<Curve2dDescriptor>,
    ) -> GeometryGraphResult<Curve2dHandle> {
        let descriptor = descriptor.into();
        validate_curve2d(&descriptor)?;
        let dependencies = dependencies_of(&descriptor);
        self.validate_dependencies(&dependencies)?;
        let handle = self.curves_2d.insert(Curve2dNode { descriptor });
        self.register(GeometryRef::Curve2d(handle), &dependencies);
        Ok(handle)
    }

    /// Borrow a live curve node.
    pub fn curve(&self, handle: CurveHandle) -> Option<&CurveNode> {
        self.curves.get(handle)
    }
    /// Borrow a live surface node.
    pub fn surface(&self, handle: SurfaceHandle) -> Option<&SurfaceNode> {
        self.surfaces.get(handle)
    }
    /// Borrow a live parameter-space curve node.
    pub fn curve2d(&self, handle: Curve2dHandle) -> Option<&Curve2dNode> {
        self.curves_2d.get(handle)
    }

    /// Iterate curves in deterministic arena-slot order.
    pub fn curves(&self) -> impl Iterator<Item = (CurveHandle, &CurveNode)> {
        self.curves.iter()
    }
    /// Iterate surfaces in deterministic arena-slot order.
    pub fn surfaces(&self) -> impl Iterator<Item = (SurfaceHandle, &SurfaceNode)> {
        self.surfaces.iter()
    }
    /// Iterate 2D curves in deterministic arena-slot order.
    pub fn curves_2d(&self) -> impl Iterator<Item = (Curve2dHandle, &Curve2dNode)> {
        self.curves_2d.iter()
    }

    /// Iterate all nodes in stable curve/surface/2D-curve, then arena-slot, order.
    pub fn geometry(&self) -> impl Iterator<Item = GeometryRef> + '_ {
        self.curves()
            .map(|(h, _)| GeometryRef::Curve(h))
            .chain(self.surfaces().map(|(h, _)| GeometryRef::Surface(h)))
            .chain(self.curves_2d().map(|(h, _)| GeometryRef::Curve2d(h)))
    }

    /// Whether a type-erased reference is live.
    pub fn contains(&self, geometry: GeometryRef) -> bool {
        match geometry {
            GeometryRef::Curve(handle) => self.curves.contains(handle),
            GeometryRef::Surface(handle) => self.surfaces.contains(handle),
            GeometryRef::Curve2d(handle) => self.curves_2d.contains(handle),
        }
    }

    /// Direct dependencies in stable descriptor-field order.
    pub fn direct_dependencies(
        &self,
        geometry: GeometryRef,
    ) -> GeometryGraphResult<Vec<GeometryRef>> {
        let mut out = Vec::new();
        match geometry {
            GeometryRef::Curve(handle) => self
                .curve(handle)
                .ok_or(stale(geometry))?
                .descriptor
                .visit_dependencies(&mut |r| out.push(r)),
            GeometryRef::Surface(handle) => self
                .surface(handle)
                .ok_or(stale(geometry))?
                .descriptor
                .visit_dependencies(&mut |r| out.push(r)),
            GeometryRef::Curve2d(handle) => self
                .curve2d(handle)
                .ok_or(stale(geometry))?
                .descriptor
                .visit_dependencies(&mut |r| out.push(r)),
        }
        Ok(out)
    }

    /// Dependency-first transitive closure, including `root`, with duplicates removed.
    pub fn dependency_closure(&self, root: GeometryRef) -> GeometryGraphResult<Vec<GeometryRef>> {
        let mut complete = Vec::new();
        let mut active = Vec::new();
        self.visit_dependency_first(root, &mut active, &mut complete)?;
        Ok(complete)
    }

    /// Direct graph dependents in deterministic insertion/slot order.
    pub fn dependents(&self, geometry: GeometryRef) -> GeometryGraphResult<Vec<GeometryRef>> {
        if !self.contains(geometry) {
            return Err(stale(geometry));
        }
        Ok(self.reverse_dependencies.dependents(geometry).to_vec())
    }

    /// Whether `from` transitively reaches `target` through dependencies.
    pub fn reaches(&self, from: GeometryRef, target: GeometryRef) -> GeometryGraphResult<bool> {
        Ok(self
            .dependency_closure(from)?
            .into_iter()
            .any(|candidate| candidate == target))
    }

    /// Remove an unreferenced curve and invalidate its handle.
    pub fn remove_curve(&mut self, handle: CurveHandle) -> GeometryGraphResult<()> {
        self.remove(GeometryRef::Curve(handle))?;
        let _ = self
            .curves
            .remove(handle)
            .ok_or(stale(GeometryRef::Curve(handle)))?;
        Ok(())
    }

    /// Remove an unreferenced surface and invalidate its handle.
    pub fn remove_surface(&mut self, handle: SurfaceHandle) -> GeometryGraphResult<()> {
        self.remove(GeometryRef::Surface(handle))?;
        let _ = self
            .surfaces
            .remove(handle)
            .ok_or(stale(GeometryRef::Surface(handle)))?;
        Ok(())
    }

    /// Remove an unreferenced 2D curve and invalidate its handle.
    pub fn remove_curve2d(&mut self, handle: Curve2dHandle) -> GeometryGraphResult<()> {
        self.remove(GeometryRef::Curve2d(handle))?;
        let _ = self
            .curves_2d
            .remove(handle)
            .ok_or(stale(GeometryRef::Curve2d(handle)))?;
        Ok(())
    }

    /// Check descriptor invariants, dependency liveness/cycles, and reverse-index agreement.
    pub fn validate(&self) -> GeometryGraphResult<()> {
        for geometry in self.geometry() {
            if self.reverse_dependencies.key_count(geometry) != 1 {
                return Err(GeometryGraphError::ReverseDependencyMismatch { geometry });
            }
            match geometry {
                GeometryRef::Curve(h) => validate_curve(
                    &self
                        .curve(h)
                        .expect("iteration yields live nodes")
                        .descriptor,
                )?,
                GeometryRef::Surface(h) => validate_surface(
                    &self
                        .surface(h)
                        .expect("iteration yields live nodes")
                        .descriptor,
                )?,
                GeometryRef::Curve2d(h) => validate_curve2d(
                    &self
                        .curve2d(h)
                        .expect("iteration yields live nodes")
                        .descriptor,
                )?,
            }
            let dependencies = self.direct_dependencies(geometry)?;
            self.validate_dependencies(&dependencies)?;
            let _ = self.dependency_closure(geometry)?;
            for dependency in dependencies {
                if !self
                    .reverse_dependencies
                    .dependents(dependency)
                    .contains(&geometry)
                {
                    return Err(GeometryGraphError::ReverseDependencyMismatch {
                        geometry: dependency,
                    });
                }
            }
        }
        for (geometry, _) in &self.reverse_dependencies.entries {
            if !self.contains(*geometry) {
                return Err(GeometryGraphError::ReverseDependencyMismatch {
                    geometry: *geometry,
                });
            }
        }
        for geometry in self.geometry() {
            for dependent in self.reverse_dependencies.dependents(geometry) {
                if !self.direct_dependencies(*dependent)?.contains(&geometry) {
                    return Err(GeometryGraphError::ReverseDependencyMismatch { geometry });
                }
            }
        }
        Ok(())
    }

    fn register(&mut self, geometry: GeometryRef, dependencies: &[GeometryRef]) {
        self.reverse_dependencies.register(geometry);
        for dependency in dependencies {
            self.reverse_dependencies.add(*dependency, geometry);
        }
        let order: Vec<_> = self.geometry().collect();
        for (_, dependents) in &mut self.reverse_dependencies.entries {
            dependents.sort_by_key(|dependent| {
                order
                    .iter()
                    .position(|candidate| candidate == dependent)
                    .unwrap_or(usize::MAX)
            });
        }
    }

    fn validate_dependencies(&self, dependencies: &[GeometryRef]) -> GeometryGraphResult<()> {
        for &dependency in dependencies {
            if !self.contains(dependency) {
                return Err(stale(dependency));
            }
        }
        Ok(())
    }

    fn visit_dependency_first(
        &self,
        geometry: GeometryRef,
        active: &mut Vec<GeometryRef>,
        complete: &mut Vec<GeometryRef>,
    ) -> GeometryGraphResult<()> {
        if complete.contains(&geometry) {
            return Ok(());
        }
        if let Some(start) = active.iter().position(|candidate| *candidate == geometry) {
            let mut path = active[start..].to_vec();
            path.push(geometry);
            return Err(GeometryGraphError::DependencyCycle { path });
        }
        active.push(geometry);
        for dependency in self.direct_dependencies(geometry)? {
            self.visit_dependency_first(dependency, active, complete)?;
        }
        let popped = active.pop();
        debug_assert_eq!(popped, Some(geometry));
        complete.push(geometry);
        Ok(())
    }

    fn remove(&mut self, geometry: GeometryRef) -> GeometryGraphResult<()> {
        if !self.contains(geometry) {
            return Err(stale(geometry));
        }
        let dependents = self.dependents(geometry)?;
        if !dependents.is_empty() {
            return Err(GeometryGraphError::HasDependents {
                geometry,
                dependents,
            });
        }
        self.reverse_dependencies.unregister(geometry);
        Ok(())
    }
}

fn stale(geometry: GeometryRef) -> GeometryGraphError {
    GeometryGraphError::StaleGeometryHandle { geometry }
}

fn dependencies_of(descriptor: &impl GeometryDependencies) -> Vec<GeometryRef> {
    let mut out = Vec::new();
    descriptor.visit_dependencies(&mut |geometry| out.push(geometry));
    out
}

fn finite2(v: kgeom::vec::Vec2) -> bool {
    v.x.is_finite() && v.y.is_finite()
}
fn finite3(v: kgeom::vec::Vec3) -> bool {
    v.x.is_finite() && v.y.is_finite() && v.z.is_finite()
}

fn finite_frame(frame: &kgeom::frame::Frame) -> bool {
    finite3(frame.origin()) && finite3(frame.x()) && finite3(frame.y()) && finite3(frame.z())
}

fn validate_curve(descriptor: &CurveDescriptor) -> GeometryGraphResult<()> {
    let valid = match descriptor {
        CurveDescriptor::Line(v) => finite3(v.origin()) && finite3(v.dir()),
        CurveDescriptor::Circle(v) => finite_frame(v.frame()) && v.radius().is_finite(),
        CurveDescriptor::Ellipse(v) => {
            finite_frame(v.frame()) && v.major_radius().is_finite() && v.minor_radius().is_finite()
        }
        CurveDescriptor::Nurbs(v) => {
            v.points().iter().copied().all(finite3)
                && v.knots().as_slice().iter().all(|x| x.is_finite())
                && v.weights().is_none_or(|w| w.iter().all(|x| x.is_finite()))
        }
    };
    if valid {
        Ok(())
    } else {
        Err(GeometryGraphError::InvalidDescriptor {
            class: descriptor.class_key(),
            reason: "descriptor contains a non-finite value",
        })
    }
}

fn validate_surface(descriptor: &SurfaceDescriptor) -> GeometryGraphResult<()> {
    let valid = match descriptor {
        SurfaceDescriptor::Plane(v) => finite_frame(v.frame()),
        SurfaceDescriptor::Cylinder(v) => finite_frame(v.frame()) && v.radius().is_finite(),
        SurfaceDescriptor::Cone(v) => {
            finite_frame(v.frame()) && v.radius().is_finite() && v.half_angle().is_finite()
        }
        SurfaceDescriptor::Sphere(v) => finite_frame(v.frame()) && v.radius().is_finite(),
        SurfaceDescriptor::Torus(v) => {
            finite_frame(v.frame()) && v.major_radius().is_finite() && v.minor_radius().is_finite()
        }
        SurfaceDescriptor::Nurbs(v) => {
            v.points().iter().copied().all(finite3)
                && v.weights().is_none_or(|w| w.iter().all(|x| x.is_finite()))
        }
    };
    if valid {
        Ok(())
    } else {
        Err(GeometryGraphError::InvalidDescriptor {
            class: descriptor.class_key(),
            reason: "descriptor contains a non-finite value",
        })
    }
}

fn validate_curve2d(descriptor: &Curve2dDescriptor) -> GeometryGraphResult<()> {
    let valid = match descriptor {
        Curve2dDescriptor::Line(v) => finite2(v.origin()) && finite2(v.dir()),
        Curve2dDescriptor::Circle(v) => {
            finite2(v.center()) && finite2(v.x_dir()) && v.radius().is_finite()
        }
        Curve2dDescriptor::Nurbs(v) => {
            v.points().iter().copied().all(finite2)
                && v.weights().is_none_or(|w| w.iter().all(|x| x.is_finite()))
        }
    };
    if valid {
        Ok(())
    } else {
        Err(GeometryGraphError::InvalidDescriptor {
            class: descriptor.class_key(),
            reason: "descriptor contains a non-finite value",
        })
    }
}
