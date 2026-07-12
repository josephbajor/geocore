//! Geometry graph storage and deterministic dependency traversal.

use crate::descriptor::{
    Curve2dDescriptor, CurveDescriptor, GeometryDependencies, SurfaceDescriptor,
};
use crate::error::{GeometryGraphError, GeometryGraphResult};
use kcore::arena::{Arena, ArenaChange, Handle};
use kcore::error::Result as CoreResult;
use std::collections::{HashMap, HashSet};

/// Immutable 3D curve node. The descriptor is the node payload itself so
/// topology's historical geometry-enum names can remain source compatible.
pub type CurveNode = CurveDescriptor;

/// Immutable surface node.
pub type SurfaceNode = SurfaceDescriptor;

/// Immutable parameter-space curve node.
pub type Curve2dNode = Curve2dDescriptor;

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

#[derive(Debug, Clone)]
struct ReverseDependencyEntry {
    geometry: GeometryRef,
    dependents: Vec<GeometryRef>,
}

// Observable dependent order lives only in `dependents`. Hash storage is used
// strictly for identity lookup/membership and is never iterated to produce a
// result, so randomized hash seeds cannot affect kernel determinism.
#[derive(Debug, Clone, Default)]
struct ReverseDependencyIndex {
    entries: Vec<Option<ReverseDependencyEntry>>,
    free_entries: Vec<usize>,
    lookup: HashMap<GeometryRef, usize>,
    membership: HashSet<(GeometryRef, GeometryRef)>,
}

/// Read-only construction counters for the isolated benchmark package.
///
/// This is not a stable kernel API. It is available only with the
/// `benchmark-internals` feature and deliberately exposes no index storage.
#[cfg(feature = "benchmark-internals")]
#[doc(hidden)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct GraphBuildObservation {
    registered_nodes: usize,
    registered_dependency_edges: usize,
    full_order_rebuilds: usize,
}

#[cfg(feature = "benchmark-internals")]
impl GraphBuildObservation {
    /// Nodes registered in the reverse-dependency index.
    pub const fn registered_nodes(self) -> usize {
        self.registered_nodes
    }

    /// Dependency edges registered in the reverse-dependency index.
    pub const fn registered_dependency_edges(self) -> usize {
        self.registered_dependency_edges
    }

    /// Complete geometry orders rebuilt for deterministic dependent sorting.
    pub const fn full_order_rebuilds(self) -> usize {
        self.full_order_rebuilds
    }

    /// Difference between two cumulative observations from the same graph.
    pub const fn since(self, earlier: Self) -> Self {
        Self {
            registered_nodes: self.registered_nodes - earlier.registered_nodes,
            registered_dependency_edges: self.registered_dependency_edges
                - earlier.registered_dependency_edges,
            full_order_rebuilds: self.full_order_rebuilds - earlier.full_order_rebuilds,
        }
    }
}

impl ReverseDependencyIndex {
    fn register(&mut self, geometry: GeometryRef) {
        assert!(
            !self.lookup.contains_key(&geometry),
            "geometry registered exactly once"
        );
        let entry = ReverseDependencyEntry {
            geometry,
            dependents: Vec::new(),
        };
        let index = if let Some(index) = self.free_entries.pop() {
            assert!(
                self.entries[index].is_none(),
                "free reverse-index slot is empty"
            );
            self.entries[index] = Some(entry);
            index
        } else {
            let index = self.entries.len();
            self.entries.push(Some(entry));
            index
        };
        self.lookup.insert(geometry, index);
    }

    fn unregister(&mut self, geometry: GeometryRef) {
        if let Some(index) = self.lookup.remove(&geometry) {
            let removed = self.entries[index].take();
            debug_assert_eq!(removed.as_ref().map(|entry| entry.geometry), Some(geometry));
            debug_assert!(
                removed
                    .as_ref()
                    .is_none_or(|entry| entry.dependents.is_empty()),
                "a geometry key is unregistered only after its dependents are gone"
            );
            debug_assert!(
                !self.membership.iter().any(|(dependency, dependent)| {
                    *dependency == geometry || *dependent == geometry
                }),
                "all reverse edges are detached before key removal"
            );
            self.free_entries.push(index);
        }
    }

    fn add(&mut self, dependency: GeometryRef, dependent: GeometryRef) {
        if self.entry(dependency).is_some()
            && self.membership.insert((dependency, dependent))
            && let Some(entry) = self.entry_mut(dependency)
        {
            entry.dependents.push(dependent);
        }
    }

    fn remove_dependent(&mut self, dependency: GeometryRef, dependent: GeometryRef) {
        if self.membership.remove(&(dependency, dependent))
            && let Some(entry) = self.entry_mut(dependency)
        {
            entry.dependents.retain(|candidate| *candidate != dependent);
        }
    }

    fn dependents(&self, geometry: GeometryRef) -> &[GeometryRef] {
        self.entry(geometry)
            .map_or(&[], |entry| entry.dependents.as_slice())
    }

    fn key_count(&self, geometry: GeometryRef) -> usize {
        usize::from(self.entry(geometry).is_some())
    }

    fn entry(&self, geometry: GeometryRef) -> Option<&ReverseDependencyEntry> {
        let index = *self.lookup.get(&geometry)?;
        self.entries
            .get(index)?
            .as_ref()
            .filter(|entry| entry.geometry == geometry)
    }

    fn entry_mut(&mut self, geometry: GeometryRef) -> Option<&mut ReverseDependencyEntry> {
        let index = *self.lookup.get(&geometry)?;
        self.entries
            .get_mut(index)?
            .as_mut()
            .filter(|entry| entry.geometry == geometry)
    }

    fn iter(&self) -> impl Iterator<Item = (GeometryRef, &[GeometryRef])> {
        self.entries.iter().filter_map(|entry| {
            entry
                .as_ref()
                .map(|entry| (entry.geometry, entry.dependents.as_slice()))
        })
    }

    fn structure_mismatch(&self) -> Option<GeometryRef> {
        let mut seen = HashSet::with_capacity(self.lookup.len());
        let mut seen_free = HashSet::with_capacity(self.free_entries.len());
        for &index in &self.free_entries {
            if index >= self.entries.len()
                || self.entries[index].is_some()
                || !seen_free.insert(index)
            {
                return self.iter().next().map(|(geometry, _)| geometry);
            }
        }
        let mut dependency_edges = 0usize;
        for (index, entry) in self.entries.iter().enumerate() {
            let Some(entry) = entry else {
                continue;
            };
            if self.lookup.get(&entry.geometry) != Some(&index)
                || !seen.insert(entry.geometry)
                || entry
                    .dependents
                    .iter()
                    .any(|dependent| !self.membership.contains(&(entry.geometry, *dependent)))
            {
                return Some(entry.geometry);
            }
            dependency_edges += entry.dependents.len();
        }
        (seen.len() != self.lookup.len()
            || dependency_edges != self.membership.len()
            || seen.len() + seen_free.len() != self.entries.len())
        .then(|| self.iter().next().map(|(geometry, _)| geometry))
        .flatten()
    }
}

/// Three typed immutable-node arenas and their dependency index.
#[derive(Default)]
pub struct GeometryGraph {
    curves: Arena<CurveNode>,
    surfaces: Arena<SurfaceNode>,
    curves_2d: Arena<Curve2dNode>,
    reverse_dependencies: ReverseDependencyIndex,
    undo_reverse_dependencies: Vec<ReverseDependencyIndex>,
    #[cfg(feature = "benchmark-internals")]
    benchmark_observation: GraphBuildObservation,
}

impl Clone for GeometryGraph {
    fn clone(&self) -> Self {
        Self {
            curves: self.curves.clone(),
            surfaces: self.surfaces.clone(),
            curves_2d: self.curves_2d.clone(),
            reverse_dependencies: self.reverse_dependencies.clone(),
            undo_reverse_dependencies: Vec::new(),
            #[cfg(feature = "benchmark-internals")]
            benchmark_observation: self.benchmark_observation,
        }
    }
}

/// Deterministic pending or committed graph-arena changes.
pub struct GeometryChanges {
    /// Curve changes in arena-slot order.
    pub curves: Vec<ArenaChange<CurveNode>>,
    /// Surface changes in arena-slot order.
    pub surfaces: Vec<ArenaChange<SurfaceNode>>,
    /// Parameter-space curve changes in arena-slot order.
    pub curves_2d: Vec<ArenaChange<Curve2dNode>>,
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

    /// Snapshot cumulative graph-construction work for benchmark verification.
    #[cfg(feature = "benchmark-internals")]
    #[doc(hidden)]
    pub const fn benchmark_observation(&self) -> GraphBuildObservation {
        self.benchmark_observation
    }

    /// Start an undo frame spanning every geometry arena and dependency index.
    pub fn begin_undo_frame(&mut self) {
        self.curves.begin_undo_frame();
        self.surfaces.begin_undo_frame();
        self.curves_2d.begin_undo_frame();
        self.undo_reverse_dependencies
            .push(self.reverse_dependencies.clone());
    }

    /// Inspect deterministic net changes without consuming the undo frame.
    pub fn pending_undo_frame_changes(&self) -> CoreResult<GeometryChanges> {
        Ok(GeometryChanges {
            curves: self.curves.pending_undo_frame_changes()?,
            surfaces: self.surfaces.pending_undo_frame_changes()?,
            curves_2d: self.curves_2d.pending_undo_frame_changes()?,
        })
    }

    /// Commit every geometry arena and dependency-index change.
    pub fn commit_undo_frame(&mut self) -> CoreResult<GeometryChanges> {
        let changes = GeometryChanges {
            curves: self.curves.commit_undo_frame()?,
            surfaces: self.surfaces.commit_undo_frame()?,
            curves_2d: self.curves_2d.commit_undo_frame()?,
        };
        self.undo_reverse_dependencies
            .pop()
            .expect("geometry commit requires an active undo frame");
        Ok(changes)
    }

    /// Restore exact arena generations, free-list order, and dependency index.
    pub fn rollback_undo_frame(&mut self) -> CoreResult<()> {
        self.curves_2d.rollback_undo_frame()?;
        self.surfaces.rollback_undo_frame()?;
        self.curves.rollback_undo_frame()?;
        self.reverse_dependencies = self
            .undo_reverse_dependencies
            .pop()
            .expect("geometry rollback requires an active undo frame");
        Ok(())
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
        let handle = self.curves.insert(descriptor);
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
        let handle = self.surfaces.insert(descriptor);
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
        let handle = self.curves_2d.insert(descriptor);
        self.register(GeometryRef::Curve2d(handle), &dependencies);
        Ok(handle)
    }

    /// Borrow a live curve node.
    pub fn curve(&self, handle: CurveHandle) -> Option<&CurveNode> {
        self.curves.get(handle)
    }
    /// Atomically replace a curve descriptor while retaining its identity.
    pub fn replace_curve(
        &mut self,
        handle: CurveHandle,
        descriptor: impl Into<CurveDescriptor>,
    ) -> GeometryGraphResult<CurveNode> {
        let descriptor = descriptor.into();
        validate_curve(&descriptor)?;
        self.replace_curve_node(handle, descriptor)
    }
    /// Borrow a live surface node.
    pub fn surface(&self, handle: SurfaceHandle) -> Option<&SurfaceNode> {
        self.surfaces.get(handle)
    }
    /// Atomically replace a surface descriptor while retaining its identity.
    pub fn replace_surface(
        &mut self,
        handle: SurfaceHandle,
        descriptor: impl Into<SurfaceDescriptor>,
    ) -> GeometryGraphResult<SurfaceNode> {
        let descriptor = descriptor.into();
        validate_surface(&descriptor)?;
        self.replace_surface_node(handle, descriptor)
    }
    /// Borrow a live parameter-space curve node.
    pub fn curve2d(&self, handle: Curve2dHandle) -> Option<&Curve2dNode> {
        self.curves_2d.get(handle)
    }
    /// Atomically replace a pcurve descriptor while retaining its identity.
    pub fn replace_curve2d(
        &mut self,
        handle: Curve2dHandle,
        descriptor: impl Into<Curve2dDescriptor>,
    ) -> GeometryGraphResult<Curve2dNode> {
        let descriptor = descriptor.into();
        validate_curve2d(&descriptor)?;
        self.replace_curve2d_node(handle, descriptor)
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
                .visit_dependencies(&mut |r| out.push(r)),
            GeometryRef::Surface(handle) => self
                .surface(handle)
                .ok_or(stale(geometry))?
                .visit_dependencies(&mut |r| out.push(r)),
            GeometryRef::Curve2d(handle) => self
                .curve2d(handle)
                .ok_or(stale(geometry))?
                .visit_dependencies(&mut |r| out.push(r)),
        }
        Ok(out)
    }

    /// Dependency-first transitive closure, including `root`, with duplicates removed.
    pub fn dependency_closure(&self, root: GeometryRef) -> GeometryGraphResult<Vec<GeometryRef>> {
        let mut complete = Vec::new();
        let mut complete_membership = HashSet::new();
        let mut active = Vec::new();
        let mut active_positions = HashMap::new();
        self.visit_dependency_first(
            root,
            &mut active,
            &mut active_positions,
            &mut complete,
            &mut complete_membership,
        )?;
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
        Ok(self.dependency_path(from, target)?.is_some())
    }

    /// First deterministic direct-dependency path from `from` to `target`.
    /// Both endpoints are included when a path exists.
    pub fn dependency_path(
        &self,
        from: GeometryRef,
        target: GeometryRef,
    ) -> GeometryGraphResult<Option<Vec<GeometryRef>>> {
        if !self.contains(from) {
            return Err(stale(from));
        }
        if !self.contains(target) {
            return Err(stale(target));
        }
        let mut active = Vec::new();
        let mut active_positions = HashMap::new();
        let mut complete = HashSet::new();
        if self.find_dependency_path(
            from,
            target,
            &mut active,
            &mut active_positions,
            &mut complete,
        )? {
            Ok(Some(active))
        } else {
            Ok(None)
        }
    }

    /// Remove an unreferenced curve and invalidate its handle.
    pub fn remove_curve(&mut self, handle: CurveHandle) -> GeometryGraphResult<CurveNode> {
        self.remove(GeometryRef::Curve(handle))?;
        self.curves
            .remove(handle)
            .ok_or(stale(GeometryRef::Curve(handle)))
    }

    /// Remove an unreferenced surface and invalidate its handle.
    pub fn remove_surface(&mut self, handle: SurfaceHandle) -> GeometryGraphResult<SurfaceNode> {
        self.remove(GeometryRef::Surface(handle))?;
        self.surfaces
            .remove(handle)
            .ok_or(stale(GeometryRef::Surface(handle)))
    }

    /// Remove an unreferenced 2D curve and invalidate its handle.
    pub fn remove_curve2d(&mut self, handle: Curve2dHandle) -> GeometryGraphResult<Curve2dNode> {
        self.remove(GeometryRef::Curve2d(handle))?;
        self.curves_2d
            .remove(handle)
            .ok_or(stale(GeometryRef::Curve2d(handle)))
    }

    /// Check descriptor invariants, dependency liveness/cycles, and reverse-index agreement.
    pub fn validate(&self) -> GeometryGraphResult<()> {
        if let Some(geometry) = self.reverse_dependencies.structure_mismatch() {
            return Err(GeometryGraphError::ReverseDependencyMismatch { geometry });
        }
        for geometry in self.geometry() {
            if self.reverse_dependencies.key_count(geometry) != 1 {
                return Err(GeometryGraphError::ReverseDependencyMismatch { geometry });
            }
            match geometry {
                GeometryRef::Curve(h) => {
                    validate_curve(self.curve(h).expect("iteration yields live nodes"))?
                }
                GeometryRef::Surface(h) => {
                    validate_surface(self.surface(h).expect("iteration yields live nodes"))?
                }
                GeometryRef::Curve2d(h) => {
                    validate_curve2d(self.curve2d(h).expect("iteration yields live nodes"))?
                }
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
        for (geometry, _) in self.reverse_dependencies.iter() {
            if !self.contains(geometry) {
                return Err(GeometryGraphError::ReverseDependencyMismatch { geometry });
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
        #[cfg(feature = "benchmark-internals")]
        {
            self.benchmark_observation.registered_nodes += 1;
        }
        for dependency in dependencies {
            self.reverse_dependencies.add(*dependency, geometry);
            #[cfg(feature = "benchmark-internals")]
            {
                self.benchmark_observation.registered_dependency_edges += 1;
            }
        }
    }

    fn prepare_replacement(
        &mut self,
        geometry: GeometryRef,
        new_dependencies: &[GeometryRef],
    ) -> GeometryGraphResult<()> {
        if !self.contains(geometry) {
            return Err(stale(geometry));
        }
        self.validate_dependencies(new_dependencies)?;
        for &dependency in new_dependencies {
            if let Some(path) = self.dependency_path(dependency, geometry)? {
                let mut cycle = vec![geometry];
                cycle.extend(path);
                return Err(GeometryGraphError::DependencyCycle { path: cycle });
            }
        }
        let old_dependencies = self.direct_dependencies(geometry)?;
        for &dependency in &old_dependencies {
            if !new_dependencies.contains(&dependency) {
                self.reverse_dependencies
                    .remove_dependent(dependency, geometry);
            }
        }
        for &dependency in new_dependencies {
            if !old_dependencies.contains(&dependency) {
                self.reverse_dependencies.add(dependency, geometry);
            }
        }
        Ok(())
    }

    fn replace_curve_node(
        &mut self,
        handle: CurveHandle,
        descriptor: CurveDescriptor,
    ) -> GeometryGraphResult<CurveNode> {
        let geometry = GeometryRef::Curve(handle);
        let previous = self.curve(handle).ok_or(stale(geometry))?.clone();
        let dependencies = dependencies_of(&descriptor);
        self.prepare_replacement(geometry, &dependencies)?;
        *self
            .curves
            .get_mut(handle)
            .expect("replacement handle is live") = descriptor;
        Ok(previous)
    }

    fn replace_surface_node(
        &mut self,
        handle: SurfaceHandle,
        descriptor: SurfaceDescriptor,
    ) -> GeometryGraphResult<SurfaceNode> {
        let geometry = GeometryRef::Surface(handle);
        let previous = self.surface(handle).ok_or(stale(geometry))?.clone();
        let dependencies = dependencies_of(&descriptor);
        self.prepare_replacement(geometry, &dependencies)?;
        *self
            .surfaces
            .get_mut(handle)
            .expect("replacement handle is live") = descriptor;
        Ok(previous)
    }

    fn replace_curve2d_node(
        &mut self,
        handle: Curve2dHandle,
        descriptor: Curve2dDescriptor,
    ) -> GeometryGraphResult<Curve2dNode> {
        let geometry = GeometryRef::Curve2d(handle);
        let previous = self.curve2d(handle).ok_or(stale(geometry))?.clone();
        let dependencies = dependencies_of(&descriptor);
        self.prepare_replacement(geometry, &dependencies)?;
        *self
            .curves_2d
            .get_mut(handle)
            .expect("replacement handle is live") = descriptor;
        Ok(previous)
    }

    fn validate_dependencies(&self, dependencies: &[GeometryRef]) -> GeometryGraphResult<()> {
        for &dependency in dependencies {
            if !self.contains(dependency) {
                return Err(stale(dependency));
            }
        }
        Ok(())
    }

    fn find_dependency_path(
        &self,
        geometry: GeometryRef,
        target: GeometryRef,
        active: &mut Vec<GeometryRef>,
        active_positions: &mut HashMap<GeometryRef, usize>,
        complete: &mut HashSet<GeometryRef>,
    ) -> GeometryGraphResult<bool> {
        if complete.contains(&geometry) {
            return Ok(false);
        }
        if let Some(&start) = active_positions.get(&geometry) {
            let mut path = active[start..].to_vec();
            path.push(geometry);
            return Err(GeometryGraphError::DependencyCycle { path });
        }
        active_positions.insert(geometry, active.len());
        active.push(geometry);
        if geometry == target {
            return Ok(true);
        }
        for dependency in self.direct_dependencies(geometry)? {
            if self.find_dependency_path(dependency, target, active, active_positions, complete)? {
                return Ok(true);
            }
        }
        let popped = active.pop();
        debug_assert_eq!(popped, Some(geometry));
        let removed = active_positions.remove(&geometry);
        debug_assert!(removed.is_some());
        complete.insert(geometry);
        Ok(false)
    }

    fn visit_dependency_first(
        &self,
        geometry: GeometryRef,
        active: &mut Vec<GeometryRef>,
        active_positions: &mut HashMap<GeometryRef, usize>,
        complete: &mut Vec<GeometryRef>,
        complete_membership: &mut HashSet<GeometryRef>,
    ) -> GeometryGraphResult<()> {
        if complete_membership.contains(&geometry) {
            return Ok(());
        }
        if let Some(&start) = active_positions.get(&geometry) {
            let mut path = active[start..].to_vec();
            path.push(geometry);
            return Err(GeometryGraphError::DependencyCycle { path });
        }
        active_positions.insert(geometry, active.len());
        active.push(geometry);
        for dependency in self.direct_dependencies(geometry)? {
            self.visit_dependency_first(
                dependency,
                active,
                active_positions,
                complete,
                complete_membership,
            )?;
        }
        let popped = active.pop();
        debug_assert_eq!(popped, Some(geometry));
        let removed = active_positions.remove(&geometry);
        debug_assert!(removed.is_some());
        complete_membership.insert(geometry);
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
        for dependency in self.direct_dependencies(geometry)? {
            self.reverse_dependencies
                .remove_dependent(dependency, geometry);
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
        SurfaceDescriptor::Offset(v) => v.signed_distance().is_finite(),
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

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::curve::Line;
    #[cfg(feature = "benchmark-internals")]
    use kgeom::frame::Frame;
    #[cfg(feature = "benchmark-internals")]
    use kgeom::surface::Plane;
    use kgeom::vec::Vec3;

    #[test]
    fn reverse_index_preserves_insertion_order_and_audits_membership() {
        let mut graph = GeometryGraph::new();
        let basis = GeometryRef::Curve(
            graph
                .insert_curve(Line::new(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap())
                .unwrap(),
        );
        let first = GeometryRef::Curve(
            graph
                .insert_curve(
                    Line::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
                )
                .unwrap(),
        );
        let second = GeometryRef::Curve(
            graph
                .insert_curve(
                    Line::new(Vec3::new(0.0, 2.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap(),
                )
                .unwrap(),
        );
        let mut index = ReverseDependencyIndex::default();
        for geometry in [basis, first, second] {
            index.register(geometry);
        }
        index.add(basis, first);
        index.add(basis, second);
        index.add(basis, first);
        assert_eq!(index.dependents(basis), &[first, second]);
        assert_eq!(index.structure_mismatch(), None);

        index.remove_dependent(basis, second);
        index.unregister(second);
        let entry_slots = index.entries.len();
        index.register(second);
        assert_eq!(index.entries.len(), entry_slots);
        assert_eq!(index.structure_mismatch(), None);

        index.membership.remove(&(basis, first));
        assert_eq!(index.structure_mismatch(), Some(basis));
    }

    #[test]
    fn removal_reports_live_graph_dependents_without_mutation() {
        let mut graph = GeometryGraph::new();
        let basis = graph
            .insert_curve(Line::new(Vec3::default(), Vec3::new(1.0, 0.0, 0.0)).unwrap())
            .unwrap();
        let dependent = graph
            .insert_curve(Line::new(Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0)).unwrap())
            .unwrap();
        graph
            .reverse_dependencies
            .add(GeometryRef::Curve(basis), GeometryRef::Curve(dependent));

        assert_eq!(
            graph.remove_curve(basis),
            Err(GeometryGraphError::HasDependents {
                geometry: GeometryRef::Curve(basis),
                dependents: vec![GeometryRef::Curve(dependent)],
            })
        );
        assert!(graph.curve(basis).is_some());
    }

    #[cfg(feature = "benchmark-internals")]
    #[test]
    fn benchmark_observation_counts_attempted_build_work_across_rollback() {
        let mut graph = GeometryGraph::new();
        let basis = graph.insert_surface(Plane::new(Frame::world())).unwrap();
        assert_eq!(
            graph.benchmark_observation(),
            GraphBuildObservation {
                registered_nodes: 1,
                registered_dependency_edges: 0,
                full_order_rebuilds: 0,
            }
        );

        let before = graph.benchmark_observation();
        graph.begin_undo_frame();
        let transient = graph
            .insert_surface(crate::OffsetSurfaceDescriptor::new(basis, 0.5))
            .unwrap();
        graph.rollback_undo_frame().unwrap();

        assert!(graph.surface(transient).is_none());
        assert_eq!(graph.len(), 1);
        assert_eq!(graph.dependents(GeometryRef::Surface(basis)).unwrap(), []);
        assert_eq!(
            graph.benchmark_observation().since(before),
            GraphBuildObservation {
                registered_nodes: 1,
                registered_dependency_edges: 1,
                full_order_rebuilds: 0,
            },
            "read-only counters report attempted work and are not graph state"
        );
        graph.validate().unwrap();
    }
}
