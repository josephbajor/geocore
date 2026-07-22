//! Deterministic realization of a completely prepared analytic shell.

use core::fmt;
use std::collections::BTreeMap;

use super::{
    AnalyticEdgeDeclaration, AnalyticEdgeKey, AnalyticEdgeProof, AnalyticEdgeUseRef,
    AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellCurve, AnalyticShellInput,
    AnalyticShellPcurve, AnalyticShellPlanError, AnalyticShellSurface, AnalyticVertexKey,
    PreparedAnalyticShell, prepare_analytic_shell,
};
use crate::entity::{
    BodyId, Edge, EdgeId, EntityRef, Face, FaceId, Fin, FinPcurve, Loop, ParamMap1d, ShellId,
    Vertex, VertexId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::transaction::Transaction;
use kcore::error::Error;
use kgeom::curve::Line;
use kgeom::param::ParamRange;
use kgraph::{
    CylinderRulingTrace, PairedCylinderCylinderRulingResidualCertificate,
    certify_paired_cylinder_cylinder_ruling_residuals,
};

/// Stable topology handles produced by analytic-shell assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AnalyticShellOutput {
    body: BodyId,
    shell: ShellId,
    vertices: Vec<(AnalyticVertexKey, VertexId)>,
    edges: Vec<(AnalyticEdgeKey, EdgeId)>,
    faces: Vec<(AnalyticFaceKey, FaceId)>,
}

impl AnalyticShellOutput {
    /// Newly allocated solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Newly allocated connected boundary shell.
    pub const fn shell(&self) -> ShellId {
        self.shell
    }

    /// Vertex handles in ascending semantic-key order.
    pub fn vertices(&self) -> &[(AnalyticVertexKey, VertexId)] {
        &self.vertices
    }

    /// Edge handles in ascending semantic-key order.
    pub fn edges(&self) -> &[(AnalyticEdgeKey, EdgeId)] {
        &self.edges
    }

    /// Face handles in ascending semantic-key order.
    pub fn faces(&self) -> &[(AnalyticFaceKey, FaceId)] {
        &self.faces
    }
}

/// Typed failure from either allocation-free plan validation or realization.
#[derive(Debug)]
#[non_exhaustive]
pub enum AnalyticShellAssemblyError {
    /// Caller geometry or combinatorics failed before the first allocation.
    Preflight(AnalyticShellPlanError),
    /// A store insertion failed after successful complete preflight.
    Store(Error),
}

impl fmt::Display for AnalyticShellAssemblyError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Preflight(error) => error.fmt(formatter),
            Self::Store(error) => write!(formatter, "analytic shell allocation failed: {error}"),
        }
    }
}

impl std::error::Error for AnalyticShellAssemblyError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Preflight(error) => Some(error),
            Self::Store(error) => Some(error),
        }
    }
}

impl From<AnalyticShellPlanError> for AnalyticShellAssemblyError {
    fn from(error: AnalyticShellPlanError) -> Self {
        Self::Preflight(error)
    }
}

impl From<Error> for AnalyticShellAssemblyError {
    fn from(error: Error) -> Self {
        Self::Store(error)
    }
}

type UseKey = (AnalyticFaceKey, usize, usize);
type FaceHandles = BTreeMap<AnalyticFaceKey, FaceId>;
type SurfaceHandles = BTreeMap<AnalyticFaceKey, crate::entity::SurfaceId>;

struct AllocatedFaces {
    topology: FaceHandles,
    surfaces: SurfaceHandles,
}

impl Transaction<'_> {
    /// Validate and assemble one connected bounded Plane/Cylinder shell.
    ///
    /// Complete semantic and combinatorial preflight finishes before the body
    /// scaffold or any geometry is allocated. Realization then consumes only
    /// the canonical immutable plan. The caller retains this transaction and
    /// decides whether to Full-check, commit, or roll back the candidate.
    pub fn assemble_analytic_shell(
        &mut self,
        input: &AnalyticShellInput,
        tolerance: f64,
    ) -> Result<AnalyticShellOutput, AnalyticShellAssemblyError> {
        let prepared = prepare_analytic_shell(input, self.store(), tolerance)?;
        self.allocate_prepared_analytic_shell(&prepared)
    }

    /// Validate and assemble independent connected analytic-shell components.
    ///
    /// Every input completes semantic and combinatorial preflight before the
    /// first component allocates topology. Components are then realized in
    /// caller order, and outputs retain that same deterministic order. Stable
    /// vertex, edge, and face keys are component-local; distinct inputs may
    /// therefore reuse the same key values without joining their topology.
    ///
    /// The caller retains this transaction and must pass all returned body
    /// handles to one checked commit, or roll back the complete batch. A store
    /// failure during realization remains transaction-atomic through that
    /// existing commit/rollback contract.
    pub fn assemble_analytic_shell_batch(
        &mut self,
        inputs: &[AnalyticShellInput],
        tolerance: f64,
    ) -> Result<Vec<AnalyticShellOutput>, AnalyticShellAssemblyError> {
        let prepared = inputs
            .iter()
            .map(|input| prepare_analytic_shell(input, self.store(), tolerance))
            .collect::<Result<Vec<_>, _>>()?;
        prepared
            .iter()
            .map(|component| self.allocate_prepared_analytic_shell(component))
            .collect()
    }

    fn allocate_prepared_analytic_shell(
        &mut self,
        prepared: &PreparedAnalyticShell,
    ) -> Result<AnalyticShellOutput, AnalyticShellAssemblyError> {
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());
        let vertices = self.allocate_vertices(prepared)?;
        let faces = self.allocate_faces(prepared, shell)?;
        let pcurves = self.allocate_pcurves(prepared)?;
        let edges = self.allocate_edges(prepared, &vertices, &faces.surfaces, &pcurves)?;
        self.allocate_loops_and_fins(prepared, &faces.topology, &edges, &pcurves)?;
        self.record_analytic_lineage(prepared, &faces.topology, &edges);

        Ok(AnalyticShellOutput {
            body,
            shell,
            vertices: vertices.into_iter().collect(),
            edges: edges.into_iter().collect(),
            faces: faces.topology.into_iter().collect(),
        })
    }

    #[cfg(test)]
    pub(super) fn allocate_prepared_analytic_shell_for_test(
        &mut self,
        prepared: &PreparedAnalyticShell,
    ) -> Result<AnalyticShellOutput, AnalyticShellAssemblyError> {
        self.allocate_prepared_analytic_shell(prepared)
    }

    fn allocate_vertices(
        &mut self,
        prepared: &PreparedAnalyticShell,
    ) -> Result<BTreeMap<AnalyticVertexKey, VertexId>, AnalyticShellAssemblyError> {
        let mut handles = BTreeMap::new();
        for vertex in prepared.vertices() {
            let point = self.store_mut().insert_point(vertex.position())?;
            let handle = self.store_mut().add(Vertex {
                point,
                tolerance: None,
            });
            handles.insert(vertex.key(), handle);
        }
        Ok(handles)
    }

    fn allocate_faces(
        &mut self,
        prepared: &PreparedAnalyticShell,
        shell: ShellId,
    ) -> Result<AllocatedFaces, AnalyticShellAssemblyError> {
        let mut faces = BTreeMap::new();
        let mut surfaces = BTreeMap::new();
        for face in prepared.faces() {
            let descriptor = match face.surface() {
                AnalyticShellSurface::Plane(surface) => SurfaceGeom::Plane(surface),
                AnalyticShellSurface::Cylinder(surface) => SurfaceGeom::Cylinder(surface),
            };
            let surface = self.store_mut().insert_surface(descriptor)?;
            let handle = self.store_mut().add(Face {
                shell,
                loops: Vec::new(),
                surface,
                sense: face.sense(),
                domain: Some(face.domain()),
                tolerance: None,
            });
            self.store_mut().get_mut(shell)?.faces.push(handle);
            faces.insert(face.key(), handle);
            surfaces.insert(face.key(), surface);
        }
        Ok(AllocatedFaces {
            topology: faces,
            surfaces,
        })
    }

    fn allocate_pcurves(
        &mut self,
        prepared: &PreparedAnalyticShell,
    ) -> Result<BTreeMap<UseKey, crate::entity::Curve2dId>, AnalyticShellAssemblyError> {
        let mut pcurves = BTreeMap::new();
        for face in prepared.faces() {
            for (loop_index, loop_) in face.loops().iter().enumerate() {
                for (fin_index, fin) in loop_.fins().iter().enumerate() {
                    let descriptor = match fin.pcurve().curve() {
                        AnalyticShellPcurve::Line(curve) => Curve2dGeom::Line(curve),
                        AnalyticShellPcurve::Circle(curve) => Curve2dGeom::Circle(curve),
                    };
                    let handle = self.store_mut().insert_pcurve(descriptor)?;
                    pcurves.insert((face.key(), loop_index, fin_index), handle);
                }
            }
        }
        Ok(pcurves)
    }

    fn allocate_edges(
        &mut self,
        prepared: &PreparedAnalyticShell,
        vertices: &BTreeMap<AnalyticVertexKey, VertexId>,
        surfaces: &BTreeMap<AnalyticFaceKey, crate::entity::SurfaceId>,
        pcurves: &BTreeMap<UseKey, crate::entity::Curve2dId>,
    ) -> Result<BTreeMap<AnalyticEdgeKey, EdgeId>, AnalyticShellAssemblyError> {
        let mut edges = BTreeMap::new();
        for &key in prepared.edge_order() {
            let declaration = prepared.declaration(key).ok_or(Error::InvalidGeometry {
                reason: "prepared analytic-shell edge order lost its declaration",
            })?;
            let (uses, proof) = match declaration {
                AnalyticEdgeDeclaration::Bounded(_) => {
                    let index = prepared
                        .edges()
                        .binary_search_by_key(&key, |candidate| candidate.edge().key())
                        .map_err(|_| Error::InvalidGeometry {
                            reason: "prepared bounded analytic edge lost its certificate",
                        })?;
                    (
                        prepared.edges()[index].uses(),
                        prepared.edges()[index].proof(),
                    )
                }
                AnalyticEdgeDeclaration::Closed(_) => {
                    let index = prepared
                        .closed_edges()
                        .binary_search_by_key(&key, |candidate| candidate.edge().key())
                        .map_err(|_| Error::InvalidGeometry {
                            reason: "prepared closed analytic edge lost its certificate",
                        })?;
                    (
                        prepared.closed_edges()[index].uses(),
                        prepared.closed_edges()[index].proof(),
                    )
                }
            };
            let curve = match proof {
                AnalyticEdgeProof::PlaneLine(certificate) => {
                    let source_surfaces = [surfaces[&uses[0].face()], surfaces[&uses[1].face()]];
                    let source_pcurves = [
                        pcurves[&(uses[0].face(), uses[0].loop_index(), uses[0].fin_index())],
                        pcurves[&(uses[1].face(), uses[1].loop_index(), uses[1].fin_index())],
                    ];
                    self.store_mut().insert_verified_plane_intersection_curve(
                        source_surfaces,
                        source_pcurves,
                        certificate,
                    )?
                }
                AnalyticEdgeProof::CylinderCylinderRuling(certificate) => {
                    let carrier = self.recertify_cylinder_cylinder_ruling(
                        prepared,
                        declaration,
                        uses,
                        surfaces,
                        pcurves,
                        certificate,
                    )?;
                    self.store_mut().insert_curve(CurveGeom::Line(carrier))?
                }
                AnalyticEdgeProof::PlaneCylinderRuling(_)
                | AnalyticEdgeProof::SourceLineagePlaneCylinderRuling(_)
                | AnalyticEdgeProof::PlaneCylinderCircle(_) => {
                    let descriptor = match declaration.carrier() {
                        AnalyticShellCurve::Line(curve) => CurveGeom::Line(curve),
                        AnalyticShellCurve::Circle(curve) => CurveGeom::Circle(curve),
                    };
                    self.store_mut().insert_curve(descriptor)?
                }
            };
            let (topology_vertices, bounds) = match declaration {
                AnalyticEdgeDeclaration::Bounded(edge) => {
                    let endpoints = edge.vertices();
                    (
                        [Some(vertices[&endpoints[0]]), Some(vertices[&endpoints[1]])],
                        Some((edge.range().lo, edge.range().hi)),
                    )
                }
                AnalyticEdgeDeclaration::Closed(_) => ([None, None], None),
            };
            let handle = self.store_mut().add(Edge {
                curve: Some(curve),
                vertices: topology_vertices,
                bounds,
                fins: Vec::new(),
                tolerance: None,
            });
            edges.insert(key, handle);
        }
        Ok(edges)
    }

    fn recertify_cylinder_cylinder_ruling(
        &self,
        prepared: &PreparedAnalyticShell,
        declaration: AnalyticEdgeDeclaration,
        uses: [AnalyticEdgeUseRef; 2],
        surfaces: &BTreeMap<AnalyticFaceKey, crate::entity::SurfaceId>,
        pcurves: &BTreeMap<UseKey, crate::entity::Curve2dId>,
        certificate: PairedCylinderCylinderRulingResidualCertificate,
    ) -> Result<Line, Error> {
        let AnalyticShellCurve::Line(carrier) = declaration.carrier() else {
            return Err(Error::InvalidGeometry {
                reason: "prepared Cylinder/Cylinder ruling lost its line carrier",
            });
        };
        let traces: [Result<CylinderRulingTrace, Error>; 2] = uses.map(|use_| {
            let surface = surfaces
                .get(&use_.face())
                .and_then(|handle| self.store().surface(*handle).ok())
                .and_then(SurfaceGeom::as_cylinder)
                .copied()
                .ok_or(Error::InvalidGeometry {
                    reason: "prepared Cylinder/Cylinder ruling lost a cylinder source",
                })?;
            let key = (use_.face(), use_.loop_index(), use_.fin_index());
            let pcurve = pcurves
                .get(&key)
                .and_then(|handle| self.store().pcurve(*handle).ok())
                .and_then(Curve2dGeom::as_line)
                .copied()
                .ok_or(Error::InvalidGeometry {
                    reason: "prepared Cylinder/Cylinder ruling lost a line trace",
                })?;
            let parameter_map = prepared_pcurve_use(prepared, use_)
                .ok_or(Error::InvalidGeometry {
                    reason: "prepared Cylinder/Cylinder ruling lost its source use",
                })?
                .edge_to_pcurve();
            Ok(CylinderRulingTrace::new(surface, pcurve, parameter_map))
        });
        let [first, second] = traces;
        let recertified = certify_paired_cylinder_cylinder_ruling_residuals(
            carrier,
            declaration.logical_range(),
            [first?, second?],
            certificate.tolerance(),
        )
        .map_err(|_| Error::InvalidGeometry {
            reason: "materialized Cylinder/Cylinder ruling failed exact residual recertification",
        })?;
        if recertified != certificate {
            return Err(Error::InvalidGeometry {
                reason: "materialized Cylinder/Cylinder ruling changed its exact source binding",
            });
        }
        Ok(carrier)
    }

    fn allocate_loops_and_fins(
        &mut self,
        prepared: &PreparedAnalyticShell,
        faces: &BTreeMap<AnalyticFaceKey, FaceId>,
        edges: &BTreeMap<AnalyticEdgeKey, EdgeId>,
        pcurves: &BTreeMap<UseKey, crate::entity::Curve2dId>,
    ) -> Result<(), AnalyticShellAssemblyError> {
        for face in prepared.faces() {
            let face_handle = faces[&face.key()];
            for (loop_index, loop_) in face.loops().iter().enumerate() {
                let loop_handle = self.store_mut().add(Loop {
                    face: face_handle,
                    fins: Vec::new(),
                });
                self.store_mut()
                    .get_mut(face_handle)?
                    .loops
                    .push(loop_handle);
                let mut fin_handles = Vec::with_capacity(loop_.fins().len());
                for (fin_index, fin) in loop_.fins().iter().enumerate() {
                    let edge = prepared
                        .declaration(fin.edge())
                        .ok_or(Error::InvalidGeometry {
                            reason: "prepared analytic-shell fin lost its edge declaration",
                        })?;
                    let map = fin.pcurve().edge_to_pcurve();
                    let range = edge.logical_range();
                    let first = map.map(range.lo);
                    let second = map.map(range.hi);
                    let mut use_ = FinPcurve::new(
                        pcurves[&(face.key(), loop_index, fin_index)],
                        ParamRange::new(first.min(second), first.max(second)),
                        ParamMap1d::affine(map.scale(), map.offset())?,
                    )?
                    .with_chart(fin.pcurve().chart());
                    if let Some(winding) = fin.pcurve().closure_winding() {
                        use_ = use_.with_closure_winding(winding);
                    }
                    let edge_handle = edges[&fin.edge()];
                    let fin_handle = self.store_mut().add(Fin {
                        parent: loop_handle,
                        edge: edge_handle,
                        sense: fin.sense(),
                        pcurve: Some(use_),
                    });
                    self.store_mut().get_mut(edge_handle)?.fins.push(fin_handle);
                    fin_handles.push(fin_handle);
                }
                self.store_mut().get_mut(loop_handle)?.fins = fin_handles;
            }
        }
        Ok(())
    }

    fn record_analytic_lineage(
        &mut self,
        prepared: &PreparedAnalyticShell,
        faces: &BTreeMap<AnalyticFaceKey, FaceId>,
        edges: &BTreeMap<AnalyticEdgeKey, EdgeId>,
    ) {
        for face in prepared.faces() {
            let result = EntityRef::Face(faces[&face.key()]);
            if let Some(source) = face.source() {
                self.record_derived_from(result, source);
            } else if let Some(sources) = face.merge_sources() {
                self.record_merge(sources.to_vec(), result);
            }
        }
        for &key in prepared.edge_order() {
            let Some(declaration) = prepared.declaration(key) else {
                continue;
            };
            let source = match declaration {
                AnalyticEdgeDeclaration::Bounded(edge) => edge.source(),
                AnalyticEdgeDeclaration::Closed(edge) => edge.source(),
            };
            if let Some(source) = source {
                self.record_derived_from(EntityRef::Edge(edges[&key]), source);
            }
        }
    }
}

fn prepared_pcurve_use(
    prepared: &PreparedAnalyticShell,
    use_: AnalyticEdgeUseRef,
) -> Option<AnalyticPcurveUse> {
    let face_index = prepared
        .faces()
        .binary_search_by_key(&use_.face(), |face| face.key())
        .ok()?;
    prepared.faces()[face_index]
        .loops()
        .get(use_.loop_index())?
        .fins()
        .get(use_.fin_index())
        .map(|fin| fin.pcurve())
}
