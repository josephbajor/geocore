//! Count-independent assembly of oriented convex whole-shell components.
//!
//! Inputs name analytic boundary representations, not Boolean operations or
//! primitive fixtures. The outer component receives positive winding and each
//! direct cavity receives negative winding. The currently admitted mixed
//! relation is a complete finite cylinder containing one or more globally
//! convex planar shells; every cavity is strictly contained and every pair is
//! support-plane separated before topology allocation.

use crate::convex_containment::{
    ConvexPlanarInputProof, certify_convex_planar_input, certify_planar_inputs_separated,
    certify_planar_inside_cylinder,
};
use crate::cylindrical_band::{CylindricalBandSolidInput, CylindricalBandWinding, PreparedBand};
use crate::entity::{BodyId, EdgeId, FaceId, Region, RegionKind, Shell, ShellId, VertexId};
use crate::planar::{
    PlanarEdgeKey, PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidInput, PlanarVertexKey,
    PreparedShellWinding, PreparedSolid,
};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};

/// A canonical positive whole-shell geometry whose role assigns final winding.
#[derive(Debug, Clone, PartialEq)]
pub enum OrientedWholeShellInput {
    /// Globally convex planar boundary.
    Planar(PlanarSolidInput),
    /// Complete finite-cylinder boundary.
    Cylindrical(CylindricalBandSolidInput),
}

/// One positive outer component and direct negative cavity components.
#[derive(Debug, Clone, PartialEq)]
pub struct MixedConvexMultiShellSolidInput {
    outer: OrientedWholeShellInput,
    cavities: Vec<OrientedWholeShellInput>,
}

impl MixedConvexMultiShellSolidInput {
    /// Construct a pure semantic proposal; assembly performs full preflight.
    pub fn new(outer: OrientedWholeShellInput, cavities: Vec<OrientedWholeShellInput>) -> Self {
        Self { outer, cavities }
    }

    /// Positive outer whole-shell proposal.
    pub const fn outer(&self) -> &OrientedWholeShellInput {
        &self.outer
    }

    /// Direct negative whole-shell cavity proposals.
    pub fn cavities(&self) -> &[OrientedWholeShellInput] {
        &self.cavities
    }
}

/// Stable handles for one allocated whole-shell representation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AllocatedWholeShell {
    /// Key-preserving planar topology.
    Planar {
        /// Owning shell.
        shell: ShellId,
        /// Vertex handles in semantic-key order.
        vertices: Vec<(PlanarVertexKey, VertexId)>,
        /// Edge handles in semantic-key order.
        edges: Vec<(PlanarEdgeKey, EdgeId)>,
        /// Face handles in input order.
        faces: Vec<FaceId>,
    },
    /// Complete finite-cylinder topology.
    Cylindrical {
        /// Owning shell.
        shell: ShellId,
        /// Whole cylindrical side face.
        side_face: FaceId,
        /// Cap faces in `[low, high]` axial order.
        cap_faces: [FaceId; 2],
        /// Vertexless rings in `[low, high]` axial order.
        ring_edges: [EdgeId; 2],
    },
}

impl AllocatedWholeShell {
    /// Owning shell for either admitted representation.
    pub const fn shell(&self) -> ShellId {
        match self {
            Self::Planar { shell, .. } | Self::Cylindrical { shell, .. } => *shell,
        }
    }
}

/// Stable handles produced by mixed convex multi-shell assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MixedConvexMultiShellSolidOutput {
    body: BodyId,
    outer: AllocatedWholeShell,
    cavities: Vec<AllocatedWholeShell>,
}

impl MixedConvexMultiShellSolidOutput {
    /// Newly assembled body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Positive outer shell handle.
    pub const fn outer_shell(&self) -> ShellId {
        self.outer.shell()
    }

    /// Negative direct cavity shell handles in input order.
    pub fn cavity_shells(&self) -> Vec<ShellId> {
        self.cavities
            .iter()
            .map(AllocatedWholeShell::shell)
            .collect()
    }

    /// Positive outer representation handles.
    pub const fn outer(&self) -> &AllocatedWholeShell {
        &self.outer
    }

    /// Negative cavity representation handles in input order.
    pub fn cavities(&self) -> &[AllocatedWholeShell] {
        &self.cavities
    }
}

/// Checked semantic preflight work for the admitted mixed representation.
///
/// For each planar cavity this charges `F*V + 4V`: the complete convexity
/// support matrix, one vertex collection, and three finite-cylinder decisions
/// per vertex. Each unordered cavity pair adds both directed support/vertex
/// matrices, `F_i*V_j + F_j*V_i`. The one six-face/eight-vertex cavity used by
/// the public rung therefore returns exactly 80.
pub fn mixed_convex_multishell_preflight_work(
    input: &MixedConvexMultiShellSolidInput,
) -> Result<u64> {
    if input.cavities.is_empty() {
        return invalid("mixed convex multi-shell assembly requires at least one cavity");
    }
    if !matches!(input.outer, OrientedWholeShellInput::Cylindrical(_))
        || input
            .cavities
            .iter()
            .any(|cavity| !matches!(cavity, OrientedWholeShellInput::Planar(_)))
    {
        return invalid(
            "admitted mixed convex relation requires a cylinder outer and planar cavities",
        );
    }
    let mut dimensions = Vec::with_capacity(input.cavities.len());
    for cavity in &input.cavities {
        let OrientedWholeShellInput::Planar(planar) = cavity else {
            return invalid("mixed convex cavity representation is not admitted");
        };
        dimensions.push((planar.faces().len(), planar.vertices().len()));
    }
    mixed_convex_multishell_dimension_work(&dimensions)
}

/// Checked semantic work from already-admitted planar cavity dimensions.
///
/// This is the allocation-free admission seam for callers that have certified
/// source dimensions before materializing semantic topology inputs. Dimensions
/// are `(face_count, vertex_count)` in cavity order and use the same formula as
/// [`mixed_convex_multishell_preflight_work`].
pub fn mixed_convex_multishell_dimension_work(dimensions: &[(usize, usize)]) -> Result<u64> {
    if dimensions.is_empty() {
        return invalid("mixed convex multi-shell work requires at least one cavity");
    }
    let mut work = 0_u64;
    for &(faces, vertices) in dimensions {
        let faces = as_u64(faces)?;
        let vertices = as_u64(vertices)?;
        work = work
            .checked_add(faces.checked_mul(vertices).ok_or_else(work_overflow)?)
            .and_then(|value| value.checked_add(vertices.checked_mul(4)?))
            .ok_or_else(work_overflow)?;
    }
    for first in 0..dimensions.len() {
        for second in first + 1..dimensions.len() {
            let (first_faces, first_vertices) = dimensions[first];
            let first_faces = as_u64(first_faces)?;
            let first_vertices = as_u64(first_vertices)?;
            let (second_faces, second_vertices) = dimensions[second];
            let second_faces = as_u64(second_faces)?;
            let second_vertices = as_u64(second_vertices)?;
            let pair = first_faces
                .checked_mul(second_vertices)
                .and_then(|value| value.checked_add(second_faces.checked_mul(first_vertices)?))
                .ok_or_else(work_overflow)?;
            work = work.checked_add(pair).ok_or_else(work_overflow)?;
        }
    }
    Ok(work)
}

/// Topology-mutation-free certificate for the admitted mixed whole-shell relation.
///
/// This performs the same geometry, convexity, strict-containment,
/// pair-separation, and source-lineage liveness preflight used by assembly,
/// but creates no topology and does not mutate `store`. Semantic preparation
/// may allocate bounded temporary vectors.
pub fn certify_mixed_convex_multishell_input(
    input: &MixedConvexMultiShellSolidInput,
    store: &crate::store::Store,
) -> Result<()> {
    PreparedMixedConvexMultiShell::new(input, store).map(drop)
}

#[derive(Debug)]
enum PreparedWholeShell {
    Planar(PreparedSolid),
    Cylindrical(Box<PreparedBand>),
}

#[derive(Debug)]
struct PreparedMixedConvexMultiShell {
    outer: PreparedWholeShell,
    cavities: Vec<PreparedWholeShell>,
}

impl PreparedMixedConvexMultiShell {
    fn new(input: &MixedConvexMultiShellSolidInput, store: &crate::store::Store) -> Result<Self> {
        let _work = mixed_convex_multishell_preflight_work(input)?;
        let OrientedWholeShellInput::Cylindrical(outer_input) = &input.outer else {
            return invalid("mixed convex outer representation is not admitted");
        };
        let outer =
            PreparedBand::new_with_winding(outer_input, CylindricalBandWinding::Positive, store)?;
        let mut cavities = Vec::with_capacity(input.cavities.len());
        let mut proofs = Vec::<ConvexPlanarInputProof>::with_capacity(input.cavities.len());
        for cavity in &input.cavities {
            let OrientedWholeShellInput::Planar(planar) = cavity else {
                return invalid("mixed convex cavity representation is not admitted");
            };
            let positive = PreparedSolid::new(planar, store)?;
            let proof = certify_convex_planar_input(planar, &positive, store)?;
            certify_planar_inside_cylinder(&proof, outer_input)?;
            let reversed = reversed_planar_input(planar);
            let negative =
                PreparedSolid::new_with_winding(&reversed, store, PreparedShellWinding::Negative)?;
            proofs.push(proof);
            cavities.push(PreparedWholeShell::Planar(negative));
        }
        for first in 0..proofs.len() {
            for second in first + 1..proofs.len() {
                certify_planar_inputs_separated(&proofs[first], &proofs[second])?;
            }
        }
        Ok(Self {
            outer: PreparedWholeShell::Cylindrical(Box::new(outer)),
            cavities,
        })
    }
}

impl Transaction<'_> {
    /// Assemble positive and negative whole shells after complete preflight.
    ///
    /// No topology is allocated until every representation, lineage handle,
    /// convexity relation, strict containment, and cavity-pair separation has
    /// certified. The caller owns the eventual Full checked commit.
    pub fn assemble_mixed_convex_multishell_solid(
        &mut self,
        input: &MixedConvexMultiShellSolidInput,
    ) -> Result<MixedConvexMultiShellSolidOutput> {
        let prepared = PreparedMixedConvexMultiShell::new(input, self.store())?;
        self.allocate_prepared_mixed_convex_multishell(prepared)
    }

    fn allocate_prepared_mixed_convex_multishell(
        &mut self,
        prepared: PreparedMixedConvexMultiShell,
    ) -> Result<MixedConvexMultiShellSolidOutput> {
        let (body, outer_shell) = crate::make::solid_body_scaffold(self.store_mut());
        let solid_region = self.store().get(outer_shell)?.region();
        let mut cavity_shells = Vec::with_capacity(prepared.cavities.len());
        for _ in &prepared.cavities {
            let shell = self.store_mut().add(Shell {
                region: solid_region,
                faces: Vec::new(),
                edges: Vec::new(),
                vertex: None,
            });
            self.store_mut().get_mut(solid_region)?.shells.push(shell);
            let void = self.store_mut().add(Region {
                body,
                kind: RegionKind::Void,
                shells: Vec::new(),
            });
            self.store_mut().get_mut(body)?.regions.push(void);
            cavity_shells.push(shell);
        }

        let outer = self.allocate_prepared_whole_shell(prepared.outer, outer_shell)?;
        let cavities = prepared
            .cavities
            .into_iter()
            .zip(cavity_shells)
            .map(|(cavity, shell)| self.allocate_prepared_whole_shell(cavity, shell))
            .collect::<Result<Vec<_>>>()?;
        Ok(MixedConvexMultiShellSolidOutput {
            body,
            outer,
            cavities,
        })
    }

    fn allocate_prepared_whole_shell(
        &mut self,
        prepared: PreparedWholeShell,
        shell: ShellId,
    ) -> Result<AllocatedWholeShell> {
        match prepared {
            PreparedWholeShell::Planar(prepared) => {
                let allocated = self.allocate_prepared_planar_shell(prepared, shell)?;
                Ok(AllocatedWholeShell::Planar {
                    shell: allocated.shell,
                    vertices: allocated.vertices,
                    edges: allocated.edges,
                    faces: allocated.faces,
                })
            }
            PreparedWholeShell::Cylindrical(prepared) => {
                let allocated = self.allocate_prepared_cylindrical_band_shell(*prepared, shell)?;
                Ok(AllocatedWholeShell::Cylindrical {
                    shell,
                    side_face: allocated.side_face,
                    cap_faces: allocated.cap_faces,
                    ring_edges: allocated.ring_edges,
                })
            }
        }
    }
}

fn reversed_planar_input(input: &PlanarSolidInput) -> PlanarSolidInput {
    let faces = input
        .faces()
        .iter()
        .map(|face| {
            let mut vertices = face.vertices().to_vec();
            vertices.reverse();
            let mut reversed = PlanarSolidFace::new(vertices);
            if let Some(source) = face.source() {
                reversed = reversed.with_source(source);
            }
            if let Some(binding) = face.plane_binding() {
                let mut carriers = binding.edge_carriers().to_vec();
                carriers.reverse();
                carriers.rotate_left(1);
                reversed = reversed
                    .with_plane_binding(PlanarFacePlaneBinding::new(binding.support(), carriers));
            }
            reversed
        })
        .collect();
    PlanarSolidInput::new(input.vertices().to_vec(), faces)
}

fn as_u64(value: usize) -> Result<u64> {
    u64::try_from(value).map_err(|_| work_overflow())
}

fn work_overflow() -> Error {
    Error::InvalidGeometry {
        reason: "mixed convex multi-shell preflight work count overflow",
    }
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::{
        CheckLevel, CheckOutcome, FaultKind, VerificationGapKind, check_body_report,
    };
    use crate::entity::{Body, Edge, Face, Fin, Loop, Region, Sense, Shell, Vertex};
    use crate::geom::{CurveGeom, SurfaceGeom};
    use crate::store::Store;
    use crate::transaction::FullCommitRequirement;
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::param::ParamRange;
    use kgeom::vec::Point3;

    fn cube(center: Point3, half: f64, first_key: u64) -> PlanarSolidInput {
        let offsets = [
            [-half, -half, -half],
            [half, -half, -half],
            [-half, half, -half],
            [half, half, -half],
            [-half, -half, half],
            [half, -half, half],
            [-half, half, half],
            [half, half, half],
        ];
        let keys =
            core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(first_key + index as u64));
        let vertices = offsets
            .into_iter()
            .enumerate()
            .map(|(index, [x, y, z])| {
                crate::planar::PlanarSolidVertex::new(
                    keys[index],
                    Point3::new(center.x + x, center.y + y, center.z + z),
                )
            })
            .collect();
        let faces = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
        .into_iter()
        .map(|ring| PlanarSolidFace::new(ring.map(|index| keys[index]).to_vec()))
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn tetrahedron(center: Point3, half: f64, first_key: u64) -> PlanarSolidInput {
        let offsets = [
            [-half, -half, -half],
            [half, -half, -half],
            [0.0, half, -half],
            [0.0, 0.0, half],
        ];
        let keys =
            core::array::from_fn::<_, 4, _>(|index| PlanarVertexKey::new(first_key + index as u64));
        let vertices = offsets
            .into_iter()
            .enumerate()
            .map(|(index, [x, y, z])| {
                crate::planar::PlanarSolidVertex::new(
                    keys[index],
                    Point3::new(center.x + x, center.y + y, center.z + z),
                )
            })
            .collect();
        let faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]]
            .into_iter()
            .map(|ring| PlanarSolidFace::new(ring.map(|index| keys[index]).to_vec()))
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn cube_in_frame(frame: Frame, half: f64, first_key: u64) -> PlanarSolidInput {
        let local = [
            [-half, -half, -half],
            [half, -half, -half],
            [-half, half, -half],
            [half, half, -half],
            [-half, -half, half],
            [half, -half, half],
            [-half, half, half],
            [half, half, half],
        ];
        let keys =
            core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(first_key + index as u64));
        let vertices = local
            .into_iter()
            .enumerate()
            .map(|(index, [x, y, z])| {
                crate::planar::PlanarSolidVertex::new(keys[index], frame.point_at(x, y, z))
            })
            .collect();
        let faces = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ]
        .into_iter()
        .map(|ring| PlanarSolidFace::new(ring.map(|index| keys[index]).to_vec()))
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn bound_cube_in_frame(
        store: &mut Store,
        frame: Frame,
        half: f64,
        first_key: u64,
    ) -> PlanarSolidInput {
        let source = crate::make::block(store, &frame, [2.0 * half; 3]).unwrap();
        let surfaces = store
            .faces_of_body(source)
            .unwrap()
            .into_iter()
            .map(|face| store.get(face).unwrap().surface())
            .collect::<Vec<_>>();
        let rings = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let input = cube_in_frame(frame, half, first_key);
        let faces = input
            .faces()
            .iter()
            .enumerate()
            .map(|(face_index, face)| {
                let carriers = (0..rings[face_index].len())
                    .map(|edge_index| {
                        let a = rings[face_index][edge_index];
                        let b = rings[face_index][(edge_index + 1) % rings[face_index].len()];
                        let other = rings
                            .iter()
                            .enumerate()
                            .find(|(candidate_index, candidate)| {
                                *candidate_index != face_index
                                    && (0..candidate.len()).any(|index| {
                                        let c = candidate[index];
                                        let d = candidate[(index + 1) % candidate.len()];
                                        a == c && b == d || a == d && b == c
                                    })
                            })
                            .unwrap()
                            .0;
                        surfaces[other]
                    })
                    .collect();
                PlanarSolidFace::new(face.vertices().to_vec())
                    .with_plane_binding(PlanarFacePlaneBinding::new(surfaces[face_index], carriers))
            })
            .collect();
        PlanarSolidInput::new(input.vertices().to_vec(), faces)
    }

    fn bound_tetrahedron(
        store: &mut Store,
        center: Point3,
        half: f64,
        first_key: u64,
    ) -> PlanarSolidInput {
        let input = tetrahedron(center, half, first_key);
        let surfaces = input
            .faces()
            .iter()
            .map(|face| {
                let point = |key| {
                    input
                        .vertices()
                        .iter()
                        .find(|vertex| vertex.key() == key)
                        .unwrap()
                        .position()
                };
                let a = point(face.vertices()[0]);
                let b = point(face.vertices()[1]);
                let c = point(face.vertices()[2]);
                let frame = Frame::new(a, (b - a).cross(c - a), b - a).unwrap();
                store
                    .insert_surface(SurfaceGeom::Plane(kgeom::surface::Plane::new(frame)))
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let faces = input
            .faces()
            .iter()
            .enumerate()
            .map(|(face_index, face)| {
                let carriers = (0..face.vertices().len())
                    .map(|edge_index| {
                        let a = face.vertices()[edge_index];
                        let b = face.vertices()[(edge_index + 1) % face.vertices().len()];
                        let other = input
                            .faces()
                            .iter()
                            .enumerate()
                            .find(|(candidate_index, candidate)| {
                                *candidate_index != face_index
                                    && candidate.vertices().contains(&a)
                                    && candidate.vertices().contains(&b)
                            })
                            .unwrap()
                            .0;
                        surfaces[other]
                    })
                    .collect();
                PlanarSolidFace::new(face.vertices().to_vec())
                    .with_plane_binding(PlanarFacePlaneBinding::new(surfaces[face_index], carriers))
            })
            .collect();
        PlanarSolidInput::new(input.vertices().to_vec(), faces)
    }

    fn cylinder(radius: f64, low: f64, high: f64) -> CylindricalBandSolidInput {
        CylindricalBandSolidInput::new(Frame::world(), radius, ParamRange::new(low, high))
    }

    fn input(cavity: PlanarSolidInput) -> MixedConvexMultiShellSolidInput {
        MixedConvexMultiShellSolidInput::new(
            OrientedWholeShellInput::Cylindrical(cylinder(2.0, -2.0, 2.0)),
            vec![OrientedWholeShellInput::Planar(cavity)],
        )
    }

    fn counts(store: &Store) -> [usize; 8] {
        [
            store.count::<Body>(),
            store.count::<Region>(),
            store.count::<Shell>(),
            store.count::<Face>(),
            store.count::<Loop>(),
            store.count::<Fin>(),
            store.count::<Edge>(),
            store.count::<Vertex>(),
        ]
    }

    fn allocate_without_relation(
        transaction: &mut Transaction<'_>,
        outer: CylindricalBandSolidInput,
        cavity: &PlanarSolidInput,
        negative_cavity: bool,
    ) -> MixedConvexMultiShellSolidOutput {
        let outer = PreparedBand::new_with_winding(
            &outer,
            CylindricalBandWinding::Positive,
            transaction.store(),
        )
        .unwrap();
        let cavity = if negative_cavity {
            PreparedSolid::new_with_winding(
                &reversed_planar_input(cavity),
                transaction.store(),
                PreparedShellWinding::Negative,
            )
            .unwrap()
        } else {
            PreparedSolid::new(cavity, transaction.store()).unwrap()
        };
        transaction
            .allocate_prepared_mixed_convex_multishell(PreparedMixedConvexMultiShell {
                outer: PreparedWholeShell::Cylindrical(Box::new(outer)),
                cavities: vec![PreparedWholeShell::Planar(cavity)],
            })
            .unwrap()
    }

    fn assert_exact_topology(store: &Store, output: &MixedConvexMultiShellSolidOutput) {
        assert_eq!(counts(store), [1, 3, 2, 9, 10, 28, 14, 8]);
        let faces = store.faces_of_body(output.body()).unwrap();
        let mut loop_counts = faces
            .iter()
            .map(|&face| store.get(face).unwrap().loops().len())
            .collect::<Vec<_>>();
        loop_counts.sort_unstable();
        assert_eq!(loop_counts, [1, 1, 1, 1, 1, 1, 1, 1, 2]);
        assert_eq!(
            faces
                .iter()
                .filter(|&&face| matches!(
                    store.get(store.get(face).unwrap().surface()).unwrap(),
                    SurfaceGeom::Plane(_)
                ))
                .count(),
            8
        );
        let edges = store.edges_of_body(output.body()).unwrap();
        let bounded_lines = edges
            .iter()
            .filter(|&&edge| {
                let edge = store.get(edge).unwrap();
                edge.vertices().iter().all(Option::is_some)
                    && edge.bounds().is_some()
                    && matches!(
                        store.get(edge.curve().unwrap()).unwrap(),
                        CurveGeom::Line(_)
                    )
            })
            .count();
        let whole_circles = edges
            .iter()
            .filter(|&&edge| {
                let edge = store.get(edge).unwrap();
                edge.vertices().iter().all(Option::is_none)
                    && edge.bounds().is_none()
                    && matches!(
                        store.get(edge.curve().unwrap()).unwrap(),
                        CurveGeom::Circle(_)
                    )
            })
            .count();
        assert_eq!((bounded_lines, whole_circles), (12, 2));
    }

    #[test]
    fn cylinder_outer_and_planar_cavity_are_full_valid_with_exact_topology() {
        let proposal = input(cube(Point3::new(0.0, 0.0, 0.0), 1.0, 10));
        assert_eq!(
            mixed_convex_multishell_preflight_work(&proposal).unwrap(),
            80
        );
        assert_eq!(
            mixed_convex_multishell_dimension_work(&[(6, 8)]).unwrap(),
            80
        );
        let mut store = Store::new();
        certify_mixed_convex_multishell_input(&proposal, &store).unwrap();
        assert_eq!(counts(&store), [0; 8]);
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        assert_exact_topology(transaction.store(), &output);
        let AllocatedWholeShell::Cylindrical {
            side_face,
            cap_faces,
            ..
        } = output.outer()
        else {
            panic!("expected cylindrical outer handles");
        };
        assert_eq!(
            transaction.store().get(*side_face).unwrap().sense(),
            Sense::Forward
        );
        assert!(
            cap_faces
                .iter()
                .all(|&face| { transaction.store().get(face).unwrap().sense() == Sense::Forward })
        );
        let [
            AllocatedWholeShell::Planar {
                shell,
                vertices,
                edges,
                faces,
            },
        ] = output.cavities()
        else {
            panic!("expected one planar cavity output");
        };
        assert_eq!(
            (*shell, vertices.len(), edges.len(), faces.len()),
            (output.cavity_shells()[0], 8, 12, 6)
        );
        let report =
            check_body_report(transaction.store(), output.body(), CheckLevel::Full).unwrap();
        assert!(report.faults.is_empty(), "report: {report:?}");
        assert!(report.gaps.is_empty(), "report: {report:?}");
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
    }

    #[test]
    fn oblique_rigid_frame_preserves_mixed_containment_and_full_proof() {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            kgeom::vec::Vec3::new(0.0, 0.6, 0.8),
            kgeom::vec::Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut store = Store::new();
        let cavity = bound_cube_in_frame(&mut store, frame, 0.5, 10);
        let proposal = MixedConvexMultiShellSolidInput::new(
            OrientedWholeShellInput::Cylindrical(CylindricalBandSolidInput::new(
                frame.with_origin(frame.point_at(0.0, 0.0, -2.0)),
                2.0,
                ParamRange::new(0.0, 4.0),
            )),
            vec![OrientedWholeShellInput::Planar(cavity)],
        );
        certify_mixed_convex_multishell_input(&proposal, &store).unwrap();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
    }

    #[test]
    fn semantic_contact_outside_and_wrong_winding_fail_before_allocation() {
        let valid_cube = cube(Point3::new(0.0, 0.0, 0.0), 1.0, 10);
        let proposals = [
            MixedConvexMultiShellSolidInput::new(
                OrientedWholeShellInput::Cylindrical(cylinder(2.0, -2.0, 2.0)),
                Vec::new(),
            ),
            MixedConvexMultiShellSolidInput::new(
                OrientedWholeShellInput::Planar(valid_cube.clone()),
                vec![OrientedWholeShellInput::Cylindrical(cylinder(
                    0.5, -0.5, 0.5,
                ))],
            ),
            MixedConvexMultiShellSolidInput::new(
                OrientedWholeShellInput::Cylindrical(cylinder(2.0, -1.0, 1.0)),
                vec![OrientedWholeShellInput::Planar(valid_cube.clone())],
            ),
            input(cube(Point3::new(2.5, 0.0, 0.0), 1.0, 20)),
            input(reversed_planar_input(&valid_cube)),
            MixedConvexMultiShellSolidInput::new(
                OrientedWholeShellInput::Cylindrical(cylinder(-2.0, -2.0, 2.0)),
                vec![OrientedWholeShellInput::Planar(valid_cube)],
            ),
        ];
        let mut store = Store::new();
        let before = counts(&store);
        let mut transaction = store.transaction().unwrap();
        for proposal in proposals {
            assert!(matches!(
                transaction.assemble_mixed_convex_multishell_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(counts(transaction.store()), before);
        }
    }

    #[test]
    fn full_proof_distinguishes_contact_outside_and_wrong_winding() {
        let cavity = cube(Point3::new(0.0, 0.0, 0.0), 1.0, 10);
        let mut contact_store = Store::new();
        let mut contact_transaction = contact_store.transaction().unwrap();
        let contact = allocate_without_relation(
            &mut contact_transaction,
            cylinder(2.0, -1.0, 1.0),
            &cavity,
            true,
        );
        let contact_report = check_body_report(
            contact_transaction.store(),
            contact.body(),
            CheckLevel::Full,
        )
        .unwrap();
        assert_eq!(contact_report.outcome(), CheckOutcome::Indeterminate);
        assert!(contact_report.faults.is_empty());
        assert!(
            contact_report
                .gaps
                .iter()
                .any(|gap| gap.kind == VerificationGapKind::RegionContainment)
        );

        let mut outside_store = Store::new();
        let mut outside_transaction = outside_store.transaction().unwrap();
        let outside_cavity = cube(Point3::new(2.5, 0.0, 0.0), 1.0, 20);
        let outside = allocate_without_relation(
            &mut outside_transaction,
            cylinder(2.0, -2.0, 2.0),
            &outside_cavity,
            true,
        );
        let outside_report = check_body_report(
            outside_transaction.store(),
            outside.body(),
            CheckLevel::Full,
        )
        .unwrap();
        assert_eq!(outside_report.outcome(), CheckOutcome::Invalid);
        assert!(
            outside_report
                .faults
                .iter()
                .any(|fault| fault.kind == FaultKind::RegionShellLayout)
        );

        let mut winding_store = Store::new();
        let mut winding_transaction = winding_store.transaction().unwrap();
        let winding = allocate_without_relation(
            &mut winding_transaction,
            cylinder(2.0, -2.0, 2.0),
            &cavity,
            false,
        );
        let winding_report = check_body_report(
            winding_transaction.store(),
            winding.body(),
            CheckLevel::Full,
        )
        .unwrap();
        assert_eq!(winding_report.outcome(), CheckOutcome::Invalid);
        assert!(
            winding_report
                .faults
                .iter()
                .any(|fault| fault.kind == FaultKind::RegionShellLayout)
        );
    }

    #[test]
    fn mixed_region_work_accepts_108_and_107_rolls_back() {
        use crate::mixed_region_proof::MIXED_CONVEX_REGION_WORK;

        let budget = |allowed| {
            BudgetPlan::new([LimitSpec::new(
                MIXED_CONVEX_REGION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap()
        };
        let proposal = input(cube(Point3::new(0.0, 0.0, 0.0), 1.0, 10));
        let mut accepted_store = Store::new();
        let accepted_session = SessionPolicy::v1();
        let accepted_context = OperationContext::new(&accepted_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(budget(108));
        let mut accepted = accepted_store.transaction().unwrap();
        let output = accepted
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        let accepted = accepted
            .commit_full_with_context(
                &[output.body()],
                FullCommitRequirement::RequireValid,
                &accepted_context,
            )
            .unwrap();
        assert!(accepted.result().as_ref().unwrap().is_committed());
        let usage = accepted
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == MIXED_CONVEX_REGION_WORK)
            .unwrap();
        assert_eq!((usage.consumed, usage.allowed), (108, 108));

        let mut denied_store = Store::new();
        let denied_session = SessionPolicy::v1();
        let denied_context = OperationContext::new(&denied_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(budget(107));
        let mut denied = denied_store.transaction().unwrap();
        let output = denied
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        let rolled_back_body = output.body();
        let denied = denied
            .commit_full_with_context(
                &[rolled_back_body],
                FullCommitRequirement::RequireValid,
                &denied_context,
            )
            .unwrap();
        let expected = kcore::operation::LimitSnapshot {
            stage: MIXED_CONVEX_REGION_WORK,
            resource: ResourceKind::Work,
            consumed: 108,
            allowed: 107,
        };
        assert_eq!(
            denied.result().as_ref().unwrap_err().limit(),
            Some(expected)
        );
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(counts(&denied_store), [0; 8]);
        let mut retry = denied_store.transaction().unwrap();
        let retried = retry
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        assert_eq!(retried.body(), rolled_back_body);
    }

    #[test]
    fn multiple_planar_cavities_are_count_neutral_and_pairwise_separated() {
        let mut store = Store::new();
        let tetrahedron = bound_tetrahedron(&mut store, Point3::new(1.5, 0.0, 0.0), 0.5, 100);
        let proposal = MixedConvexMultiShellSolidInput::new(
            OrientedWholeShellInput::Cylindrical(cylinder(4.0, -2.0, 2.0)),
            vec![
                OrientedWholeShellInput::Planar(cube(Point3::new(-1.5, 0.0, 0.0), 0.5, 10)),
                OrientedWholeShellInput::Planar(tetrahedron),
            ],
        );
        assert_eq!(
            mixed_convex_multishell_preflight_work(&proposal).unwrap(),
            168
        );
        assert_eq!(
            mixed_convex_multishell_dimension_work(&[(6, 8), (4, 4)]).unwrap(),
            168
        );
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_mixed_convex_multishell_solid(&proposal)
            .unwrap();
        assert_eq!(counts(transaction.store()), [1, 4, 3, 13, 14, 40, 20, 12]);
        let session = SessionPolicy::v1();
        let context = OperationContext::new(&session, Tolerances::default()).unwrap();
        let committed = transaction
            .commit_full_with_context(
                &[output.body()],
                FullCommitRequirement::RequireValid,
                &context,
            )
            .unwrap();
        assert!(
            committed.result().as_ref().unwrap().is_committed(),
            "result: {:?}",
            committed.result()
        );
        let mixed_work = committed
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == crate::mixed_region_proof::MIXED_CONVEX_REGION_WORK)
            .unwrap();
        assert_eq!(mixed_work.consumed, 212);
    }
}
