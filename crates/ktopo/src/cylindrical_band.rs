//! Semantic assembly of one finite cylindrical-band solid.
//!
//! This is the topology realization seam for a proof-selected pair of whole
//! Plane/Cylinder section rings.  The input is an analytic cylinder frame,
//! radius, and finite axial interval rather than a primitive-layout tag.  The
//! assembler preflights all caller-controlled geometry and optional source-face
//! lineage before allocating a body, then creates the canonical vertex-free
//! ring topology admitted by the Full checker:
//!
//! - one cylindrical side face with two single-fin loops,
//! - two planar cap faces with one single-fin loop each, and
//! - two vertex-free periodic circle edges, each used by the side and one cap.
//!
//! The source faces are retained only as semantic lineage.  A Boolean adapter
//! remains responsible for proving that its selected face regions describe the
//! requested band and for supplying the result-oriented frame and interval.

use crate::entity::{
    BodyId, Edge, EdgeId, EntityRef, Face, FaceDomain, FaceId, Fin, FinPcurve, Loop, LoopId,
    ParamMap1d, Sense, ShellId,
};
use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use kcore::tolerance::check_in_size_box;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Vec2, Vec3};

/// Complete semantic input for one positive finite cylindrical-band solid.
///
/// `axial_range` is expressed in the supplied cylinder frame's `v` parameter.
/// Optional cap sources are ordered `[low, high]` in that same parameter.
#[derive(Debug, Clone, PartialEq)]
pub struct CylindricalBandSolidInput {
    frame: Frame,
    radius: f64,
    axial_range: ParamRange,
    side_source: Option<FaceId>,
    cap_sources: [Option<FaceId>; 2],
}

impl CylindricalBandSolidInput {
    /// Describe one finite band on `frame` between `axial_range.lo` and
    /// `axial_range.hi`.
    ///
    /// Validation is deferred to assembly so constructing an input is a pure,
    /// allocation-free operation.
    pub const fn new(frame: Frame, radius: f64, axial_range: ParamRange) -> Self {
        Self {
            frame,
            radius,
            axial_range,
            side_source: None,
            cap_sources: [None, None],
        }
    }

    /// Attach the live source face from which the side region was selected.
    pub const fn with_side_source(mut self, source: FaceId) -> Self {
        self.side_source = Some(source);
        self
    }

    /// Attach live source faces for the low and high cap regions.
    pub const fn with_cap_sources(mut self, sources: [Option<FaceId>; 2]) -> Self {
        self.cap_sources = sources;
        self
    }

    /// Cylinder placement and parameter frame.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Positive cylinder radius requested by the semantic adapter.
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Finite increasing cylinder-height interval.
    pub const fn axial_range(&self) -> ParamRange {
        self.axial_range
    }

    /// Optional selected source of the cylindrical side region.
    pub const fn side_source(&self) -> Option<FaceId> {
        self.side_source
    }

    /// Optional selected sources of the `[low, high]` planar cap regions.
    pub const fn cap_sources(&self) -> [Option<FaceId>; 2] {
        self.cap_sources
    }
}

/// Stable handles produced by cylindrical-band assembly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CylindricalBandSolidOutput {
    body: BodyId,
    shell: ShellId,
    side_face: FaceId,
    cap_faces: [FaceId; 2],
    ring_edges: [EdgeId; 2],
}

impl CylindricalBandSolidOutput {
    /// Newly assembled solid body.
    pub const fn body(self) -> BodyId {
        self.body
    }

    /// Positive connected boundary shell.
    pub const fn shell(self) -> ShellId {
        self.shell
    }

    /// Cylindrical side face.
    pub const fn side_face(self) -> FaceId {
        self.side_face
    }

    /// Planar cap faces in `[low, high]` axial order.
    pub const fn cap_faces(self) -> [FaceId; 2] {
        self.cap_faces
    }

    /// Vertex-free circle edges in `[low, high]` axial order.
    pub const fn ring_edges(self) -> [EdgeId; 2] {
        self.ring_edges
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CylindricalBandWinding {
    Positive,
    Negative,
}

#[derive(Debug, Clone, Copy)]
struct PreparedCap {
    circle: Circle,
    plane: Plane,
    domain: FaceDomain,
    side_sense: Sense,
    side_pcurve: Line2d,
    cap_pcurve: Circle2d,
    cap_parameter_map: ParamMap1d,
    face_sense: Sense,
    source: Option<FaceId>,
}

#[derive(Debug)]
pub(crate) struct PreparedBand {
    pub(crate) cylinder: Cylinder,
    side_domain: FaceDomain,
    side_sense: Sense,
    side_source: Option<FaceId>,
    caps: [PreparedCap; 2],
}

impl PreparedBand {
    pub(crate) fn new_with_winding(
        input: &CylindricalBandSolidInput,
        winding: CylindricalBandWinding,
        store: &crate::store::Store,
    ) -> Result<Self> {
        if !input.radius.is_finite() || input.radius <= 0.0 {
            return invalid("cylindrical-band radius must be finite and positive");
        }
        if !input.axial_range.is_finite() || input.axial_range.lo >= input.axial_range.hi {
            return invalid("cylindrical-band axial range must be finite and increasing");
        }
        for source in core::iter::once(input.side_source).chain(input.cap_sources) {
            if source.is_some_and(|face| !store.contains(face)) {
                return Err(Error::StaleHandle);
            }
        }

        let source_frame = input.frame;
        let radius = input.radius;
        check_in_size_box(source_frame.origin().to_array())?;
        let low_origin = source_frame.origin() + source_frame.z() * input.axial_range.lo;
        let frame = source_frame.with_origin(low_origin);
        let height = input.axial_range.hi - input.axial_range.lo;
        if !height.is_finite() || height <= 0.0 {
            return invalid("cylindrical-band normalized height must be finite and positive");
        }
        let cylinder = Cylinder::new(frame, radius)?;
        let side_domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, height)?;
        let cap_domain = FaceDomain::from_bounds(-radius, radius, -radius, radius)?;

        let mut caps = Vec::with_capacity(2);
        for (index, side_v) in [0.0, height].into_iter().enumerate() {
            let center = frame.origin() + frame.z() * side_v;
            let circle_frame = frame.with_origin(center);
            let circle = Circle::new(circle_frame, radius)?;
            preflight_circle_extent(circle)?;
            let low = index == 0;
            let cap_frame = if low {
                Frame::new(center, -frame.z(), frame.x())?
            } else {
                circle_frame
            };
            let plane = Plane::new(cap_frame);
            let side_pcurve = Line2d::new(Point2::new(0.0, side_v), Vec2::new(1.0, 0.0))?;
            let circle_x = frame_uv_vector(&cap_frame, circle.frame().x());
            let circle_y = frame_uv_vector(&cap_frame, circle.frame().y());
            let cap_pcurve = Circle2d::new(Point2::new(0.0, 0.0), radius, circle_x)?;
            let cap_parameter_map = if circle_y.dot(cap_pcurve.x_dir().perp()) >= 0.0 {
                ParamMap1d::identity()
            } else {
                ParamMap1d::affine(-1.0, core::f64::consts::TAU)?
            };
            let conventional_side_sense = if low { Sense::Forward } else { Sense::Reversed };
            caps.push(PreparedCap {
                circle,
                plane,
                domain: cap_domain,
                side_sense: if winding == CylindricalBandWinding::Positive {
                    conventional_side_sense
                } else {
                    conventional_side_sense.flipped()
                },
                side_pcurve,
                cap_pcurve,
                cap_parameter_map,
                face_sense: if winding == CylindricalBandWinding::Positive {
                    Sense::Forward
                } else {
                    Sense::Reversed
                },
                source: input.cap_sources[index],
            });
        }
        let caps: [PreparedCap; 2] = caps
            .try_into()
            .expect("the two finite axial endpoints prepare exactly two caps");
        Ok(Self {
            cylinder,
            side_domain,
            side_sense: if winding == CylindricalBandWinding::Positive {
                Sense::Forward
            } else {
                Sense::Reversed
            },
            side_source: input.side_source,
            caps,
        })
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct AllocatedCylindricalBandShell {
    pub(crate) side_face: FaceId,
    pub(crate) cap_faces: [FaceId; 2],
    pub(crate) ring_edges: [EdgeId; 2],
}

impl Transaction<'_> {
    /// Assemble one positive finite cylindrical-band solid.
    ///
    /// All input geometry, size-box constraints, face domains, and lineage
    /// liveness are validated before the body scaffold is allocated.  The
    /// caller owns the eventual checked or Full commit.
    pub fn assemble_cylindrical_band_solid(
        &mut self,
        input: &CylindricalBandSolidInput,
    ) -> Result<CylindricalBandSolidOutput> {
        let prepared =
            PreparedBand::new_with_winding(input, CylindricalBandWinding::Positive, self.store())?;
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());
        let allocated = self.allocate_prepared_cylindrical_band_shell(prepared, shell)?;

        Ok(CylindricalBandSolidOutput {
            body,
            shell,
            side_face: allocated.side_face,
            cap_faces: allocated.cap_faces,
            ring_edges: allocated.ring_edges,
        })
    }

    pub(crate) fn allocate_prepared_cylindrical_band_shell(
        &mut self,
        prepared: PreparedBand,
        shell: ShellId,
    ) -> Result<AllocatedCylindricalBandShell> {
        let mut lineage = Vec::with_capacity(3);

        let side_surface = self
            .store_mut()
            .insert_surface(SurfaceGeom::Cylinder(prepared.cylinder))?;
        let side_face = self.store_mut().add(Face {
            shell,
            loops: Vec::new(),
            surface: side_surface,
            sense: prepared.side_sense,
            domain: Some(prepared.side_domain),
            tolerance: None,
        });
        self.store_mut().get_mut(shell)?.faces.push(side_face);
        if let Some(source) = prepared.side_source {
            lineage.push((EntityRef::Face(side_face), EntityRef::Face(source)));
        }

        let mut cap_faces = Vec::with_capacity(2);
        let mut ring_edges = Vec::with_capacity(2);
        for cap in prepared.caps {
            let (cap_face, edge) = self.allocate_cylindrical_band_ring(shell, side_face, cap)?;
            if let Some(source) = cap.source {
                lineage.push((EntityRef::Face(cap_face), EntityRef::Face(source)));
            }
            cap_faces.push(cap_face);
            ring_edges.push(edge);
        }

        for (derived, source) in lineage {
            self.record_derived_from(derived, source);
        }

        Ok(AllocatedCylindricalBandShell {
            side_face,
            cap_faces: cap_faces
                .try_into()
                .expect("two prepared caps allocate exactly two faces"),
            ring_edges: ring_edges
                .try_into()
                .expect("two prepared caps allocate exactly two edges"),
        })
    }

    fn allocate_cylindrical_band_ring(
        &mut self,
        shell: ShellId,
        side_face: FaceId,
        cap: PreparedCap,
    ) -> Result<(FaceId, EdgeId)> {
        let store = self.store_mut();
        let curve = store.insert_curve(CurveGeom::Circle(cap.circle))?;
        let edge = store.add(Edge {
            curve: Some(curve),
            vertices: [None, None],
            bounds: None,
            fins: Vec::new(),
            tolerance: None,
        });

        let range = ParamRange::new(0.0, core::f64::consts::TAU);
        let side_curve = store.insert_pcurve(Curve2dGeom::Line(cap.side_pcurve))?;
        let side_pcurve =
            FinPcurve::new(side_curve, range, ParamMap1d::identity())?.with_closure_winding([1, 0]);
        let side_loop: LoopId = store.add(Loop {
            face: side_face,
            fins: Vec::new(),
        });
        store.get_mut(side_face)?.loops.push(side_loop);
        let side_fin = store.add(Fin {
            parent: side_loop,
            edge,
            sense: cap.side_sense,
            pcurve: Some(side_pcurve),
        });
        store.get_mut(side_loop)?.fins.push(side_fin);

        let cap_surface = store.insert_surface(SurfaceGeom::Plane(cap.plane))?;
        let cap_face = store.add(Face {
            shell,
            loops: Vec::new(),
            surface: cap_surface,
            sense: cap.face_sense,
            domain: Some(cap.domain),
            tolerance: None,
        });
        store.get_mut(shell)?.faces.push(cap_face);
        let cap_loop = store.add(Loop {
            face: cap_face,
            fins: Vec::new(),
        });
        store.get_mut(cap_face)?.loops.push(cap_loop);

        let cap_curve = store.insert_pcurve(Curve2dGeom::Circle(cap.cap_pcurve))?;
        let cap_pcurve =
            FinPcurve::new(cap_curve, range, cap.cap_parameter_map)?.with_closure_winding([0, 0]);
        let cap_fin = store.add(Fin {
            parent: cap_loop,
            edge,
            sense: cap.side_sense.flipped(),
            pcurve: Some(cap_pcurve),
        });
        store.get_mut(cap_loop)?.fins.push(cap_fin);
        store.get_mut(edge)?.fins = vec![side_fin, cap_fin];
        Ok((cap_face, edge))
    }
}

fn preflight_circle_extent(circle: Circle) -> Result<()> {
    let bounds = circle.bounding_box(circle.param_range());
    check_in_size_box(bounds.min.to_array())?;
    check_in_size_box(bounds.max.to_array())?;
    Ok(())
}

fn frame_uv_vector(frame: &Frame, vector: Vec3) -> Vec2 {
    Vec2::new(vector.dot(frame.x()), vector.dot(frame.y()))
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::{CheckLevel, CheckOutcome, check_body_report};
    use crate::entity::{Body, Face as RawFace, Region, Shell};
    use crate::make;
    use crate::store::Store;
    use crate::transaction::{FullCommitRequirement, LineageEvent};
    use kgeom::vec::{Point3, Vec3};

    fn exact_rotated_frame() -> Frame {
        Frame::new(
            Point3::new(3.0, -2.0, 5.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        )
        .unwrap()
    }

    fn source_faces(store: &Store, body: BodyId) -> [FaceId; 3] {
        let solid = store
            .get(body)
            .unwrap()
            .regions()
            .iter()
            .copied()
            .find(|region| store.get(*region).unwrap().kind() == crate::entity::RegionKind::Solid)
            .unwrap();
        let shell = store.get(solid).unwrap().shells()[0];
        store.get(shell).unwrap().faces().try_into().unwrap()
    }

    fn assemble_with_lineage() -> (
        Store,
        CylindricalBandSolidOutput,
        crate::transaction::Journal,
        [FaceId; 3],
    ) {
        let mut store = Store::new();
        let frame = exact_rotated_frame();
        let source = make::cylinder(&mut store, &frame, 1.5, 4.0).unwrap();
        let sources = source_faces(&store, source);
        let input = CylindricalBandSolidInput::new(frame, 1.5, ParamRange::new(0.0, 4.0))
            .with_side_source(sources[0])
            .with_cap_sources([Some(sources[1]), Some(sources[2])]);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_cylindrical_band_solid(&input).unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        let journal = decision.journal().unwrap().clone();
        (store, output, journal, sources)
    }

    #[test]
    fn arbitrary_finite_axial_interval_is_full_valid() {
        let mut store = Store::new();
        let frame = exact_rotated_frame();
        let input = CylindricalBandSolidInput::new(frame, 1.25, ParamRange::new(-1.5, 2.75));
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_cylindrical_band_solid(&input).unwrap();
        assert_eq!(
            transaction
                .store()
                .get(output.shell())
                .unwrap()
                .faces()
                .len(),
            3
        );
        assert_eq!(output.cap_faces().len(), 2);
        assert_eq!(output.ring_edges().len(), 2);
        assert!(
            transaction
                .store()
                .vertices_of_body(output.body())
                .unwrap()
                .is_empty()
        );
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        assert!(decision.journal().unwrap().lineage().is_empty());
        assert!(
            decision
                .checks()
                .iter()
                .all(|check| check.report().outcome() == CheckOutcome::Valid)
        );
        assert_eq!(
            check_body_report(&store, output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        );
    }

    #[test]
    fn non_axis_aligned_band_is_full_valid() {
        let mut store = Store::new();
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.0, 0.6, 0.8),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let input = CylindricalBandSolidInput::new(frame, 0.75, ParamRange::new(0.5, 1.5));
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_cylindrical_band_solid(&input).unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
    }

    #[test]
    fn face_lineage_is_optional_canonical_and_deterministic() {
        let (first_store, first_output, first_journal, sources) = assemble_with_lineage();
        let (_, _, second_journal, _) = assemble_with_lineage();
        assert_eq!(first_journal, second_journal);

        let all_faces = first_store.faces_of_body(first_output.body()).unwrap();
        assert_eq!(
            all_faces,
            vec![
                first_output.side_face(),
                first_output.cap_faces()[0],
                first_output.cap_faces()[1]
            ]
        );
        assert_eq!(
            first_journal.lineage(),
            &[
                LineageEvent::DerivedFrom {
                    derived: EntityRef::Face(first_output.side_face()),
                    source: EntityRef::Face(sources[0]),
                },
                LineageEvent::DerivedFrom {
                    derived: EntityRef::Face(first_output.cap_faces()[0]),
                    source: EntityRef::Face(sources[1]),
                },
                LineageEvent::DerivedFrom {
                    derived: EntityRef::Face(first_output.cap_faces()[1]),
                    source: EntityRef::Face(sources[2]),
                },
            ]
        );
    }

    #[test]
    fn invalid_input_is_rejected_before_topology_allocation() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = (
            transaction.store().count::<Body>(),
            transaction.store().count::<Region>(),
            transaction.store().count::<Shell>(),
            transaction.store().count::<RawFace>(),
        );
        for input in [
            CylindricalBandSolidInput::new(Frame::world(), -1.0, ParamRange::new(0.0, 2.0)),
            CylindricalBandSolidInput::new(Frame::world(), 1.0, ParamRange::new(2.0, 2.0)),
        ] {
            assert!(matches!(
                transaction.assemble_cylindrical_band_solid(&input),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(
                (
                    transaction.store().count::<Body>(),
                    transaction.store().count::<Region>(),
                    transaction.store().count::<Shell>(),
                    transaction.store().count::<RawFace>(),
                ),
                before
            );
        }
    }
}
