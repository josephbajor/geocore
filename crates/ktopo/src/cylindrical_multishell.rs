//! Assembly of a convex planar outer shell with one closed cylindrical cavity.
//!
//! This module names a boundary-representation class, not a Boolean operation
//! or primitive fixture. The positive planar host and negative finite-cylinder
//! shell are prepared independently, then the complete cylinder is proven
//! strictly inside every convex-host support before any body scaffold is
//! allocated. The resulting reduced region partition owns both boundary
//! shells from one solid region and retains one empty finite-void region.

use crate::cylindrical_band::{CylindricalBandSolidInput, CylindricalBandWinding, PreparedBand};
use crate::entity::{BodyId, FaceId, Region, RegionKind, Shell, ShellId};
use crate::planar::{PlanarSolidInput, PreparedSolid};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::vec::{Point3, Vec3};

/// Semantic input for one convex planar solid containing a cylindrical cavity.
#[derive(Debug, Clone, PartialEq)]
pub struct CylindricalCavitySolidInput {
    host: PlanarSolidInput,
    cavity: CylindricalBandSolidInput,
}

impl CylindricalCavitySolidInput {
    /// Pair one positive convex host with one negative closed cylinder shell.
    pub const fn new(host: PlanarSolidInput, cavity: CylindricalBandSolidInput) -> Self {
        Self { host, cavity }
    }

    /// Positive planar outer-shell proposal.
    pub const fn host(&self) -> &PlanarSolidInput {
        &self.host
    }

    /// Finite cylinder geometry and face lineage for the cavity shell.
    pub const fn cavity(&self) -> &CylindricalBandSolidInput {
        &self.cavity
    }
}

/// Stable handles produced by cylindrical-cavity assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CylindricalCavitySolidOutput {
    body: BodyId,
    outer_shell: ShellId,
    cavity_shell: ShellId,
    host_faces: Vec<FaceId>,
    side_face: FaceId,
    cap_faces: [FaceId; 2],
    ring_edges: [crate::entity::EdgeId; 2],
}

impl CylindricalCavitySolidOutput {
    /// Newly assembled multi-shell solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Positive planar outer shell.
    pub const fn outer_shell(&self) -> ShellId {
        self.outer_shell
    }

    /// Negative closed-cylinder cavity shell.
    pub const fn cavity_shell(&self) -> ShellId {
        self.cavity_shell
    }

    /// Planar host faces in input order.
    pub fn host_faces(&self) -> &[FaceId] {
        &self.host_faces
    }

    /// Reversed cylindrical side face.
    pub const fn side_face(&self) -> FaceId {
        self.side_face
    }

    /// Reversed planar cap faces in `[low, high]` axial order.
    pub const fn cap_faces(&self) -> [FaceId; 2] {
        self.cap_faces
    }

    /// Vertexless ring edges in `[low, high]` axial order.
    pub const fn ring_edges(&self) -> [crate::entity::EdgeId; 2] {
        self.ring_edges
    }
}

#[derive(Debug)]
struct PreparedCylindricalCavity {
    host: PreparedSolid,
    cavity: PreparedBand,
}

impl PreparedCylindricalCavity {
    fn new(input: &CylindricalCavitySolidInput, store: &crate::store::Store) -> Result<Self> {
        let host = PreparedSolid::new(&input.host, store)?;
        let cavity =
            PreparedBand::new_with_winding(&input.cavity, CylindricalBandWinding::Negative, store)?;
        certify_convex_host(&input.host, &host, store)?;
        certify_complete_cavity_containment(input, &host, store)?;
        Ok(Self { host, cavity })
    }
}

impl Transaction<'_> {
    /// Assemble one positive convex planar shell and one negative cylinder shell.
    ///
    /// Host geometry, cylinder geometry, lineage liveness, convexity, and
    /// complete strict containment are preflighted before topology allocation.
    /// The caller owns the eventual checked or Full commit.
    pub fn assemble_cylindrical_cavity_solid(
        &mut self,
        input: &CylindricalCavitySolidInput,
    ) -> Result<CylindricalCavitySolidOutput> {
        let prepared = PreparedCylindricalCavity::new(input, self.store())?;
        self.allocate_prepared_cylindrical_cavity_solid(prepared)
    }

    fn allocate_prepared_cylindrical_cavity_solid(
        &mut self,
        prepared: PreparedCylindricalCavity,
    ) -> Result<CylindricalCavitySolidOutput> {
        let (body, outer_shell) = crate::make::solid_body_scaffold(self.store_mut());
        let solid_region = self.store().get(outer_shell)?.region();
        let cavity_shell = self.store_mut().add(Shell {
            region: solid_region,
            faces: Vec::new(),
            edges: Vec::new(),
            vertex: None,
        });
        self.store_mut()
            .get_mut(solid_region)?
            .shells
            .push(cavity_shell);
        let finite_void = self.store_mut().add(Region {
            body,
            kind: RegionKind::Void,
            shells: Vec::new(),
        });
        self.store_mut().get_mut(body)?.regions.push(finite_void);

        let host = self.allocate_prepared_planar_shell(prepared.host, outer_shell)?;
        let cavity =
            self.allocate_prepared_cylindrical_band_shell(prepared.cavity, cavity_shell)?;
        Ok(CylindricalCavitySolidOutput {
            body,
            outer_shell,
            cavity_shell,
            host_faces: host.faces,
            side_face: cavity.side_face,
            cap_faces: cavity.cap_faces,
            ring_edges: cavity.ring_edges,
        })
    }
}

fn certify_convex_host(
    input: &PlanarSolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
) -> Result<()> {
    for index in 0..input.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "cylindrical-cavity host face disappeared during preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        let mut strictly_inside = false;
        for vertex in input.vertices() {
            match exact_affine(outward, vertex.position(), plane.frame().origin())? {
                Orientation::Negative => strictly_inside = true,
                Orientation::Zero => {}
                Orientation::Positive => {
                    return invalid("cylindrical-cavity host must be globally convex");
                }
            }
        }
        if !strictly_inside {
            return invalid("cylindrical-cavity host face must support a bounded convex solid");
        }
    }
    Ok(())
}

fn certify_complete_cavity_containment(
    input: &CylindricalCavitySolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
) -> Result<()> {
    let frame = input.cavity.frame();
    let range = input.cavity.axial_range();
    let endpoints = [
        frame.origin() + frame.z() * range.lo,
        frame.origin() + frame.z() * range.hi,
    ];
    for index in 0..input.host.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "cylindrical-cavity host face disappeared during containment preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        for center in endpoints {
            if exact_affine(outward, center, plane.frame().origin())? != Orientation::Negative
                || !certify_circle_strictly_inside_support(
                    outward,
                    plane.frame().origin(),
                    frame.x(),
                    frame.y(),
                    center,
                    input.cavity.radius(),
                )
            {
                return invalid(
                    "complete cylindrical cavity must lie strictly inside every convex-host support",
                );
            }
        }
    }
    Ok(())
}

fn certify_circle_strictly_inside_support(
    outward: Vec3,
    support_origin: Point3,
    radial_x: Vec3,
    radial_y: Vec3,
    center: Point3,
    radius: f64,
) -> bool {
    let signed = interval_dot(outward, center - support_origin);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(radius);
    let radial_x = interval_dot(outward, radial_x) * radius;
    let radial_y = interval_dot(outward, radial_y) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    radial_squared.hi() < signed.square().lo()
}

fn exact_affine(normal: Vec3, point: Point3, origin: Point3) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "cylindrical-cavity exact affine predicate is indeterminate",
        })
}

fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
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
    use crate::entity::{
        Body, Edge, EntityRef, Face, FaceId, Fin, Loop, Region, Sense, Shell, Vertex,
    };
    use crate::geom::SurfaceGeom;
    use crate::planar::{PlanarSolidFace, PlanarSolidVertex, PlanarVertexKey};
    use crate::store::Store;
    use crate::transaction::{FullCommitRequirement, LineageEvent};
    use kcore::operation::{
        AccountingMode, BudgetPlan, LimitSpec, OperationContext, ResourceKind, SessionPolicy,
    };
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::param::ParamRange;

    fn cube(half: f64) -> PlanarSolidInput {
        cube_with_sources(half, None)
    }

    fn cube_with_sources(half: f64, sources: Option<[FaceId; 6]>) -> PlanarSolidInput {
        let points = [
            Point3::new(-half, -half, -half),
            Point3::new(half, -half, -half),
            Point3::new(-half, half, -half),
            Point3::new(half, half, -half),
            Point3::new(-half, -half, half),
            Point3::new(half, -half, half),
            Point3::new(-half, half, half),
            Point3::new(half, half, half),
        ];
        let keys = core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(index as u64));
        let vertices = keys
            .into_iter()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
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
        .enumerate()
        .map(|(index, ring)| {
            let face = PlanarSolidFace::new(ring.map(|vertex| keys[vertex]).to_vec());
            if let Some(sources) = sources {
                face.with_source(EntityRef::Face(sources[index]))
            } else {
                face
            }
        })
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn input(radius: f64, origin: Point3) -> CylindricalCavitySolidInput {
        CylindricalCavitySolidInput::new(
            cube(2.0),
            CylindricalBandSolidInput::new(
                Frame::world().with_origin(origin),
                radius,
                ParamRange::new(0.0, 1.0),
            ),
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

    fn assemble_without_containment(
        transaction: &mut Transaction<'_>,
        input: &CylindricalCavitySolidInput,
    ) -> CylindricalCavitySolidOutput {
        let prepared = PreparedCylindricalCavity {
            host: PreparedSolid::new(input.host(), transaction.store()).unwrap(),
            cavity: PreparedBand::new_with_winding(
                input.cavity(),
                CylindricalBandWinding::Negative,
                transaction.store(),
            )
            .unwrap(),
        };
        transaction
            .allocate_prepared_cylindrical_cavity_solid(prepared)
            .unwrap()
    }

    #[test]
    fn contained_cylinder_cavity_is_full_valid_with_exact_topology() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_cavity_solid(&input(0.75, Point3::new(0.0, 0.0, -0.5)))
            .unwrap();
        assert_eq!(counts(transaction.store()), [1, 3, 2, 9, 10, 28, 14, 8]);
        let faces = transaction.store().faces_of_body(output.body()).unwrap();
        let mut loop_counts = faces
            .iter()
            .map(|face| transaction.store().get(*face).unwrap().loops().len())
            .collect::<Vec<_>>();
        loop_counts.sort_unstable();
        assert_eq!(loop_counts, [1, 1, 1, 1, 1, 1, 1, 1, 2]);
        let (planes, cylinders) = faces.iter().fold((0, 0), |counts, face| {
            match transaction
                .store()
                .get(transaction.store().get(*face).unwrap().surface())
                .unwrap()
            {
                SurfaceGeom::Plane(_) => (counts.0 + 1, counts.1),
                SurfaceGeom::Cylinder(_) => (counts.0, counts.1 + 1),
                _ => counts,
            }
        });
        assert_eq!((planes, cylinders), (8, 1));
        assert_eq!(
            transaction.store().get(output.side_face()).unwrap().sense(),
            Sense::Reversed
        );
        assert!(
            output
                .cap_faces()
                .into_iter()
                .all(|face| { transaction.store().get(face).unwrap().sense() == Sense::Reversed })
        );
        let solid = transaction
            .store()
            .get(output.outer_shell())
            .unwrap()
            .region();
        assert_eq!(
            transaction.store().get(solid).unwrap().shells(),
            [output.outer_shell(), output.cavity_shell()]
        );
        assert_eq!(
            transaction
                .store()
                .get(output.body())
                .unwrap()
                .regions()
                .len(),
            3
        );

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
        assert!(decision.checks().iter().all(|check| {
            check.report().outcome() == CheckOutcome::Valid && check.report().gaps.is_empty()
        }));
        assert_eq!(
            check_body_report(&store, output.body(), CheckLevel::Full)
                .unwrap()
                .outcome(),
            CheckOutcome::Valid
        );
    }

    #[test]
    fn invalid_cavity_geometry_fails_before_topology_allocation() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = counts(transaction.store());
        for proposal in [
            input(2.0, Point3::new(0.0, 0.0, -0.5)),
            input(0.75, Point3::new(1.5, 0.0, -0.5)),
            input(-0.75, Point3::new(0.0, 0.0, -0.5)),
        ] {
            assert!(matches!(
                transaction.assemble_cylindrical_cavity_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(counts(transaction.store()), before);
        }
    }

    #[test]
    fn wrong_cavity_winding_is_full_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_cavity_solid(&input(0.75, Point3::new(0.0, 0.0, -0.5)))
            .unwrap();
        transaction
            .store_mut()
            .get_mut(output.side_face())
            .unwrap()
            .sense = Sense::Forward;
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(!decision.is_committed());
        assert!(
            decision
                .checks()
                .iter()
                .any(|check| check.report().outcome() == CheckOutcome::Invalid)
        );
        assert_eq!(store.count::<Body>(), 0);
    }

    #[test]
    fn full_region_proof_distinguishes_contact_from_outside() {
        let mut contact_store = Store::new();
        let mut contact_transaction = contact_store.transaction().unwrap();
        let contact_input = input(2.0, Point3::new(0.0, 0.0, -0.5));
        let contact = assemble_without_containment(&mut contact_transaction, &contact_input);
        let contact_report = check_body_report(
            contact_transaction.store(),
            contact.body(),
            CheckLevel::Full,
        )
        .unwrap();
        assert_eq!(contact_report.outcome(), CheckOutcome::Indeterminate);
        assert!(contact_report.faults.is_empty());
        assert!(contact_report.gaps.iter().any(|gap| {
            gap.kind == VerificationGapKind::RegionContainment
                && gap.entity
                    == crate::entity::EntityRef::Region(
                        contact_transaction
                            .store()
                            .get(contact.outer_shell())
                            .unwrap()
                            .region(),
                    )
        }));

        let mut outside_store = Store::new();
        let mut outside_transaction = outside_store.transaction().unwrap();
        let outside_input = input(0.5, Point3::new(3.0, 0.0, -0.5));
        let outside = assemble_without_containment(&mut outside_transaction, &outside_input);
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
    }

    #[test]
    fn cavity_assembly_records_exactly_nine_face_lineages() {
        let mut store = Store::new();
        let host_source = crate::make::block(&mut store, &Frame::world(), [4.0; 3]).unwrap();
        let host_sources: [FaceId; 6] = store
            .faces_of_body(host_source)
            .unwrap()
            .try_into()
            .unwrap();
        let cavity_frame = Frame::world().with_origin(Point3::new(0.0, 0.0, -0.5));
        let cavity_source = crate::make::cylinder(&mut store, &cavity_frame, 0.75, 1.0).unwrap();
        let cavity_sources: [FaceId; 3] = store
            .faces_of_body(cavity_source)
            .unwrap()
            .try_into()
            .unwrap();
        let input = CylindricalCavitySolidInput::new(
            cube_with_sources(2.0, Some(host_sources)),
            CylindricalBandSolidInput::new(cavity_frame, 0.75, ParamRange::new(0.0, 1.0))
                .with_side_source(cavity_sources[0])
                .with_cap_sources([Some(cavity_sources[1]), Some(cavity_sources[2])]),
        );
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_cavity_solid(&input)
            .unwrap();
        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed());
        let lineage = decision.journal().unwrap().lineage();
        assert_eq!(lineage.len(), 9);
        let result_faces = store.faces_of_body(output.body()).unwrap();
        let source_faces = host_sources
            .into_iter()
            .chain(cavity_sources)
            .collect::<Vec<_>>();
        for event in lineage {
            let LineageEvent::DerivedFrom {
                derived: EntityRef::Face(derived),
                source: EntityRef::Face(source),
            } = event
            else {
                panic!("cylindrical cavity lineage must remain face-only: {event:?}");
            };
            assert!(result_faces.contains(derived));
            assert!(source_faces.contains(source));
        }
    }

    #[test]
    fn cavity_region_work_accepts_exact_n_and_n_minus_one_rolls_back() {
        use crate::cylindrical_region_proof::CYLINDRICAL_CAVITY_REGION_WORK;

        let budget = |allowed| {
            BudgetPlan::new([LimitSpec::new(
                CYLINDRICAL_CAVITY_REGION_WORK,
                ResourceKind::Work,
                AccountingMode::Cumulative,
                allowed,
            )])
            .unwrap()
        };

        let mut accepted_store = Store::new();
        let accepted_session = SessionPolicy::v1();
        let accepted_context = OperationContext::new(&accepted_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(budget(192));
        let mut accepted = accepted_store.transaction().unwrap();
        let accepted_output = accepted
            .assemble_cylindrical_cavity_solid(&input(0.75, Point3::new(0.0, 0.0, -0.5)))
            .unwrap();
        let accepted = accepted
            .commit_full_with_context(
                &[accepted_output.body()],
                FullCommitRequirement::RequireValid,
                &accepted_context,
            )
            .unwrap();
        assert!(accepted.result().as_ref().unwrap().is_committed());
        let usage = accepted
            .report()
            .usage()
            .iter()
            .find(|usage| usage.stage == CYLINDRICAL_CAVITY_REGION_WORK)
            .copied()
            .unwrap();
        assert_eq!((usage.consumed, usage.allowed), (192, 192));

        let mut denied_store = Store::new();
        let before = counts(&denied_store);
        let denied_session = SessionPolicy::v1();
        let denied_context = OperationContext::new(&denied_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(budget(191));
        let mut denied = denied_store.transaction().unwrap();
        let denied_output = denied
            .assemble_cylindrical_cavity_solid(&input(0.75, Point3::new(0.0, 0.0, -0.5)))
            .unwrap();
        let rolled_back_body = denied_output.body();
        let denied = denied
            .commit_full_with_context(
                &[rolled_back_body],
                FullCommitRequirement::RequireValid,
                &denied_context,
            )
            .unwrap();
        let expected = kcore::operation::LimitSnapshot {
            stage: CYLINDRICAL_CAVITY_REGION_WORK,
            resource: ResourceKind::Work,
            consumed: 192,
            allowed: 191,
        };
        assert_eq!(
            denied.result().as_ref().unwrap_err().limit(),
            Some(expected)
        );
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(counts(&denied_store), before);

        let mut retry = denied_store.transaction().unwrap();
        let retried = retry
            .assemble_cylindrical_cavity_solid(&input(0.75, Point3::new(0.0, 0.0, -0.5)))
            .unwrap();
        assert_eq!(retried.body(), rolled_back_body);
    }
}
