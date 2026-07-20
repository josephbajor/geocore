//! Proof-bearing extraction of finite full-period cylinder sources.
//!
//! The extractor does not trust primitive face order or constructor identity.
//! It first requires a proof-complete Full report, then maps the certified
//! three-face cylinder-band topology back to its analytic carrier and its two
//! ordered cap boundaries. The result is read-only source evidence for the
//! curved Boolean pipeline; it allocates no candidate topology.

use kcore::operation::{AccountingMode, OperationScope, ResourceKind};
use kcore::predicates::{Orientation, affine_dot3};
use kgeom::surface::Cylinder;
use kgeom::vec::Point3;
use ktopo::check::{CheckLevel, CheckOutcome, CheckReport, check_body_report_in_scope};
use ktopo::entity::{BodyKind, EdgeId, FaceId, FinId, LoopId, RegionKind, ShellId};
use ktopo::geom::{CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use super::extract::PLANAR_SOURCE_EXTRACTION_WORK;
use crate::error::{Error, Result};

/// Valid source geometry that is outside the finite cylinder-band class.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CylinderSourceGap {
    BodyLayout,
    FaceLayout,
    TolerantEntity,
    BoundaryIncidence,
    AnalyticGeometry,
}

/// Fail-closed source extraction result.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CylinderSourceOutcome {
    Ready(CertifiedCylinderSource),
    NotFullValid(CheckReport),
    Unsupported(CylinderSourceGap),
}

/// One exact cap boundary, ordered along the cylinder's authored axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CertifiedCylinderBoundary {
    cap_face: FaceId,
    edge: EdgeId,
    center: Point3,
}

impl CertifiedCylinderBoundary {
    pub(crate) const fn cap_face(self) -> FaceId {
        self.cap_face
    }

    pub(crate) const fn edge(self) -> EdgeId {
        self.edge
    }

    pub(crate) const fn center(self) -> Point3 {
        self.center
    }
}

/// Exact analytic and topological evidence for one finite cylinder source.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct CertifiedCylinderSource {
    side_face: FaceId,
    shell: ShellId,
    cylinder: Cylinder,
    boundaries: [CertifiedCylinderBoundary; 2],
}

impl CertifiedCylinderSource {
    pub(crate) const fn side_face(&self) -> FaceId {
        self.side_face
    }

    pub(crate) const fn shell(&self) -> ShellId {
        self.shell
    }

    pub(crate) const fn cylinder(&self) -> Cylinder {
        self.cylinder
    }

    pub(crate) const fn boundaries(&self) -> &[CertifiedCylinderBoundary; 2] {
        &self.boundaries
    }
}

#[derive(Debug, Clone, Copy)]
struct UnorderedBoundary {
    cap_face: FaceId,
    edge: EdgeId,
    center: Point3,
}

/// Extract one proof-complete finite cylinder without relying on storage
/// order. Full checking is the certification authority; the remaining scan
/// only binds the certified topology to stable source handles and geometry.
pub(crate) fn extract_cylinder_source(
    store: &Store,
    body_id: ktopo::entity::BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<CylinderSourceOutcome> {
    scope
        .ledger()
        .require_limit(
            PLANAR_SOURCE_EXTRACTION_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )
        .map_err(Error::from)?;
    charge(scope, 1)?;
    let report =
        check_body_report_in_scope(store, body_id, CheckLevel::Full, scope).map_err(Error::from)?;
    if report.outcome() != CheckOutcome::Valid {
        return Ok(CylinderSourceOutcome::NotFullValid(report));
    }

    let Some((shell_id, faces)) = simple_solid_shell(store, body_id, scope)? else {
        return Ok(CylinderSourceOutcome::Unsupported(
            CylinderSourceGap::BodyLayout,
        ));
    };
    if faces.len() != 3 {
        return Ok(CylinderSourceOutcome::Unsupported(
            CylinderSourceGap::FaceLayout,
        ));
    }

    let mut side = None;
    let mut caps = Vec::with_capacity(2);
    for &face_id in &faces {
        charge(scope, 1)?;
        let face = store
            .get(face_id)
            .map_err(|source| Error::InconsistentTopology { source })?;
        if face.tolerance().is_some() {
            return Ok(CylinderSourceOutcome::Unsupported(
                CylinderSourceGap::TolerantEntity,
            ));
        }
        match store
            .surface(face.surface())
            .map_err(|source| Error::InconsistentTopology { source })?
        {
            SurfaceGeom::Cylinder(cylinder) if side.is_none() => {
                side = Some((face_id, *cylinder));
            }
            SurfaceGeom::Plane(_) => caps.push(face_id),
            _ => {
                return Ok(CylinderSourceOutcome::Unsupported(
                    CylinderSourceGap::FaceLayout,
                ));
            }
        }
    }
    let (Some((side_face, cylinder)), [cap_a, cap_b]) = (side, caps.as_slice()) else {
        return Ok(CylinderSourceOutcome::Unsupported(
            CylinderSourceGap::FaceLayout,
        ));
    };
    let cap_faces = [*cap_a, *cap_b];
    let Some(unordered) = bind_boundaries(store, side_face, &cap_faces, scope)? else {
        return Ok(CylinderSourceOutcome::Unsupported(
            CylinderSourceGap::BoundaryIncidence,
        ));
    };
    let Some(boundaries) = order_boundaries(cylinder, unordered) else {
        return Ok(CylinderSourceOutcome::Unsupported(
            CylinderSourceGap::AnalyticGeometry,
        ));
    };
    Ok(CylinderSourceOutcome::Ready(CertifiedCylinderSource {
        side_face,
        shell: shell_id,
        cylinder,
        boundaries,
    }))
}

fn simple_solid_shell(
    store: &Store,
    body_id: ktopo::entity::BodyId,
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<(ShellId, Vec<FaceId>)>> {
    let body = store
        .get(body_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    charge(scope, usize_work(body.regions().len())?)?;
    if body.kind() != BodyKind::Solid || body.regions().len() != 2 {
        return Ok(None);
    }
    let exterior = store
        .get(body.regions()[0])
        .map_err(|source| Error::InconsistentTopology { source })?;
    let material = store
        .get(body.regions()[1])
        .map_err(|source| Error::InconsistentTopology { source })?;
    if exterior.body() != body_id
        || material.body() != body_id
        || exterior.kind() != RegionKind::Void
        || !exterior.shells().is_empty()
        || material.kind() != RegionKind::Solid
        || material.shells().len() != 1
    {
        return Ok(None);
    }
    let shell_id = material.shells()[0];
    let shell = store
        .get(shell_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    charge(scope, usize_work(shell.faces().len())?)?;
    if shell.region() != body.regions()[1] || !shell.edges().is_empty() || shell.vertex().is_some()
    {
        return Ok(None);
    }
    Ok(Some((shell_id, shell.faces().to_vec())))
}

fn bind_boundaries(
    store: &Store,
    side_face_id: FaceId,
    cap_faces: &[FaceId; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<[UnorderedBoundary; 2]>> {
    let side_face = store
        .get(side_face_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    charge(scope, usize_work(side_face.loops().len())?)?;
    let [loop_a, loop_b] = side_face.loops() else {
        return Ok(None);
    };
    let Some(a) = bind_boundary(store, side_face_id, *loop_a, cap_faces, scope)? else {
        return Ok(None);
    };
    let Some(b) = bind_boundary(store, side_face_id, *loop_b, cap_faces, scope)? else {
        return Ok(None);
    };
    if a.edge == b.edge || a.cap_face == b.cap_face {
        return Ok(None);
    }
    Ok(Some([a, b]))
}

fn bind_boundary(
    store: &Store,
    side_face_id: FaceId,
    side_loop_id: LoopId,
    cap_faces: &[FaceId; 2],
    scope: &mut OperationScope<'_, '_>,
) -> Result<Option<UnorderedBoundary>> {
    charge(scope, 4)?;
    let side_loop = store
        .get(side_loop_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    let [side_fin_id] = side_loop.fins() else {
        return Ok(None);
    };
    let side_fin = store
        .get(*side_fin_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    if side_loop.face() != side_face_id || side_fin.parent() != side_loop_id {
        return Ok(None);
    }
    let edge = store
        .get(side_fin.edge())
        .map_err(|source| Error::InconsistentTopology { source })?;
    if edge.tolerance().is_some() || edge.vertices() != [None, None] || edge.bounds().is_some() {
        return Ok(None);
    }
    let [first, second] = edge.fins() else {
        return Ok(None);
    };
    let Some(cap_fin_id) = peer_fin(*side_fin_id, *first, *second) else {
        return Ok(None);
    };
    let cap_fin = store
        .get(cap_fin_id)
        .map_err(|source| Error::InconsistentTopology { source })?;
    if cap_fin.edge() != side_fin.edge() || cap_fin.sense() == side_fin.sense() {
        return Ok(None);
    }
    let cap_loop = store
        .get(cap_fin.parent())
        .map_err(|source| Error::InconsistentTopology { source })?;
    let cap_face_id = cap_loop.face();
    if !cap_faces.contains(&cap_face_id)
        || cap_loop.fins() != [cap_fin_id]
        || store
            .get(cap_face_id)
            .map_err(|source| Error::InconsistentTopology { source })?
            .loops()
            != [cap_fin.parent()]
    {
        return Ok(None);
    }
    let Some(curve_id) = edge.curve() else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store
        .curve(curve_id)
        .map_err(|source| Error::InconsistentTopology { source })?
    else {
        return Ok(None);
    };
    Ok(Some(UnorderedBoundary {
        cap_face: cap_face_id,
        edge: side_fin.edge(),
        center: circle.frame().origin(),
    }))
}

fn peer_fin(side: FinId, first: FinId, second: FinId) -> Option<FinId> {
    if first == side {
        Some(second)
    } else if second == side {
        Some(first)
    } else {
        None
    }
}

fn order_boundaries(
    cylinder: Cylinder,
    boundaries: [UnorderedBoundary; 2],
) -> Option<[CertifiedCylinderBoundary; 2]> {
    let sign = affine_dot3(
        cylinder.frame().z().to_array(),
        boundaries[1].center.to_array(),
        boundaries[0].center.to_array(),
        0.0,
    )?
    .sign();
    let [low, high] = match sign {
        Orientation::Positive => boundaries,
        Orientation::Negative => [boundaries[1], boundaries[0]],
        Orientation::Zero => return None,
    };
    Some([low, high].map(|boundary| CertifiedCylinderBoundary {
        cap_face: boundary.cap_face,
        edge: boundary.edge,
        center: boundary.center,
    }))
}

fn charge(scope: &mut OperationScope<'_, '_>, amount: u64) -> Result<()> {
    scope
        .ledger_mut()
        .charge(PLANAR_SOURCE_EXTRACTION_WORK, amount)
        .map_err(Error::from)
}

fn usize_work(value: usize) -> Result<u64> {
    u64::try_from(value).map_err(|_| Error::Core {
        source: kcore::error::Error::InvalidGeometry {
            reason: "curved Boolean source scan exceeds u64 accounting",
        },
    })
}

#[cfg(test)]
mod tests {
    use kcore::operation::{OperationContext, OperationScope};
    use kcore::tolerance::Tolerances;
    use kgeom::frame::Frame;
    use kgeom::vec::{Point3, Vec3};

    use super::*;
    use crate::{BlockRequest, BodyId, CylinderRequest, Kernel, Session};

    fn extract(session: &Session, body: &BodyId) -> CylinderSourceOutcome {
        let part = session.part(body.part().clone()).unwrap();
        let context = OperationContext::new(part.policy(), Tolerances::default())
            .unwrap()
            .with_family_budget_defaults(super::super::BooleanBudgetProfile::v1_defaults());
        let mut scope = OperationScope::new(&context);
        extract_cylinder_source(&part.state.store, body.raw(), &mut scope).unwrap()
    }

    #[test]
    fn finite_cylinder_source_is_topology_driven_and_axis_ordered() {
        let frame = Frame::new(
            Point3::new(1.0, -2.0, 0.5),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id.clone())
            .unwrap()
            .create_cylinder(CylinderRequest::new(frame, 1.25, 3.5))
            .unwrap()
            .into_result()
            .unwrap()
            .body();

        {
            let mut edit = session.edit_part(part_id).unwrap();
            let store = edit.store_mut_for_test();
            let material = store.get(body.raw()).unwrap().regions()[1];
            let shell = store.get(material).unwrap().shells()[0];
            let mut transaction = store.transaction().unwrap();
            transaction
                .assembly()
                .get_mut(shell)
                .unwrap()
                .faces
                .rotate_left(2);
            transaction.commit_checked_body(body.raw()).unwrap();
        }

        let CylinderSourceOutcome::Ready(source) = extract(&session, &body) else {
            panic!("reordered finite cylinder must retain its source certificate")
        };
        assert_eq!(source.cylinder().radius(), 1.25);
        assert_eq!(source.boundaries()[0].center(), frame.origin());
        assert_eq!(
            source.boundaries()[1].center(),
            frame.origin() + frame.z() * 3.5
        );
        assert_ne!(
            source.boundaries()[0].cap_face(),
            source.boundaries()[1].cap_face()
        );
        assert_ne!(source.boundaries()[0].edge(), source.boundaries()[1].edge());
    }

    #[test]
    fn full_valid_planar_body_is_not_mistaken_for_a_cylinder() {
        let mut session = Kernel::new().create_session();
        let part_id = session.create_part();
        let body = session
            .edit_part(part_id)
            .unwrap()
            .create_block(BlockRequest::new(Frame::world(), [2.0, 3.0, 4.0]))
            .unwrap()
            .into_result()
            .unwrap()
            .body();
        assert_eq!(
            extract(&session, &body),
            CylinderSourceOutcome::Unsupported(CylinderSourceGap::FaceLayout)
        );
    }
}
