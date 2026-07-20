//! Full shell proof for convex planar hosts carrying cylindrical bands.
//!
//! Recognition is an incidence-graph proof, not a constructor certificate.
//! Every cylindrical side contributes two complete periodic ring uses. Their
//! planar peers are classified from topology as either host ports (a circular
//! inner loop beside one polygonal outer loop) or caps (one circular loop).
//! The polygonal outer loops reconstruct one virtual convex host. Endpoint
//! roles and exact support incidence then derive through-hole, outward-boss,
//! and inward-pocket winding without an operation or feature-count tag.

use super::*;
use kcore::error::Error;

/// Cumulative structural, support, and band-pair decisions for connected
/// cylindrical-host shells.
pub(crate) const CYLINDRICAL_HOST_SHELL_WORK: StageId =
    match StageId::new("ktopo.check.cylindrical-host-shell-work") {
        Ok(stage) => stage,
        Err(_) => panic!("valid cylindrical host shell work stage"),
    };

const DEFAULT_CYLINDRICAL_HOST_SHELL_WORK: u64 = 1_048_576;

pub(super) fn cylindrical_host_proof_budget() -> BudgetPlan {
    BudgetPlan::new([LimitSpec::new(
        CYLINDRICAL_HOST_SHELL_WORK,
        ResourceKind::Work,
        AccountingMode::Cumulative,
        DEFAULT_CYLINDRICAL_HOST_SHELL_WORK,
    )])
    .expect("built-in cylindrical host shell proof budget is valid")
}

#[derive(Debug, Clone, Copy)]
struct RingBoundary {
    planar_face: FaceId,
    planar_loop: LoopId,
    edge: EdgeId,
    center: Point3,
    planar_axis_alignment: PredicateOrientation,
    side_traverses_positive_u: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EndpointRole {
    Port,
    Cap,
}

#[derive(Debug, Clone, Copy)]
struct BandEvidence {
    side_face: FaceId,
    cylinder: Cylinder,
    low: RingBoundary,
    high: RingBoundary,
    roles: [Option<EndpointRole>; 2],
}

#[derive(Debug)]
struct ShellClasses {
    cylinders: Vec<(FaceId, Cylinder)>,
    planar_faces: Vec<FaceId>,
    loop_count: usize,
    fin_count: usize,
}

type HostFacets = Vec<(FaceId, Vec<VertexId>)>;

/// Certify one connected positive shell reconstructed from a convex planar
/// host and any nonzero number of full-period cylindrical bands.
pub(super) fn certify_cylindrical_host_shell(
    store: &Store,
    shell_id: ShellId,
    mut scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellCertification>> {
    let shell = store.get(shell_id)?;
    if shell.faces.len() < 5 || !shell.edges.is_empty() || shell.vertex.is_some() {
        return Ok(None);
    }
    if let Some(scope) = scope.as_deref_mut() {
        scope.ledger().require_limit(
            CYLINDRICAL_HOST_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
        )?;
        charge_count(scope, shell.faces.len())?;
    }

    let Some(classes) = classify_shell(store, shell_id, scope.as_deref_mut())? else {
        return Ok(None);
    };
    if classes.cylinders.is_empty() {
        return Ok(None);
    }
    let Some(proof_work) = proof_work(&classes) else {
        return Ok(Some(indeterminate()));
    };
    if let Some(scope) = scope.as_deref_mut() {
        scope
            .ledger_mut()
            .charge(CYLINDRICAL_HOST_SHELL_WORK, proof_work)?;
    }

    let Some(mut bands) = prepare_bands(store, shell_id, &classes.cylinders)? else {
        return Ok(None);
    };
    let Some(host_facets) = classify_endpoints_and_host(store, &classes.planar_faces, &mut bands)?
    else {
        return Ok(None);
    };
    if bands
        .iter()
        .any(|band| band.roles.iter().any(Option::is_none))
    {
        return Ok(None);
    }

    let host = certify_convex_planar_facets(store, host_facets.clone(), scope)?;
    if host.embedding != ShellEmbedding::Certified {
        return Ok(Some(indeterminate()));
    }

    let mut orientation_invalid = host.orientation != ShellOrientation::Positive;
    for band in &bands {
        let Some(invalid) = certify_band_in_host(store, *band, &host_facets)? else {
            return Ok(None);
        };
        orientation_invalid |= invalid;
    }
    if !bands_are_pairwise_separated(&bands) {
        return Ok(Some(indeterminate()));
    }

    Ok(Some(ShellCertification {
        embedding: ShellEmbedding::Certified,
        orientation: if orientation_invalid {
            ShellOrientation::Invalid
        } else {
            ShellOrientation::Positive
        },
    }))
}

fn classify_shell(
    store: &Store,
    shell_id: ShellId,
    mut scope: Option<&mut OperationScope<'_, '_>>,
) -> Result<Option<ShellClasses>> {
    let shell = store.get(shell_id)?;
    let mut cylinders = Vec::new();
    let mut planar_faces = Vec::new();
    let mut loop_count = 0_usize;
    for &face_id in &shell.faces {
        let face = store.get(face_id)?;
        if face.shell != shell_id {
            return Ok(None);
        }
        match store.get(face.surface)? {
            SurfaceGeom::Cylinder(cylinder) => cylinders.push((face_id, *cylinder)),
            SurfaceGeom::Plane(_) => planar_faces.push(face_id),
            _ => return Ok(None),
        }
        let Some(next) = loop_count.checked_add(face.loops.len()) else {
            return Ok(None);
        };
        loop_count = next;
    }
    if cylinders.is_empty() {
        return Ok(None);
    }
    if let Some(scope) = scope.as_deref_mut() {
        charge_count(scope, loop_count)?;
    }
    let mut fin_count = 0_usize;
    for &face_id in &shell.faces {
        for &loop_id in &store.get(face_id)?.loops {
            let loop_ = store.get(loop_id)?;
            let Some(next) = fin_count.checked_add(loop_.fins.len()) else {
                return Ok(None);
            };
            fin_count = next;
        }
    }
    if let Some(scope) = scope {
        charge_count(scope, fin_count)?;
    }
    Ok(Some(ShellClasses {
        cylinders,
        planar_faces,
        loop_count,
        fin_count,
    }))
}

/// Input-size-exact conservative bound for the quadratic scans below:
/// loop/layout and vertex deduplication, face/vertex support decisions,
/// whole-fin ownership scans, endpoint support decisions, and band pairs.
fn proof_work(classes: &ShellClasses) -> Option<u64> {
    let face_count = classes
        .cylinders
        .len()
        .checked_add(classes.planar_faces.len())?;
    let faces = u64::try_from(face_count).ok()?;
    let loops = u64::try_from(classes.loop_count).ok()?;
    let fins = u64::try_from(classes.fin_count).ok()?;
    let bands = u64::try_from(classes.cylinders.len()).ok()?;
    let layout = loops.checked_mul(fins)?;
    let host_supports = faces.checked_mul(fins)?;
    let incidence_and_sweeps = faces.checked_mul(bands)?.checked_mul(8)?;
    let band_pairs = bands.checked_mul(bands.checked_sub(1)?)?.checked_div(2)?;
    layout
        .checked_add(host_supports)?
        .checked_add(incidence_and_sweeps)?
        .checked_add(band_pairs)
}

fn charge_count(scope: &mut OperationScope<'_, '_>, count: usize) -> Result<()> {
    let amount = u64::try_from(count).map_err(|_| Error::InvalidGeometry {
        reason: "cylindrical host proof work count exceeds u64",
    })?;
    scope
        .ledger_mut()
        .charge(CYLINDRICAL_HOST_SHELL_WORK, amount)?;
    Ok(())
}

fn prepare_bands(
    store: &Store,
    shell_id: ShellId,
    cylinders: &[(FaceId, Cylinder)],
) -> Result<Option<Vec<BandEvidence>>> {
    let mut bands = Vec::with_capacity(cylinders.len());
    let mut used_edges = Vec::with_capacity(cylinders.len());
    let mut used_planar_loops = Vec::with_capacity(cylinders.len());
    for &(side_face, cylinder) in cylinders {
        let face = store.get(side_face)?;
        let [first_loop, second_loop] = face.loops.as_slice() else {
            return Ok(None);
        };
        let Some(first) =
            host_cylinder_ring_boundary(store, shell_id, side_face, cylinder, *first_loop)?
        else {
            return Ok(None);
        };
        let Some(second) =
            host_cylinder_ring_boundary(store, shell_id, side_face, cylinder, *second_loop)?
        else {
            return Ok(None);
        };
        if first.edge == second.edge
            || first.planar_face == second.planar_face
            || [first.edge, second.edge]
                .iter()
                .any(|edge| used_edges.contains(edge))
            || [first.planar_loop, second.planar_loop]
                .iter()
                .any(|loop_id| used_planar_loops.contains(loop_id))
        {
            return Ok(None);
        }
        used_edges.extend([first.edge, second.edge]);
        used_planar_loops.extend([first.planar_loop, second.planar_loop]);
        let (low, high) = match exact_affine_sign(cylinder.frame().z(), second.center, first.center)
        {
            Some(PredicateOrientation::Positive) => (first, second),
            Some(PredicateOrientation::Negative) => (second, first),
            _ => return Ok(None),
        };
        bands.push(BandEvidence {
            side_face,
            cylinder,
            low,
            high,
            roles: [None, None],
        });
    }
    Ok(Some(bands))
}

fn classify_endpoints_and_host(
    store: &Store,
    planar_faces: &[FaceId],
    bands: &mut [BandEvidence],
) -> Result<Option<HostFacets>> {
    let mut host_facets = Vec::with_capacity(planar_faces.len());
    let mut cap_faces = Vec::new();
    for &face_id in planar_faces {
        let face = store.get(face_id)?;
        let references = boundary_references(face_id, bands);
        if references.is_empty() {
            let Some(vertices) = convex_planar_face_vertices(store, face_id)? else {
                return Ok(None);
            };
            host_facets.push((face_id, vertices));
            continue;
        }

        if references.len() == 1 && face.loops.as_slice() == [references[0].2] {
            if cap_faces.contains(&face_id) {
                return Ok(None);
            }
            cap_faces.push(face_id);
            set_role(bands, references[0], EndpointRole::Cap)?;
            continue;
        }

        if references
            .iter()
            .any(|(_, _, loop_id)| !face.loops.contains(loop_id))
            || certify_face_loop_layout(store, face_id)? != LoopContainment::Certified
        {
            return Ok(None);
        }
        let boundary_loops = references
            .iter()
            .map(|reference| reference.2)
            .collect::<Vec<_>>();
        let outer_loops = face
            .loops
            .iter()
            .copied()
            .filter(|loop_id| !boundary_loops.contains(loop_id))
            .collect::<Vec<_>>();
        let [outer_loop] = outer_loops.as_slice() else {
            return Ok(None);
        };
        let Some(vertices) = convex_planar_face_loop_vertices(store, face_id, *outer_loop)? else {
            return Ok(None);
        };
        for reference in references {
            set_role(bands, reference, EndpointRole::Port)?;
        }
        host_facets.push((face_id, vertices));
    }
    if host_facets.len() < 4 {
        return Ok(None);
    }
    Ok(Some(host_facets))
}

fn boundary_references(face: FaceId, bands: &[BandEvidence]) -> Vec<(usize, usize, LoopId)> {
    let mut references = Vec::new();
    for (band_index, band) in bands.iter().enumerate() {
        for (endpoint, boundary) in [band.low, band.high].into_iter().enumerate() {
            if boundary.planar_face == face {
                references.push((band_index, endpoint, boundary.planar_loop));
            }
        }
    }
    references
}

fn set_role(
    bands: &mut [BandEvidence],
    reference: (usize, usize, LoopId),
    role: EndpointRole,
) -> Result<()> {
    let (band, endpoint, _) = reference;
    let Some(slot) = bands
        .get_mut(band)
        .and_then(|band| band.roles.get_mut(endpoint))
    else {
        return Err(Error::InvalidGeometry {
            reason: "cylindrical host endpoint reference is invalid",
        });
    };
    if slot.replace(role).is_some() {
        return Err(Error::InvalidGeometry {
            reason: "cylindrical host endpoint was classified twice",
        });
    }
    Ok(())
}

/// Return whether an otherwise embedded band has invalid winding.
fn certify_band_in_host(
    store: &Store,
    band: BandEvidence,
    host_facets: &HostFacets,
) -> Result<Option<bool>> {
    let [Some(low_role), Some(high_role)] = band.roles else {
        return Ok(None);
    };
    let side_face = store.get(band.side_face)?;
    let low_invalid = endpoint_loop_orientation_invalid(store, band.low, low_role)?;
    let high_invalid = endpoint_loop_orientation_invalid(store, band.high, high_role)?;
    let expected_low_traversal = side_face.sense == Sense::Forward;
    let expected_high_traversal = side_face.sense == Sense::Reversed;
    let mut invalid = low_invalid
        || high_invalid
        || band.low.side_traverses_positive_u != expected_low_traversal
        || band.high.side_traverses_positive_u != expected_high_traversal;

    match [low_role, high_role] {
        [EndpointRole::Port, EndpointRole::Port] => {
            let low_face = store.get(band.low.planar_face)?;
            let high_face = store.get(band.high.planar_face)?;
            invalid |= side_face.sense != Sense::Reversed
                || oriented_axis_alignment(band.low.planar_axis_alignment, low_face.sense)
                    != Some(-1)
                || oriented_axis_alignment(band.high.planar_axis_alignment, high_face.sense)
                    != Some(1);
            if !certify_port_to_port_sweep(store, band, host_facets)? {
                return Ok(None);
            }
        }
        [EndpointRole::Port, EndpointRole::Cap] | [EndpointRole::Cap, EndpointRole::Port] => {
            let (port, cap, cap_direction) = if low_role == EndpointRole::Port {
                (band.low, band.high, 1)
            } else {
                (band.high, band.low, -1)
            };
            let port_face = store.get(port.planar_face)?;
            let cap_face = store.get(cap.planar_face)?;
            let Some(port_outward) =
                oriented_axis_alignment(port.planar_axis_alignment, port_face.sense)
            else {
                return Ok(None);
            };
            let Some(cap_outward) =
                oriented_axis_alignment(cap.planar_axis_alignment, cap_face.sense)
            else {
                return Ok(None);
            };
            let outward = cap_direction == port_outward;
            invalid |= side_face.sense
                != if outward {
                    Sense::Forward
                } else {
                    Sense::Reversed
                }
                || cap_outward != port_outward;
            if !outward && !certify_inward_sweep(store, band, port, cap, host_facets)? {
                return Ok(None);
            }
        }
        [EndpointRole::Cap, EndpointRole::Cap] => return Ok(None),
    }
    Ok(Some(invalid))
}

fn endpoint_loop_orientation_invalid(
    store: &Store,
    boundary: RingBoundary,
    role: EndpointRole,
) -> Result<bool> {
    let face = store.get(boundary.planar_face)?;
    let orientation = certify_loop_orientation(store, boundary.planar_face, boundary.planar_loop)?;
    let expected_positive = match role {
        EndpointRole::Port => !face.sense.is_forward(),
        EndpointRole::Cap => face.sense.is_forward(),
    };
    Ok(orientation.is_none_or(|orientation| {
        (orientation == PredicateOrientation::Positive) != expected_positive
    }))
}

fn certify_port_to_port_sweep(
    store: &Store,
    band: BandEvidence,
    host_facets: &HostFacets,
) -> Result<bool> {
    for (face_id, _) in host_facets {
        let face = store.get(*face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(false);
        };
        let outward = plane.frame().z() * sense_factor(face.sense);
        if *face_id == band.low.planar_face || *face_id == band.high.planar_face {
            let (incident, opposite) = if *face_id == band.low.planar_face {
                (band.low.center, band.high.center)
            } else {
                (band.high.center, band.low.center)
            };
            if exact_affine_sign(outward, incident, plane.frame().origin())
                != Some(PredicateOrientation::Zero)
                || exact_affine_sign(outward, opposite, plane.frame().origin())
                    != Some(PredicateOrientation::Negative)
                || exact_vector_dot(outward, band.cylinder.frame().x())
                    != Some(PredicateOrientation::Zero)
                || exact_vector_dot(outward, band.cylinder.frame().y())
                    != Some(PredicateOrientation::Zero)
            {
                return Ok(false);
            }
            continue;
        }
        if !endpoint_circles_inside_support(store, band, *face_id)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn certify_inward_sweep(
    store: &Store,
    band: BandEvidence,
    port: RingBoundary,
    cap: RingBoundary,
    host_facets: &HostFacets,
) -> Result<bool> {
    for (face_id, _) in host_facets {
        let face = store.get(*face_id)?;
        let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
            return Ok(false);
        };
        let outward = plane.frame().z() * sense_factor(face.sense);
        if *face_id == port.planar_face {
            if exact_affine_sign(outward, port.center, plane.frame().origin())
                != Some(PredicateOrientation::Zero)
                || exact_affine_sign(outward, cap.center, plane.frame().origin())
                    != Some(PredicateOrientation::Negative)
                || exact_vector_dot(outward, band.cylinder.frame().x())
                    != Some(PredicateOrientation::Zero)
                || exact_vector_dot(outward, band.cylinder.frame().y())
                    != Some(PredicateOrientation::Zero)
            {
                return Ok(false);
            }
            continue;
        }
        if !endpoint_circles_inside_support(store, band, *face_id)? {
            return Ok(false);
        }
    }
    Ok(true)
}

fn endpoint_circles_inside_support(
    store: &Store,
    band: BandEvidence,
    face_id: FaceId,
) -> Result<bool> {
    let face = store.get(face_id)?;
    let SurfaceGeom::Plane(plane) = store.get(face.surface)? else {
        return Ok(false);
    };
    let outward = plane.frame().z() * sense_factor(face.sense);
    for center in [band.low.center, band.high.center] {
        if exact_affine_sign(outward, center, plane.frame().origin())
            != Some(PredicateOrientation::Negative)
            || !certify_circle_strictly_inside_support(
                outward,
                plane.frame().origin(),
                band.cylinder,
                center,
            )
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn certify_circle_strictly_inside_support(
    outward: Vec3,
    support_origin: Point3,
    cylinder: Cylinder,
    center: Point3,
) -> bool {
    let signed = interval_vector_dot(outward, center - support_origin);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(cylinder.radius());
    let radial_x = interval_vector_dot(outward, cylinder.frame().x()) * radius;
    let radial_y = interval_vector_dot(outward, cylinder.frame().y()) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    radial_squared.hi() < signed.square().lo()
}

fn interval_vector_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
}

fn bands_are_pairwise_separated(bands: &[BandEvidence]) -> bool {
    for left in 0..bands.len() {
        for right in left + 1..bands.len() {
            if !parallel_axial_slabs_are_strictly_separated(bands[left], bands[right]) {
                return false;
            }
        }
    }
    true
}

fn parallel_axial_slabs_are_strictly_separated(first: BandEvidence, second: BandEvidence) -> bool {
    let Some(alignment) = exact_axis_alignment(first.cylinder.frame(), second.cylinder.frame().z())
    else {
        return false;
    };
    let (second_low, second_high) = if alignment == PredicateOrientation::Positive {
        (second.low.center, second.high.center)
    } else {
        (second.high.center, second.low.center)
    };
    exact_affine_sign(first.cylinder.frame().z(), second_low, first.high.center)
        == Some(PredicateOrientation::Positive)
        || exact_affine_sign(first.cylinder.frame().z(), first.low.center, second_high)
            == Some(PredicateOrientation::Positive)
}

fn host_cylinder_ring_boundary(
    store: &Store,
    shell_id: ShellId,
    side_face_id: FaceId,
    cylinder: Cylinder,
    side_loop_id: LoopId,
) -> Result<Option<RingBoundary>> {
    let side_loop = store.get(side_loop_id)?;
    let [side_fin_id] = side_loop.fins.as_slice() else {
        return Ok(None);
    };
    if side_loop.face != side_face_id
        || certify_loop_simplicity(store, side_loop_id)? != LoopSimplicity::Certified
    {
        return Ok(None);
    }
    let side_fin = store.get(*side_fin_id)?;
    let edge = store.get(side_fin.edge)?;
    let [first_fin, second_fin] = edge.fins.as_slice() else {
        return Ok(None);
    };
    if side_fin.parent != side_loop_id
        || edge.tolerance.is_some()
        || edge.bounds.is_some()
        || edge.vertices != [None, None]
        || !edge.fins.contains(side_fin_id)
    {
        return Ok(None);
    }
    let planar_fin_id = if first_fin == side_fin_id {
        *second_fin
    } else if second_fin == side_fin_id {
        *first_fin
    } else {
        return Ok(None);
    };
    let planar_fin = store.get(planar_fin_id)?;
    if planar_fin.edge != side_fin.edge || planar_fin.sense == side_fin.sense {
        return Ok(None);
    }
    let planar_loop_id = planar_fin.parent;
    let planar_loop = store.get(planar_loop_id)?;
    let planar_face_id = planar_loop.face;
    let planar_face = store.get(planar_face_id)?;
    if planar_face.shell != shell_id
        || planar_loop.fins.as_slice() != [planar_fin_id]
        || !planar_face.loops.contains(&planar_loop_id)
        || certify_loop_simplicity(store, planar_loop_id)? != LoopSimplicity::Certified
        || !matches!(store.get(planar_face.surface)?, SurfaceGeom::Plane(_))
    {
        return Ok(None);
    }
    if certify_whole_fin_incidence(
        store,
        side_face_id,
        side_loop_id,
        *side_fin_id,
        LINEAR_RESOLUTION,
    ) != WholeFinIncidence::Certified
        || certify_whole_fin_incidence(
            store,
            planar_face_id,
            planar_loop_id,
            planar_fin_id,
            LINEAR_RESOLUTION,
        ) != WholeFinIncidence::Certified
    {
        return Ok(None);
    }

    let Some(curve_id) = edge.curve else {
        return Ok(None);
    };
    let CurveGeom::Circle(circle) = store.get(curve_id)? else {
        return Ok(None);
    };
    let SurfaceGeom::Plane(plane) = store.get(planar_face.surface)? else {
        unreachable!("host ring classification retains a plane");
    };
    if circle.radius() != cylinder.radius()
        || exact_axis_alignment(cylinder.frame(), circle.frame().z()).is_none()
        || !certified_point_on_axis(cylinder.frame(), circle.frame().origin())
        || exact_affine_sign(
            plane.frame().z(),
            circle.frame().origin(),
            plane.frame().origin(),
        ) != Some(PredicateOrientation::Zero)
    {
        return Ok(None);
    }
    let Some(planar_axis_alignment) = exact_axis_alignment(cylinder.frame(), plane.frame().z())
    else {
        return Ok(None);
    };
    let Some(side_use) = side_fin.pcurve else {
        return Ok(None);
    };
    let Curve2dGeom::Line(side_line) = store.get(side_use.curve())? else {
        return Ok(None);
    };
    if side_line.dir().y != 0.0 || side_line.dir().x == 0.0 {
        return Ok(None);
    }
    let Some(side_traverses_positive_u) = traversal_is_positive(
        [side_line.dir().x, side_use.edge_to_pcurve().scale()],
        side_fin.sense,
    ) else {
        return Ok(None);
    };
    let Some(planar_use) = planar_fin.pcurve else {
        return Ok(None);
    };
    if !matches!(store.get(planar_use.curve())?, Curve2dGeom::Circle(_))
        || planar_use.closure_winding() != Some([0, 0])
    {
        return Ok(None);
    }
    Ok(Some(RingBoundary {
        planar_face: planar_face_id,
        planar_loop: planar_loop_id,
        edge: side_fin.edge,
        center: circle.frame().origin(),
        planar_axis_alignment,
        side_traverses_positive_u,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::CheckOutcome;
    use crate::cylindrical_host::{
        CylindricalHostBandInput, CylindricalHostEndpoint, CylindricalHostSolidInput,
    };
    use crate::entity::Body;
    use crate::planar::{PlanarSolidFace, PlanarSolidInput, PlanarSolidVertex, PlanarVertexKey};
    use crate::transaction::FullCommitRequirement;
    use kcore::operation::{LimitSnapshot, OperationContext, SessionPolicy};
    use kcore::tolerance::Tolerances;
    use kgeom::param::ParamRange;

    const TWO_BAND_PROOF_WORK: u64 = 985;

    fn cube() -> PlanarSolidInput {
        let points = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
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
        .map(|ring| PlanarSolidFace::new(ring.map(|vertex| keys[vertex]).to_vec()))
        .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn two_outward_bands() -> CylindricalHostSolidInput {
        let low = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::port(0),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let high = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::cap(),
                CylindricalHostEndpoint::port(1),
            ],
        );
        CylindricalHostSolidInput::new(cube(), vec![high, low])
    }

    fn proof_budget(allowed: u64) -> BudgetPlan {
        BudgetPlan::new([LimitSpec::new(
            CYLINDRICAL_HOST_SHELL_WORK,
            ResourceKind::Work,
            AccountingMode::Cumulative,
            allowed,
        )])
        .unwrap()
    }

    #[test]
    fn multiple_outward_bands_are_full_valid_independent_of_storage_order() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        transaction
            .store_mut()
            .get_mut(output.shell())
            .unwrap()
            .faces
            .reverse();
        let faces = transaction
            .store()
            .get(output.shell())
            .unwrap()
            .faces
            .clone();
        for face in faces {
            transaction
                .store_mut()
                .get_mut(face)
                .unwrap()
                .loops
                .reverse();
        }

        let decision = transaction
            .commit_full(&[output.body()], FullCommitRequirement::RequireValid)
            .unwrap();
        assert!(decision.is_committed(), "checks: {:?}", decision.checks());
        assert!(decision.checks().iter().all(|check| {
            check.report().outcome() == CheckOutcome::Valid && check.report().gaps.is_empty()
        }));
    }

    #[test]
    fn multiple_outward_bands_with_wrong_side_sense_are_full_invalid() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        let side = output.bands()[0].side_face();
        transaction.store_mut().get_mut(side).unwrap().sense = Sense::Reversed;

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
    fn touching_and_overlapping_axial_slabs_are_not_certified_separate() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        let classes = classify_shell(transaction.store(), output.shell(), None)
            .unwrap()
            .unwrap();
        let bands = prepare_bands(transaction.store(), output.shell(), &classes.cylinders)
            .unwrap()
            .unwrap();
        assert_eq!(bands.len(), 2);
        assert!(parallel_axial_slabs_are_strictly_separated(
            bands[0], bands[1]
        ));

        let mut touching = bands[1];
        touching.low.center = bands[0].high.center;
        touching.high.center = bands[0].high.center + bands[0].cylinder.frame().z();
        assert!(!parallel_axial_slabs_are_strictly_separated(
            bands[0], touching
        ));

        let mut overlapping = bands[1];
        overlapping.low.center = bands[0].low.center;
        overlapping.high.center = bands[0].high.center;
        assert!(!parallel_axial_slabs_are_strictly_separated(
            bands[0],
            overlapping
        ));
    }

    #[test]
    fn cylindrical_host_shell_work_accepts_exact_n_and_n_minus_one_rolls_back() {
        let mut accepted_store = Store::new();
        let accepted_session = SessionPolicy::v1();
        let accepted_context = OperationContext::new(&accepted_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(proof_budget(TWO_BAND_PROOF_WORK));
        let mut accepted = accepted_store.transaction().unwrap();
        let accepted_output = accepted
            .assemble_cylindrical_host_solid(&two_outward_bands())
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
            .find(|usage| usage.stage == CYLINDRICAL_HOST_SHELL_WORK)
            .copied()
            .unwrap();
        assert_eq!(
            (usage.consumed, usage.allowed),
            (TWO_BAND_PROOF_WORK, TWO_BAND_PROOF_WORK)
        );

        let mut denied_store = Store::new();
        let denied_session = SessionPolicy::v1();
        let denied_context = OperationContext::new(&denied_session, Tolerances::default())
            .unwrap()
            .with_budget_overrides(proof_budget(TWO_BAND_PROOF_WORK - 1));
        let mut denied = denied_store.transaction().unwrap();
        let denied_output = denied
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        let rolled_back_body = denied_output.body();
        let denied = denied
            .commit_full_with_context(
                &[rolled_back_body],
                FullCommitRequirement::RequireValid,
                &denied_context,
            )
            .unwrap();
        let expected = LimitSnapshot {
            stage: CYLINDRICAL_HOST_SHELL_WORK,
            resource: ResourceKind::Work,
            consumed: TWO_BAND_PROOF_WORK,
            allowed: TWO_BAND_PROOF_WORK - 1,
        };
        assert_eq!(
            denied.result().as_ref().unwrap_err().limit(),
            Some(expected)
        );
        assert_eq!(denied.report().limit_events(), &[expected]);
        assert_eq!(denied_store.count::<Body>(), 0);

        let mut retry = denied_store.transaction().unwrap();
        let retried = retry
            .assemble_cylindrical_host_solid(&two_outward_bands())
            .unwrap();
        assert_eq!(retried.body(), rolled_back_body);
    }
}
