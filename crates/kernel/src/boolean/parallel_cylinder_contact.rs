//! Exact closed-ring realization for axial-contact cylinder unions.
//!
//! The axial relation has already proved that two live cap/ring boundaries
//! meet at one exact common axial level. This module classifies their radial
//! disk relation with exact dyadic predicates. Strict containment retains the
//! two source side bands and replaces the coincident cap pair by one annulus;
//! coincident disks coalesce to one longer cylindrical band. Both proposals
//! are authored only from live analytic/topological source evidence and are
//! committed through the shared Full-checked analytic-shell path.
//!
//! Strict secancy binds Section's two proof-joined inside arcs to the source
//! rings, completes each ring with its exact outside arc, and assembles the
//! two shared-cap crescents. Tangency remains a typed contact refusal.

use kcore::interval::Interval;
use kcore::operation::OperationScope;
use kcore::predicates::{Orientation, affine_dot3, orient3d};
use kcore::tolerance::LINEAR_RESOLUTION;
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};
use kgraph::AffineParamMap1d;
use ktopo::analytic_shell::{
    AnalyticEdgeKey, AnalyticFaceKey, AnalyticPcurveUse, AnalyticShellClosedEdge,
    AnalyticShellCurve, AnalyticShellFace, AnalyticShellFin, AnalyticShellInput, AnalyticShellLoop,
    AnalyticShellPcurve, AnalyticShellSurface,
};
use ktopo::entity::{EntityRef, FaceDomain, FaceId as RawFaceId, FinId as RawFinId, Sense};
use ktopo::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use ktopo::store::Store;

use super::curved_pipeline::{
    CurvedBooleanPipelineOutcome, CurvedBooleanPipelineRefusal, StageResult, refused,
};
use super::curved_realize::realize_analytic_shell_input;
use super::curved_source::{CertifiedCylinderBoundary, CertifiedCylinderSource};
use super::parallel_cylinder_relation::{
    CertifiedParallelCylinderAxialContact, ParallelCylinderAxialBoundaryWitness,
    interval_axis_distance_squared,
};
use crate::BodyId;
use crate::BodySectionGraph;
use crate::session::PartEdit;

mod secant;

const PERIOD: f64 = core::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactRadialRelation {
    StrictSecant,
    StrictInternal { outer: usize },
    Coincident,
    BoundaryContact,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ContactPlanGap {
    RelationBinding,
    SourceTopology,
    ArithmeticGuard,
    BoundaryContact,
}

#[derive(Clone, Copy)]
struct ContactSource<'a> {
    source: &'a CertifiedCylinderSource,
    contact_boundary: usize,
    far_boundary: usize,
}

/// Realize exactly classified positive-area axial contact.
pub(super) fn execute_parallel_cylinder_contact_unite(
    edit: &mut PartEdit<'_>,
    _bodies: &[BodyId; 2],
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    contact: &CertifiedParallelCylinderAxialContact,
    linear: f64,
    scope: &mut OperationScope<'_, '_>,
) -> StageResult<CurvedBooleanPipelineOutcome> {
    let input = match prepare_contact_unite(&edit.state.store, cylinders, graph, contact) {
        Ok(input) => input,
        Err(ContactPlanGap::BoundaryContact) => {
            return refused(CurvedBooleanPipelineRefusal::ClassificationBoundaryContact);
        }
        Err(
            ContactPlanGap::RelationBinding
            | ContactPlanGap::SourceTopology
            | ContactPlanGap::ArithmeticGuard,
        ) => {
            return refused(CurvedBooleanPipelineRefusal::ResultTopologyUnsupported);
        }
    };
    realize_analytic_shell_input(edit, &input, linear, scope)
}

fn prepare_contact_unite(
    store: &Store,
    cylinders: [&CertifiedCylinderSource; 2],
    graph: &BodySectionGraph,
    contact: &CertifiedParallelCylinderAxialContact,
) -> Result<AnalyticShellInput, ContactPlanGap> {
    let sources = bind_contact_sources(cylinders, contact)?;
    match classify_radial_contact(store, &sources)? {
        ContactRadialRelation::StrictInternal { outer } => {
            prepare_internal_contact_shell(store, &sources, outer)
        }
        ContactRadialRelation::Coincident => prepare_coincident_contact_shell(store, &sources),
        ContactRadialRelation::StrictSecant => {
            secant::prepare_strict_secant_contact_shell(store, graph, &sources)
        }
        ContactRadialRelation::BoundaryContact => Err(ContactPlanGap::BoundaryContact),
    }
}

fn bind_contact_sources<'a>(
    cylinders: [&'a CertifiedCylinderSource; 2],
    relation: &CertifiedParallelCylinderAxialContact,
) -> Result<[ContactSource<'a>; 2], ContactPlanGap> {
    let mut boundaries = [None; 2];
    for witness in relation.contact_boundaries() {
        let operand = witness.operand();
        let source = cylinders
            .get(operand)
            .ok_or(ContactPlanGap::RelationBinding)?;
        let boundary = source
            .boundaries()
            .get(witness.boundary())
            .ok_or(ContactPlanGap::RelationBinding)?;
        if boundary.cap_face() != witness.cap_face()
            || boundary.edge() != witness.edge()
            || boundaries[operand].replace(witness).is_some()
        {
            return Err(ContactPlanGap::RelationBinding);
        }
    }
    let [Some(first), Some(second)] = boundaries else {
        return Err(ContactPlanGap::RelationBinding);
    };
    Ok([
        contact_source(cylinders[0], first)?,
        contact_source(cylinders[1], second)?,
    ])
}

fn contact_source<'a>(
    source: &'a CertifiedCylinderSource,
    witness: &ParallelCylinderAxialBoundaryWitness,
) -> Result<ContactSource<'a>, ContactPlanGap> {
    let contact_boundary = witness.boundary();
    let far_boundary = 1_usize
        .checked_sub(contact_boundary)
        .ok_or(ContactPlanGap::RelationBinding)?;
    Ok(ContactSource {
        source,
        contact_boundary,
        far_boundary,
    })
}

/// Classify disk support from outward enclosures of the authored dyadic
/// inputs. Rounded sums, differences, and center deltas are never decision
/// authority. Boundary-touching or overlapping enclosures fail closed.
fn classify_radial_contact(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
) -> Result<ContactRadialRelation, ContactPlanGap> {
    let circles = [
        source_circle(
            store,
            sources[0].source.boundaries()[sources[0].contact_boundary],
        )?,
        source_circle(
            store,
            sources[1].source.boundaries()[sources[1].contact_boundary],
        )?,
    ];
    let centers = circles.map(|circle| circle.frame().origin());
    let radii = circles.map(|circle| circle.radius());
    if radii
        .into_iter()
        .any(|radius| !radius.is_finite() || radius <= 0.0)
    {
        return Err(ContactPlanGap::ArithmeticGuard);
    }
    if centers[0] == centers[1]
        && radii[0].to_bits() == radii[1].to_bits()
        && coincident_supports_are_coalescible(sources)
    {
        return Ok(ContactRadialRelation::Coincident);
    }
    let distance_squared = interval_distance_squared(centers[1], centers[0]);
    let first = Interval::point(radii[0]);
    let second = Interval::point(radii[1]);
    let difference_squared = (first - second).square();
    let sum_squared = (first + second).square();
    let outer = usize::from(radii[1] > radii[0]);
    let internal_clearance = Interval::point(radii[outer])
        - Interval::point(radii[1 - outer])
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    if !finite_interval(distance_squared)
        || !finite_interval(difference_squared)
        || !finite_interval(sum_squared)
    {
        return Err(ContactPlanGap::ArithmeticGuard);
    }
    if distance_squared.lo() > difference_squared.hi() && distance_squared.hi() < sum_squared.lo() {
        Ok(ContactRadialRelation::StrictSecant)
    } else if finite_interval(internal_clearance)
        && internal_clearance.lo() > 0.0
        && distance_squared.hi() < internal_clearance.square().lo()
    {
        if strictly_contains_cylinder_support(sources, outer) {
            Ok(ContactRadialRelation::StrictInternal { outer })
        } else {
            Ok(ContactRadialRelation::BoundaryContact)
        }
    } else {
        Ok(ContactRadialRelation::BoundaryContact)
    }
}

/// Coalescing replaces both side parameterizations by the first support. The
/// source axes must therefore be exact-parallel, equal-radius, exactly
/// coaxial, and every rebuilt ring center must lie exactly on the chosen
/// infinite-cylinder axis. Near-coaxial supports cannot acquire unrecorded
/// approximation provenance through this path.
fn coincident_supports_are_coalescible(sources: &[ContactSource<'_>; 2]) -> bool {
    let first = sources[0].source.cylinder();
    let second = sources[1].source.cylinder();
    cylinders_have_exact_common_support(first, second)
        && sources.iter().all(|source| {
            source.source.boundaries().iter().all(|boundary| {
                points_are_exactly_axis_aligned(
                    boundary.center(),
                    first.frame().origin(),
                    first.frame().z(),
                )
            })
        })
}

fn cylinders_have_exact_common_support(first: Cylinder, second: Cylinder) -> bool {
    first.radius().to_bits() == second.radius().to_bits()
        && vectors_are_exactly_parallel(first.frame().z(), second.frame().z())
        && points_are_exactly_axis_aligned(
            second.frame().origin(),
            first.frame().origin(),
            first.frame().z(),
        )
}

fn vectors_are_exactly_parallel(first: Vec3, second: Vec3) -> bool {
    if first == second || first == -second {
        return true;
    }
    [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        .into_iter()
        .all(|basis| {
            orient3d(first.to_array(), second.to_array(), basis, [0.0; 3]) == Orientation::Zero
        })
}

fn strictly_contains_cylinder_support(sources: &[ContactSource<'_>; 2], outer: usize) -> bool {
    let inner = 1 - outer;
    let outer_cylinder = sources[outer].source.cylinder();
    let inner_cylinder = sources[inner].source.cylinder();
    if outer_cylinder.radius() <= inner_cylinder.radius()
        || !vectors_are_exactly_parallel(outer_cylinder.frame().z(), inner_cylinder.frame().z())
    {
        return false;
    }
    let Some(distance) = interval_axis_distance_squared(
        inner_cylinder.frame().origin(),
        outer_cylinder.frame().origin(),
        outer_cylinder.frame().z(),
    ) else {
        return false;
    };
    let clearance = Interval::point(outer_cylinder.radius())
        - Interval::point(inner_cylinder.radius())
        - Interval::point(2.0 * LINEAR_RESOLUTION);
    finite_interval(distance)
        && finite_interval(clearance)
        && clearance.lo() > 0.0
        && distance.hi() < clearance.square().lo()
}

fn points_are_exactly_axis_aligned(point: Point3, origin: Point3, axis: Vec3) -> bool {
    let cross_normals = [
        Vec3::new(0.0, axis.z, -axis.y),
        Vec3::new(-axis.z, 0.0, axis.x),
        Vec3::new(axis.y, -axis.x, 0.0),
    ];
    cross_normals.into_iter().all(|normal| {
        affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
            .is_some_and(|value| value.sign() == Orientation::Zero)
    })
}

fn interval_distance_squared(point: Point3, origin: Point3) -> Interval {
    point.to_array().into_iter().zip(origin.to_array()).fold(
        Interval::point(0.0),
        |sum, (point, origin)| {
            let point = Interval::point(point);
            let origin = Interval::point(origin);
            sum + point.square() - Interval::point(2.0) * point * origin + origin.square()
        },
    )
}

fn finite_interval(value: Interval) -> bool {
    value.lo().is_finite() && value.hi().is_finite()
}

/// Preserve both bands and both far caps; replace the cap pair by one exact
/// annulus on the larger disk's source plane.
fn prepare_internal_contact_shell(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
    outer: usize,
) -> Result<AnalyticShellInput, ContactPlanGap> {
    let inner = 1 - outer;
    let role_order = [outer, inner];
    let mut closed_edges = Vec::with_capacity(4);
    for (role, operand) in role_order.into_iter().enumerate() {
        let source = &sources[operand];
        closed_edges.push(source_closed_edge(
            store,
            AnalyticEdgeKey::new((2 * role) as u64),
            source.source.boundaries()[source.far_boundary],
        )?);
        closed_edges.push(source_closed_edge(
            store,
            AnalyticEdgeKey::new((2 * role + 1) as u64),
            source.source.boundaries()[source.contact_boundary],
        )?);
    }

    let mut faces = Vec::with_capacity(5);
    for (role, operand) in role_order.into_iter().enumerate() {
        let source = &sources[operand];
        let far_edge = AnalyticEdgeKey::new((2 * role) as u64);
        let contact_edge = AnalyticEdgeKey::new((2 * role + 1) as u64);
        faces.push(source_side_face(
            store,
            AnalyticFaceKey::new(role as u64),
            source,
            far_edge,
            contact_edge,
        )?);
    }
    for (role, operand) in role_order.into_iter().enumerate() {
        let source = &sources[operand];
        faces.push(source_cap_face(
            store,
            AnalyticFaceKey::new((2 + role) as u64),
            source.source.boundaries()[source.far_boundary],
            AnalyticEdgeKey::new((2 * role) as u64),
        )?);
    }

    let outer_source = &sources[outer];
    let inner_source = &sources[inner];
    let outer_boundary = outer_source.source.boundaries()[outer_source.contact_boundary];
    let inner_boundary = inner_source.source.boundaries()[inner_source.contact_boundary];
    let outer_cap = source_face_data(store, outer_boundary.cap_face())?;
    let plane = source_plane(store, outer_boundary.cap_face())?;
    let outer_use = source_pcurve(store, outer_boundary.cap_fin(), true)?;
    let inner_circle = source_circle(store, inner_boundary)?;
    let inner_use = projected_circle_pcurve(plane, inner_circle)?.with_closure_winding([0, 0]);
    let inner_side_sense = source_fin_sense(store, inner_boundary.side_fin())?;
    faces.push(
        AnalyticShellFace::new(
            AnalyticFaceKey::new(4),
            AnalyticShellSurface::Plane(plane),
            outer_cap.sense(),
            outer_cap.domain().ok_or(ContactPlanGap::SourceTopology)?,
            vec![
                AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                    AnalyticEdgeKey::new(1),
                    source_fin_sense(store, outer_boundary.cap_fin())?,
                    outer_use,
                )]),
                AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                    AnalyticEdgeKey::new(3),
                    inner_side_sense.flipped(),
                    inner_use,
                )]),
            ],
        )
        .with_source(EntityRef::Face(outer_boundary.cap_face())),
    );

    Ok(AnalyticShellInput::new(Vec::new(), Vec::new(), faces).with_closed_edges(closed_edges))
}

/// Coalesce two exactly coaxial touching bands into one band. The retained
/// side records an ordered binary Merge from both source sides; each physical
/// far cap and ring retains its own DerivedFrom lineage.
fn prepare_coincident_contact_shell(
    store: &Store,
    sources: &[ContactSource<'_>; 2],
) -> Result<AnalyticShellInput, ContactPlanGap> {
    let cylinder = sources[0].source.cylinder();
    let side_source = source_face_data(store, sources[0].source.side_face())?;
    if !side_source.sense().is_forward() {
        return Err(ContactPlanGap::SourceTopology);
    }
    let far = sources.map(|source| source.source.boundaries()[source.far_boundary]);
    let parameters = far.map(|boundary| axial_parameter(cylinder, boundary.center()));
    let order = match affine_dot3(
        cylinder.frame().z().to_array(),
        far[1].center().to_array(),
        far[0].center().to_array(),
        0.0,
    )
    .ok_or(ContactPlanGap::ArithmeticGuard)?
    .sign()
    {
        Orientation::Positive => [0, 1],
        Orientation::Negative => [1, 0],
        Orientation::Zero => return Err(ContactPlanGap::RelationBinding),
    };
    let low = order[0];
    let high = order[1];
    if !parameters[low].is_finite()
        || !parameters[high].is_finite()
        || parameters[low] >= parameters[high]
    {
        return Err(ContactPlanGap::ArithmeticGuard);
    }

    let common_cylinder = Cylinder::new(*cylinder.frame(), cylinder.radius())
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let circles = [
        Circle::new(
            cylinder.frame().with_origin(far[low].center()),
            cylinder.radius(),
        )
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?,
        Circle::new(
            cylinder.frame().with_origin(far[high].center()),
            cylinder.radius(),
        )
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?,
    ];
    let closed_edges = vec![
        AnalyticShellClosedEdge::new(
            AnalyticEdgeKey::new(0),
            AnalyticShellCurve::Circle(circles[0]),
            ParamRange::new(0.0, PERIOD),
        )
        .with_source(EntityRef::Edge(far[low].edge())),
        AnalyticShellClosedEdge::new(
            AnalyticEdgeKey::new(1),
            AnalyticShellCurve::Circle(circles[1]),
            ParamRange::new(0.0, PERIOD),
        )
        .with_source(EntityRef::Edge(far[high].edge())),
    ];
    let side_domain = FaceDomain::from_bounds(0.0, PERIOD, parameters[low], parameters[high])
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let side_faces = vec![
        AnalyticShellFace::new(
            AnalyticFaceKey::new(0),
            AnalyticShellSurface::Cylinder(common_cylinder),
            Sense::Forward,
            side_domain,
            vec![
                AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                    AnalyticEdgeKey::new(0),
                    Sense::Forward,
                    cylinder_ring_pcurve(parameters[low], true)?,
                )]),
                AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                    AnalyticEdgeKey::new(1),
                    Sense::Reversed,
                    cylinder_ring_pcurve(parameters[high], true)?,
                )]),
            ],
        )
        .with_merge_sources([
            EntityRef::Face(sources[0].source.side_face()),
            EntityRef::Face(sources[1].source.side_face()),
        ]),
        rebuilt_cap_face(
            store,
            AnalyticFaceKey::new(1),
            far[low],
            AnalyticEdgeKey::new(0),
            circles[0],
            Sense::Reversed,
        )?,
        rebuilt_cap_face(
            store,
            AnalyticFaceKey::new(2),
            far[high],
            AnalyticEdgeKey::new(1),
            circles[1],
            Sense::Forward,
        )?,
    ];
    Ok(AnalyticShellInput::new(Vec::new(), Vec::new(), side_faces).with_closed_edges(closed_edges))
}

fn source_closed_edge(
    store: &Store,
    key: AnalyticEdgeKey,
    boundary: CertifiedCylinderBoundary,
) -> Result<AnalyticShellClosedEdge, ContactPlanGap> {
    let circle = source_circle(store, boundary)?;
    Ok(AnalyticShellClosedEdge::new(
        key,
        AnalyticShellCurve::Circle(circle),
        circle.param_range(),
    )
    .with_source(EntityRef::Edge(boundary.edge())))
}

fn source_side_face(
    store: &Store,
    key: AnalyticFaceKey,
    source: &ContactSource<'_>,
    far_edge: AnalyticEdgeKey,
    contact_edge: AnalyticEdgeKey,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let face = source_face_data(store, source.source.side_face())?;
    let far = source.source.boundaries()[source.far_boundary];
    let contact = source.source.boundaries()[source.contact_boundary];
    Ok(AnalyticShellFace::new(
        key,
        AnalyticShellSurface::Cylinder(source.source.cylinder()),
        face.sense(),
        face.domain().ok_or(ContactPlanGap::SourceTopology)?,
        vec![
            AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                far_edge,
                source_fin_sense(store, far.side_fin())?,
                source_pcurve(store, far.side_fin(), true)?,
            )]),
            AnalyticShellLoop::new(vec![AnalyticShellFin::new(
                contact_edge,
                source_fin_sense(store, contact.side_fin())?,
                source_pcurve(store, contact.side_fin(), true)?,
            )]),
        ],
    )
    .with_source(EntityRef::Face(source.source.side_face())))
}

fn source_cap_face(
    store: &Store,
    key: AnalyticFaceKey,
    boundary: CertifiedCylinderBoundary,
    edge: AnalyticEdgeKey,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let face = source_face_data(store, boundary.cap_face())?;
    Ok(AnalyticShellFace::new(
        key,
        AnalyticShellSurface::Plane(source_plane(store, boundary.cap_face())?),
        face.sense(),
        face.domain().ok_or(ContactPlanGap::SourceTopology)?,
        vec![AnalyticShellLoop::new(vec![AnalyticShellFin::new(
            edge,
            source_fin_sense(store, boundary.cap_fin())?,
            source_pcurve(store, boundary.cap_fin(), true)?,
        )])],
    )
    .with_source(EntityRef::Face(boundary.cap_face())))
}

fn rebuilt_cap_face(
    store: &Store,
    key: AnalyticFaceKey,
    boundary: CertifiedCylinderBoundary,
    edge: AnalyticEdgeKey,
    circle: Circle,
    sense: Sense,
) -> Result<AnalyticShellFace, ContactPlanGap> {
    let face = source_face_data(store, boundary.cap_face())?;
    let plane = source_plane(store, boundary.cap_face())?;
    Ok(AnalyticShellFace::new(
        key,
        AnalyticShellSurface::Plane(plane),
        face.sense(),
        face.domain().ok_or(ContactPlanGap::SourceTopology)?,
        vec![AnalyticShellLoop::new(vec![AnalyticShellFin::new(
            edge,
            sense,
            projected_circle_pcurve(plane, circle)?.with_closure_winding([0, 0]),
        )])],
    )
    .with_source(EntityRef::Face(boundary.cap_face())))
}

fn cylinder_ring_pcurve(height: f64, closed: bool) -> Result<AnalyticPcurveUse, ContactPlanGap> {
    let line = Line2d::new(Point2::new(0.0, height), Vec2::new(1.0, 0.0))
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    let mut use_ = AnalyticPcurveUse::new(
        AnalyticShellPcurve::Line(line),
        AffineParamMap1d::new(1.0, 0.0).map_err(|_| ContactPlanGap::ArithmeticGuard)?,
    );
    if closed {
        use_ = use_.with_closure_winding([1, 0]);
    }
    Ok(use_)
}

fn projected_circle_pcurve(
    plane: Plane,
    circle: Circle,
) -> Result<AnalyticPcurveUse, ContactPlanGap> {
    let local = plane.frame().to_local(circle.frame().origin());
    let x = Vec2::new(
        circle.frame().x().dot(plane.frame().x()),
        circle.frame().x().dot(plane.frame().y()),
    );
    let axis_sign = affine_dot3(
        circle.frame().z().to_array(),
        plane.frame().z().to_array(),
        [0.0; 3],
        0.0,
    )
    .ok_or(ContactPlanGap::ArithmeticGuard)?
    .sign();
    let scale = match axis_sign {
        Orientation::Positive => 1.0,
        Orientation::Negative => -1.0,
        Orientation::Zero => return Err(ContactPlanGap::SourceTopology),
    };
    let pcurve = Circle2d::new(Point2::new(local.x, local.y), circle.radius(), x)
        .map_err(|_| ContactPlanGap::ArithmeticGuard)?;
    Ok(AnalyticPcurveUse::new(
        AnalyticShellPcurve::Circle(pcurve),
        AffineParamMap1d::new(scale, 0.0).map_err(|_| ContactPlanGap::ArithmeticGuard)?,
    ))
}

fn source_pcurve(
    store: &Store,
    fin: RawFinId,
    retain_winding: bool,
) -> Result<AnalyticPcurveUse, ContactPlanGap> {
    let use_ = store
        .get(fin)
        .map_err(|_| ContactPlanGap::SourceTopology)?
        .pcurve()
        .ok_or(ContactPlanGap::SourceTopology)?;
    let pcurve = match store
        .pcurve(use_.curve())
        .map_err(|_| ContactPlanGap::SourceTopology)?
    {
        Curve2dGeom::Line(line) => AnalyticShellPcurve::Line(*line),
        Curve2dGeom::Circle(circle) => AnalyticShellPcurve::Circle(*circle),
        _ => return Err(ContactPlanGap::SourceTopology),
    };
    let map = use_.edge_to_pcurve();
    let mut result = AnalyticPcurveUse::new(
        pcurve,
        AffineParamMap1d::new(map.scale(), map.offset())
            .map_err(|_| ContactPlanGap::ArithmeticGuard)?,
    )
    .with_chart(use_.chart());
    if retain_winding && let Some(winding) = use_.closure_winding() {
        result = result.with_closure_winding(winding);
    }
    Ok(result)
}

fn source_circle(
    store: &Store,
    boundary: CertifiedCylinderBoundary,
) -> Result<Circle, ContactPlanGap> {
    let edge = store
        .get(boundary.edge())
        .map_err(|_| ContactPlanGap::SourceTopology)?;
    let curve = edge.curve().ok_or(ContactPlanGap::SourceTopology)?;
    match store
        .curve(curve)
        .map_err(|_| ContactPlanGap::SourceTopology)?
    {
        CurveGeom::Circle(circle) => Ok(*circle),
        _ => Err(ContactPlanGap::SourceTopology),
    }
}

fn source_plane(store: &Store, face: RawFaceId) -> Result<Plane, ContactPlanGap> {
    let face = source_face_data(store, face)?;
    match store
        .surface(face.surface())
        .map_err(|_| ContactPlanGap::SourceTopology)?
    {
        SurfaceGeom::Plane(plane) => Ok(*plane),
        _ => Err(ContactPlanGap::SourceTopology),
    }
}

fn source_face_data<'a>(
    store: &'a Store,
    face: RawFaceId,
) -> Result<&'a ktopo::entity::Face, ContactPlanGap> {
    store.get(face).map_err(|_| ContactPlanGap::SourceTopology)
}

fn source_fin_sense(store: &Store, fin: RawFinId) -> Result<Sense, ContactPlanGap> {
    store
        .get(fin)
        .map(|fin| fin.sense())
        .map_err(|_| ContactPlanGap::SourceTopology)
}

fn axial_parameter(cylinder: Cylinder, point: Point3) -> f64 {
    (point - cylinder.frame().origin()).dot(cylinder.frame().z())
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;

    #[test]
    fn axis_distance_encloses_cross_product_oracle_for_ill_conditioned_frame() {
        let axis = Vec3::new(1.0, 1.0, 1.0).normalized().unwrap();
        let frame = Frame::new(
            Point3::new(0.0, 0.0, 0.0),
            axis,
            axis + Vec3::new(1.0e-6, -1.0e-6, 0.0),
        )
        .unwrap();
        let radial = Vec3::new(1.0, -1.0, 0.0).normalized().unwrap();
        let displacement = frame.z() * 400.0 + radial;
        let point = frame.origin() + displacement;

        let distance = interval_axis_distance_squared(point, frame.origin(), frame.z()).unwrap();
        let cross = displacement.cross(frame.z());
        let oracle = cross.norm_sq() / frame.z().norm_sq();
        let legacy_projection =
            displacement.dot(frame.x()).powi(2) + displacement.dot(frame.y()).powi(2);

        assert!(frame.x().dot(frame.z()).abs() > 1.0e-10);
        assert!(oracle - legacy_projection > 1.0e-8);
        assert!(distance.lo() <= oracle && oracle <= distance.hi());
    }
}
