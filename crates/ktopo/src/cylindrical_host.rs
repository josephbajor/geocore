//! Operation-neutral assembly of cylindrical bands attached to one planar host.
//!
//! A proposal contains one convex planar host and one or more finite analytic
//! cylinder bands. Each band endpoint is declared as either a host port or a
//! closing cap. Support incidence is exact for unbound planes and resolution-
//! bounded for structurally bound planes. It maps declarations onto geometric
//! endpoints and derives the only admissible winding:
//! one-port bands may point outward or inward, while two-port bands are
//! reversed void boundaries. Multiple bands are admitted only when their
//! cylinders are exactly coaxial and their closed axial intervals are
//! strictly separated. All host, band, containment, lineage, and separation
//! preflight completes before the body scaffold is allocated.

use std::collections::{BTreeMap, BTreeSet};

use crate::convex_containment::certify_convex_planar_input;
use crate::cylindrical_boss::{PreparedPlanarRingUse, PreparedRing};
use crate::entity::{BodyId, EdgeId, EntityRef, Face, FaceDomain, FaceId, Sense, ShellId};
use crate::geom::SurfaceGeom;
use crate::loop_proof::certify_convex_polygon_circle_containment;
use crate::planar::{PlanarSolidInput, PreparedSolid};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use kcore::interval::Interval;
use kcore::predicates::{Orientation, affine_dot3};
use kcore::tolerance::{LINEAR_RESOLUTION, check_in_size_box};
use kgeom::curve::{Circle, Curve};
use kgeom::curve2d::{Circle2d, Line2d};
use kgeom::frame::Frame;
use kgeom::param::ParamRange;
use kgeom::surface::{Cylinder, Plane};
use kgeom::vec::{Point2, Point3, Vec2, Vec3};

/// Semantic role of one finite-band endpoint.
///
/// Declarations are unordered. Unbound port incidence is exact; structurally
/// bound planes admit a conservative [`LINEAR_RESOLUTION`] envelope. A cap
/// occupies the remaining endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CylindricalHostEndpoint {
    /// Complete circular attachment to one convex host face.
    Port {
        /// Index into [`CylindricalHostSolidInput::host`]'s face list.
        host_face: usize,
    },
    /// Complete planar closing disk.
    Cap {
        /// Optional source face retained as semantic lineage.
        source: Option<FaceId>,
    },
}

impl CylindricalHostEndpoint {
    /// Declare a host port.
    pub const fn port(host_face: usize) -> Self {
        Self::Port { host_face }
    }

    /// Declare a cap without source lineage.
    pub const fn cap() -> Self {
        Self::Cap { source: None }
    }

    /// Declare a cap derived from a live source face.
    pub const fn cap_with_source(source: FaceId) -> Self {
        Self::Cap {
            source: Some(source),
        }
    }
}

/// One finite cylinder band attached to the common host.
#[derive(Debug, Clone, PartialEq)]
pub struct CylindricalHostBandInput {
    frame: Frame,
    radius: f64,
    axial_range: ParamRange,
    endpoints: [CylindricalHostEndpoint; 2],
    side_source: Option<FaceId>,
}

impl CylindricalHostBandInput {
    /// Describe one finite cylinder and its two unordered endpoint roles.
    pub const fn new(
        frame: Frame,
        radius: f64,
        axial_range: ParamRange,
        endpoints: [CylindricalHostEndpoint; 2],
    ) -> Self {
        Self {
            frame,
            radius,
            axial_range,
            endpoints,
            side_source: None,
        }
    }

    /// Attach the selected source of the cylindrical side face.
    pub const fn with_side_source(mut self, source: FaceId) -> Self {
        self.side_source = Some(source);
        self
    }

    /// Cylinder parameter frame.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Positive cylinder radius.
    pub const fn radius(&self) -> f64 {
        self.radius
    }

    /// Finite increasing cylinder interval.
    pub const fn axial_range(&self) -> ParamRange {
        self.axial_range
    }

    /// Unordered endpoint declarations.
    pub const fn endpoints(&self) -> [CylindricalHostEndpoint; 2] {
        self.endpoints
    }

    /// Optional source of the side face.
    pub const fn side_source(&self) -> Option<FaceId> {
        self.side_source
    }
}

/// One connected planar host plus all attached finite cylinder bands.
#[derive(Debug, Clone, PartialEq)]
pub struct CylindricalHostSolidInput {
    host: PlanarSolidInput,
    bands: Vec<CylindricalHostBandInput>,
}

impl CylindricalHostSolidInput {
    /// Construct a proposal. Complete semantic validation occurs on assembly.
    pub const fn new(host: PlanarSolidInput, bands: Vec<CylindricalHostBandInput>) -> Self {
        Self { host, bands }
    }

    /// Convex planar host proposal.
    pub const fn host(&self) -> &PlanarSolidInput {
        &self.host
    }

    /// Finite attached bands in caller order.
    pub fn bands(&self) -> &[CylindricalHostBandInput] {
        &self.bands
    }
}

/// Checked conservative accounting for generalized-host semantic preflight.
///
/// The individual fields expose every term that can grow faster than the
/// serialized output: the host face/vertex proof matrix, endpoint/support
/// sweep matrix, and complete band-pair separation matrix. `total()` also
/// includes port polygon uses and a fixed linear preparation allowance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CylindricalHostPreflightWork {
    host_vertex_face_pairs: u64,
    endpoint_support_pairs: u64,
    band_pairs: u64,
    port_boundary_uses: u64,
    band_linear_work: u64,
    total: u64,
}

impl CylindricalHostPreflightWork {
    /// Exact host-face/host-vertex pair count.
    pub const fn host_vertex_face_pairs(self) -> u64 {
        self.host_vertex_face_pairs
    }

    /// Conservative two-endpoint/host-support pair count over every band.
    pub const fn endpoint_support_pairs(self) -> u64 {
        self.endpoint_support_pairs
    }

    /// Complete unordered band-pair count.
    pub const fn band_pairs(self) -> u64 {
        self.band_pairs
    }

    /// Total polygon edge uses across all declared ports.
    pub const fn port_boundary_uses(self) -> u64 {
        self.port_boundary_uses
    }

    /// Fixed sixteen-unit preparation allowance per band.
    pub const fn band_linear_work(self) -> u64 {
        self.band_linear_work
    }

    /// Checked sum of every exposed preflight term.
    pub const fn total(self) -> u64 {
        self.total
    }
}

/// Return the exact/conservative generalized-host preflight accounting.
///
/// The formula is `F*V + 2*F*B + B*(B-1)/2 + P + 16*B`, where `F` is host
/// faces, `V` host vertices, `B` bands, and `P` total boundary uses of all
/// declared port faces. All arithmetic and index conversion is checked.
pub fn cylindrical_host_preflight_work(
    input: &CylindricalHostSolidInput,
) -> Result<CylindricalHostPreflightWork> {
    let mut port_boundary_uses = 0_usize;
    for endpoint in input.bands.iter().flat_map(|band| band.endpoints) {
        if let CylindricalHostEndpoint::Port { host_face } = endpoint {
            let face = input
                .host
                .faces()
                .get(host_face)
                .ok_or(Error::InvalidGeometry {
                    reason: "cylindrical-host port face index is invalid",
                })?;
            port_boundary_uses = port_boundary_uses
                .checked_add(face.vertices().len())
                .ok_or_else(work_overflow)?;
        }
    }
    cylindrical_host_dimension_work(
        input.host.faces().len(),
        input.host.vertices().len(),
        input.bands.len(),
        port_boundary_uses,
    )
}

/// Return generalized-host work from already-admitted source dimensions.
///
/// This allocation-free admission seam uses the same
/// `F*V + 2*F*B + B*(B-1)/2 + P + 16*B` formula as
/// [`cylindrical_host_preflight_work`]. `P` is the caller-certified total
/// boundary-use count across every declared port.
pub fn cylindrical_host_dimension_work(
    host_faces: usize,
    host_vertices: usize,
    band_count: usize,
    port_boundary_uses: usize,
) -> Result<CylindricalHostPreflightWork> {
    let faces = as_u64(host_faces)?;
    let vertices = as_u64(host_vertices)?;
    let bands = as_u64(band_count)?;
    let port_boundary_uses = as_u64(port_boundary_uses)?;
    let host_vertex_face_pairs = faces.checked_mul(vertices).ok_or_else(work_overflow)?;
    let endpoint_support_pairs = faces
        .checked_mul(bands)
        .and_then(|value| value.checked_mul(2))
        .ok_or_else(work_overflow)?;
    let band_pairs = bands
        .checked_mul(bands.saturating_sub(1))
        .and_then(|value| value.checked_div(2))
        .ok_or_else(work_overflow)?;
    let band_linear_work = bands.checked_mul(16).ok_or_else(work_overflow)?;
    let total = host_vertex_face_pairs
        .checked_add(endpoint_support_pairs)
        .and_then(|value| value.checked_add(band_pairs))
        .and_then(|value| value.checked_add(port_boundary_uses))
        .and_then(|value| value.checked_add(band_linear_work))
        .ok_or_else(work_overflow)?;
    Ok(CylindricalHostPreflightWork {
        host_vertex_face_pairs,
        endpoint_support_pairs,
        band_pairs,
        port_boundary_uses,
        band_linear_work,
        total,
    })
}

/// Realized role of one geometric low/high band endpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CylindricalHostEndpointOutput {
    /// Endpoint attached to an allocated host face.
    Port {
        /// Punctured host face.
        host_face: FaceId,
    },
    /// Endpoint closed by a newly allocated cap face.
    Cap {
        /// Closing planar disk.
        face: FaceId,
    },
}

/// Stable handles for one allocated finite band.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CylindricalHostBandOutput {
    side_face: FaceId,
    endpoints: [CylindricalHostEndpointOutput; 2],
    ring_edges: [EdgeId; 2],
}

impl CylindricalHostBandOutput {
    /// Cylindrical side face.
    pub const fn side_face(&self) -> FaceId {
        self.side_face
    }

    /// Realized endpoint roles in geometric `[low, high]` parameter order.
    pub const fn endpoints(&self) -> [CylindricalHostEndpointOutput; 2] {
        self.endpoints
    }

    /// Vertexless circle edges in geometric `[low, high]` parameter order.
    pub const fn ring_edges(&self) -> [EdgeId; 2] {
        self.ring_edges
    }
}

/// Stable handles for one generalized cylindrical-host assembly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CylindricalHostSolidOutput {
    body: BodyId,
    shell: ShellId,
    host_faces: Vec<FaceId>,
    bands: Vec<CylindricalHostBandOutput>,
}

impl CylindricalHostSolidOutput {
    /// Newly assembled solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Single positive connected boundary shell.
    pub const fn shell(&self) -> ShellId {
        self.shell
    }

    /// Planar host faces in input order.
    pub fn host_faces(&self) -> &[FaceId] {
        &self.host_faces
    }

    /// Allocated bands in canonical geometric order.
    pub fn bands(&self) -> &[CylindricalHostBandOutput] {
        &self.bands
    }
}

#[derive(Debug, Clone, Copy)]
enum PreparedEndpoint {
    Port {
        host_face: usize,
    },
    Cap {
        plane: Plane,
        domain: FaceDomain,
        source: Option<FaceId>,
    },
}

#[derive(Debug)]
struct PreparedBand {
    cylinder: Cylinder,
    side_domain: FaceDomain,
    side_sense: Sense,
    endpoints: [PreparedEndpoint; 2],
    centers: [Point3; 2],
    rings: [PreparedRing; 2],
    side_source: Option<FaceId>,
}

#[derive(Debug)]
struct PreparedCylindricalHost {
    host: PreparedSolid,
    bands: Vec<PreparedBand>,
}

impl PreparedCylindricalHost {
    fn new(input: &CylindricalHostSolidInput, store: &crate::store::Store) -> Result<Self> {
        if input.bands.is_empty() {
            return invalid("cylindrical-host assembly requires at least one band");
        }
        let _work = cylindrical_host_preflight_work(input)?;
        let host = PreparedSolid::new(&input.host, store)?;
        certify_convex_planar_input(&input.host, &host, store)?;

        let mut ports = BTreeSet::new();
        for endpoint in input.bands.iter().flat_map(|band| band.endpoints) {
            if let CylindricalHostEndpoint::Port { host_face } = endpoint
                && !ports.insert(host_face)
            {
                return invalid("cylindrical-host bands require globally distinct port faces");
            }
        }

        let positions = input
            .host
            .vertices()
            .iter()
            .map(|vertex| (vertex.key(), vertex.position()))
            .collect::<BTreeMap<_, _>>();
        let mut bands = Vec::with_capacity(input.bands.len());
        for band in &input.bands {
            bands.push(PreparedBand::new(
                band,
                &input.host,
                &host,
                &positions,
                store,
            )?);
        }
        certify_and_sort_bands(&mut bands)?;
        Ok(Self { host, bands })
    }
}

impl PreparedBand {
    fn new(
        input: &CylindricalHostBandInput,
        host_input: &PlanarSolidInput,
        host: &PreparedSolid,
        positions: &BTreeMap<crate::planar::PlanarVertexKey, Point3>,
        store: &crate::store::Store,
    ) -> Result<Self> {
        if !input.radius.is_finite() || input.radius <= 0.0 {
            return invalid("cylindrical-host band radius must be finite and positive");
        }
        if !input.axial_range.is_finite() || input.axial_range.lo >= input.axial_range.hi {
            return invalid("cylindrical-host band range must be finite and increasing");
        }
        for source in std::iter::once(input.side_source)
            .chain(input.endpoints.into_iter().map(|endpoint| match endpoint {
                CylindricalHostEndpoint::Port { .. } => None,
                CylindricalHostEndpoint::Cap { source } => source,
            }))
            .flatten()
        {
            if !store.contains(source) {
                return Err(Error::StaleHandle);
            }
        }

        check_in_size_box(input.frame.origin().to_array())?;
        let low_center = input.frame.origin() + input.frame.z() * input.axial_range.lo;
        let frame = input.frame.with_origin(low_center);
        let height = input.axial_range.hi - input.axial_range.lo;
        let high_center = frame.origin() + frame.z() * height;
        let centers = [low_center, high_center];
        for center in centers {
            check_in_size_box(center.to_array())?;
        }
        let cylinder = Cylinder::new(frame, input.radius)?;
        let circles = [
            Circle::new(frame, input.radius)?,
            Circle::new(frame.with_origin(high_center), input.radius)?,
        ];
        for circle in circles {
            preflight_circle_extent(circle)?;
        }

        let ports = input
            .endpoints
            .iter()
            .filter_map(|endpoint| match endpoint {
                CylindricalHostEndpoint::Port { host_face } => Some(*host_face),
                CylindricalHostEndpoint::Cap { .. } => None,
            })
            .collect::<Vec<_>>();
        let caps = input
            .endpoints
            .iter()
            .filter_map(|endpoint| match endpoint {
                CylindricalHostEndpoint::Port { .. } => None,
                CylindricalHostEndpoint::Cap { source } => Some(*source),
            })
            .collect::<Vec<_>>();
        if !matches!((ports.len(), caps.len()), (1, 1) | (2, 0)) {
            return invalid(
                "each cylindrical-host band requires one port and one cap or two ports",
            );
        }

        let mut endpoint_ports = [None, None];
        let mut planar_rings = [None, None];
        let mut endpoint_outward = [None, None];
        for host_face in ports {
            let Some((plane, sense)) = host.face_plane(host_face, store)? else {
                return invalid("cylindrical-host port face index is invalid");
            };
            let bound_support = host_input.faces()[host_face].plane_binding().is_some();
            if !axis_is_port_normal(plane.frame(), &frame, bound_support)? {
                return invalid(
                    "cylindrical-host band axis must be perpendicular to every port plane",
                );
            }
            let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
            let signs = centers.map(|center| exact_affine(outward, center, plane.frame().origin()));
            let [low, high] = signs;
            let [low, high] = [low?, high?];
            let exact_endpoint = match (low, high) {
                (Orientation::Zero, sign) if sign != Orientation::Zero => 0,
                (sign, Orientation::Zero) if sign != Orientation::Zero => 1,
                _ => usize::MAX,
            };
            let endpoint = if bound_support {
                resolution_port_endpoint(outward, plane.frame().origin(), centers)
            } else {
                (exact_endpoint < 2).then_some(exact_endpoint)
            }
            .ok_or(Error::InvalidGeometry {
                reason: "each cylindrical-host port must support exactly one band endpoint",
            })?;
            if endpoint_ports[endpoint].replace(host_face).is_some() {
                return invalid("cylindrical-host ports must map to distinct band endpoints");
            }
            let polygon = host_input.faces()[host_face]
                .vertices()
                .iter()
                .map(|key| {
                    positions
                        .get(key)
                        .copied()
                        .map(|point| frame_uv(plane.frame(), point))
                        .ok_or(Error::InvalidGeometry {
                            reason: "cylindrical-host port references an unknown host vertex",
                        })
                })
                .collect::<Result<Vec<_>>>()?;
            let planar = planar_ring_use(plane.frame(), circles[endpoint])?;
            if !certify_convex_polygon_circle_containment(&polygon, planar.curve) {
                return invalid(
                    "cylindrical-host ring must lie strictly inside its convex port face",
                );
            }
            planar_rings[endpoint] = Some(planar);
            endpoint_outward[endpoint] = Some(outward);
        }

        let (side_sense, endpoints) = if caps.len() == 1 {
            let port_endpoint = endpoint_ports
                .iter()
                .position(Option::is_some)
                .expect("one declared port maps to one endpoint");
            let cap_endpoint = 1 - port_endpoint;
            let outward = endpoint_outward[port_endpoint]
                .expect("mapped port retains its oriented support normal");
            let feature_direction =
                exact_affine(outward, centers[cap_endpoint], centers[port_endpoint])?;
            let side_sense = match feature_direction {
                Orientation::Positive => Sense::Forward,
                Orientation::Negative => Sense::Reversed,
                Orientation::Zero => {
                    return invalid("cylindrical-host capped band must have signed height");
                }
            };
            if side_sense == Sense::Reversed {
                certify_complete_sweep_containment(
                    host_input,
                    host,
                    store,
                    frame,
                    input.radius,
                    centers,
                    &[(port_endpoint, endpoint_ports[port_endpoint].unwrap())],
                )?;
            }
            let cap_frame = Frame::new(centers[cap_endpoint], outward, frame.x())?;
            let plane = Plane::new(cap_frame);
            let domain =
                FaceDomain::from_bounds(-input.radius, input.radius, -input.radius, input.radius)?;
            planar_rings[cap_endpoint] = Some(planar_ring_use(&cap_frame, circles[cap_endpoint])?);
            let mut endpoints = [None, None];
            endpoints[port_endpoint] = Some(PreparedEndpoint::Port {
                host_face: endpoint_ports[port_endpoint].unwrap(),
            });
            endpoints[cap_endpoint] = Some(PreparedEndpoint::Cap {
                plane,
                domain,
                source: caps[0],
            });
            (
                side_sense,
                endpoints
                    .map(|endpoint| endpoint.expect("one port and one cap fill both endpoints")),
            )
        } else {
            let port_faces = endpoint_ports
                .map(|port| port.expect("two declared ports map onto both geometric endpoints"));
            for (endpoint, outward) in endpoint_outward.into_iter().enumerate() {
                let outward =
                    outward.expect("two mapped ports retain both oriented support normals");
                let expected = if endpoint == 0 {
                    Orientation::Negative
                } else {
                    Orientation::Positive
                };
                if exact_dot(outward, frame.z())? != expected {
                    return invalid("two-port cylindrical-host supports must outwardly oppose");
                }
            }
            certify_complete_sweep_containment(
                host_input,
                host,
                store,
                frame,
                input.radius,
                centers,
                &[(0, port_faces[0]), (1, port_faces[1])],
            )?;
            (
                Sense::Reversed,
                port_faces.map(|host_face| PreparedEndpoint::Port { host_face }),
            )
        };

        let side_domain = FaceDomain::from_bounds(0.0, core::f64::consts::TAU, 0.0, height)?;
        let rings = core::array::from_fn(|endpoint| {
            let conventional = if endpoint == 0 {
                Sense::Forward
            } else {
                Sense::Reversed
            };
            PreparedRing {
                circle: circles[endpoint],
                side_sense: if side_sense == Sense::Forward {
                    conventional
                } else {
                    conventional.flipped()
                },
                side_pcurve: Line2d::new(
                    Point2::new(0.0, if endpoint == 0 { 0.0 } else { height }),
                    Vec2::new(1.0, 0.0),
                )
                .expect("finite validated cylinder height makes an exact pcurve line"),
                planar: planar_rings[endpoint]
                    .expect("every prepared endpoint has one planar ring use"),
            }
        });
        Ok(Self {
            cylinder,
            side_domain,
            side_sense,
            endpoints,
            centers,
            rings,
            side_source: input.side_source,
        })
    }
}

impl Transaction<'_> {
    /// Assemble one planar host and all exactly prepared bands into one shell.
    ///
    /// Complete semantic preflight and canonical band ordering precede the
    /// first topology allocation. The caller owns the eventual checked or Full
    /// commit.
    pub fn assemble_cylindrical_host_solid(
        &mut self,
        input: &CylindricalHostSolidInput,
    ) -> Result<CylindricalHostSolidOutput> {
        let prepared = PreparedCylindricalHost::new(input, self.store())?;
        let (body, shell) = crate::make::solid_body_scaffold(self.store_mut());
        let host = self.allocate_prepared_planar_shell(prepared.host, shell)?;
        let mut outputs = Vec::with_capacity(prepared.bands.len());
        for band in prepared.bands {
            let side_surface = self
                .store_mut()
                .insert_surface(SurfaceGeom::Cylinder(band.cylinder))?;
            let side_face = self.store_mut().add(Face {
                shell,
                loops: Vec::new(),
                surface: side_surface,
                sense: band.side_sense,
                domain: Some(band.side_domain),
                tolerance: None,
            });
            self.store_mut().get_mut(shell)?.faces.push(side_face);

            let mut endpoints = [None, None];
            let mut ring_edges = [None, None];
            let mut cap_lineage = Vec::new();
            for endpoint in 0..2 {
                match band.endpoints[endpoint] {
                    PreparedEndpoint::Port { host_face } => {
                        let face = host.faces[host_face];
                        ring_edges[endpoint] = Some(self.allocate_cylindrical_host_port_ring(
                            side_face,
                            face,
                            band.rings[endpoint],
                        )?);
                        endpoints[endpoint] =
                            Some(CylindricalHostEndpointOutput::Port { host_face: face });
                    }
                    PreparedEndpoint::Cap {
                        plane,
                        domain,
                        source,
                    } => {
                        let (face, edge) = self.allocate_cylindrical_host_cap_ring(
                            shell,
                            side_face,
                            band.rings[endpoint],
                            plane,
                            domain,
                        )?;
                        ring_edges[endpoint] = Some(edge);
                        endpoints[endpoint] = Some(CylindricalHostEndpointOutput::Cap { face });
                        if let Some(source) = source {
                            cap_lineage.push((face, source));
                        }
                    }
                }
            }
            if let Some(source) = band.side_source {
                self.record_derived_from(EntityRef::Face(side_face), EntityRef::Face(source));
            }
            for (face, source) in cap_lineage {
                self.record_derived_from(EntityRef::Face(face), EntityRef::Face(source));
            }
            outputs.push(CylindricalHostBandOutput {
                side_face,
                endpoints: endpoints.map(|endpoint| {
                    endpoint.expect("both prepared endpoints allocate stable output roles")
                }),
                ring_edges: ring_edges.map(|edge| {
                    edge.expect("both prepared endpoints allocate one complete ring edge")
                }),
            });
        }
        Ok(CylindricalHostSolidOutput {
            body,
            shell,
            host_faces: host.faces,
            bands: outputs,
        })
    }
}

#[allow(clippy::too_many_arguments)]
fn certify_complete_sweep_containment(
    input: &PlanarSolidInput,
    prepared: &PreparedSolid,
    store: &crate::store::Store,
    frame: Frame,
    radius: f64,
    centers: [Point3; 2],
    endpoint_ports: &[(usize, usize)],
) -> Result<()> {
    for index in 0..input.faces().len() {
        let (plane, sense) = prepared
            .face_plane(index, store)?
            .ok_or(Error::InvalidGeometry {
                reason: "cylindrical-host face disappeared during sweep preflight",
            })?;
        let outward = plane.frame().z() * if sense.is_forward() { 1.0 } else { -1.0 };
        if let Some((endpoint, _)) = endpoint_ports
            .iter()
            .find(|(_, host_face)| *host_face == index)
        {
            let other = 1 - *endpoint;
            let bound_support = input.faces()[index].plane_binding().is_some();
            if !(exact_affine(outward, centers[*endpoint], plane.frame().origin())?
                == Orientation::Zero
                || bound_support
                    && within_resolution(support_projection(
                        outward,
                        centers[*endpoint],
                        plane.frame().origin(),
                    )))
                || support_projection(outward, centers[other], plane.frame().origin()).hi()
                    >= -LINEAR_RESOLUTION
                || !axis_is_port_normal(plane.frame(), &frame, bound_support)?
            {
                return invalid("cylindrical-host inward sweep must enter every port support");
            }
            continue;
        }
        for center in centers {
            if exact_affine(outward, center, plane.frame().origin())? != Orientation::Negative
                || !certify_circle_inside_support(
                    outward,
                    plane.frame().origin(),
                    frame,
                    center,
                    radius,
                )
            {
                return invalid(
                    "complete cylindrical-host inward sweep must lie inside every non-port support",
                );
            }
        }
    }
    Ok(())
}

fn certify_and_sort_bands(bands: &mut [PreparedBand]) -> Result<()> {
    if bands.len() < 2 {
        return Ok(());
    }
    let axis = canonical_axis(bands[0].cylinder.frame().z());
    for first in 0..bands.len() {
        for second in first + 1..bands.len() {
            if !same_coaxial_cylinder(&bands[first], &bands[second])? {
                return invalid("multiple cylindrical-host bands must be exactly coaxial");
            }
            let first_range = canonical_centers(&bands[first], axis)?;
            let second_range = canonical_centers(&bands[second], axis)?;
            let first_below =
                exact_affine(axis, first_range[1], second_range[0])? == Orientation::Negative;
            let second_below =
                exact_affine(axis, second_range[1], first_range[0])? == Orientation::Negative;
            if !first_below && !second_below {
                return invalid(
                    "coaxial cylindrical-host band intervals must be strictly separated",
                );
            }
        }
    }
    for index in 1..bands.len() {
        let mut cursor = index;
        while cursor > 0 {
            let current = canonical_centers(&bands[cursor], axis)?[0];
            let prior = canonical_centers(&bands[cursor - 1], axis)?[0];
            match exact_affine(axis, current, prior)? {
                Orientation::Negative => bands.swap(cursor, cursor - 1),
                Orientation::Positive => break,
                Orientation::Zero => {
                    return invalid("cylindrical-host bands have ambiguous canonical order");
                }
            }
            cursor -= 1;
        }
    }
    Ok(())
}

fn same_coaxial_cylinder(first: &PreparedBand, second: &PreparedBand) -> Result<bool> {
    if first.cylinder.radius() != second.cylinder.radius() {
        return Ok(false);
    }
    let first_frame = first.cylinder.frame();
    let second_frame = second.cylinder.frame();
    Ok(
        exact_dot(first_frame.x(), second_frame.z())? == Orientation::Zero
            && exact_dot(first_frame.y(), second_frame.z())? == Orientation::Zero
            && exact_dot(first_frame.z(), second_frame.z())? != Orientation::Zero
            && exact_affine(first_frame.x(), second_frame.origin(), first_frame.origin())?
                == Orientation::Zero
            && exact_affine(first_frame.y(), second_frame.origin(), first_frame.origin())?
                == Orientation::Zero,
    )
}

fn canonical_centers(band: &PreparedBand, axis: Vec3) -> Result<[Point3; 2]> {
    match exact_affine(axis, band.centers[1], band.centers[0])? {
        Orientation::Positive => Ok(band.centers),
        Orientation::Negative => Ok([band.centers[1], band.centers[0]]),
        Orientation::Zero => invalid("cylindrical-host band endpoints are not axially ordered"),
    }
}

fn canonical_axis(axis: Vec3) -> Vec3 {
    let sign = [axis.x, axis.y, axis.z]
        .into_iter()
        .find(|component| *component != 0.0)
        .expect("validated frame axis is nonzero");
    if sign < 0.0 { axis * -1.0 } else { axis }
}

fn certify_circle_inside_support(
    outward: Vec3,
    support_origin: Point3,
    frame: Frame,
    center: Point3,
    radius: f64,
) -> bool {
    let signed = interval_dot(outward, center - support_origin);
    if signed.hi() >= 0.0 {
        return false;
    }
    let radius = Interval::point(radius);
    let radial_x = interval_dot(outward, frame.x()) * radius;
    let radial_y = interval_dot(outward, frame.y()) * radius;
    let radial_squared = radial_x.square() + radial_y.square();
    radial_squared.hi() < signed.square().lo()
}

fn planar_ring_use(frame: &Frame, circle: Circle) -> Result<PreparedPlanarRingUse> {
    let center = frame_uv(frame, circle.frame().origin());
    let x = frame_uv_vector(frame, circle.frame().x());
    let y = frame_uv_vector(frame, circle.frame().y());
    let curve = Circle2d::new(center, circle.radius(), x)?;
    let map = if y.dot(curve.x_dir().perp()) >= 0.0 {
        crate::entity::ParamMap1d::identity()
    } else {
        crate::entity::ParamMap1d::affine(-1.0, core::f64::consts::TAU)?
    };
    Ok(PreparedPlanarRingUse { curve, map })
}

fn frame_uv(frame: &Frame, point: Point3) -> Point2 {
    let offset = point - frame.origin();
    Point2::new(offset.dot(frame.x()), offset.dot(frame.y()))
}

fn frame_uv_vector(frame: &Frame, vector: Vec3) -> Vec2 {
    Vec2::new(vector.dot(frame.x()), vector.dot(frame.y()))
}

fn exact_dot(left: Vec3, right: Vec3) -> Result<Orientation> {
    affine_dot3(left.to_array(), right.to_array(), [0.0; 3], 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "cylindrical-host exact vector predicate is indeterminate",
        })
}

fn exact_affine(normal: Vec3, point: Point3, origin: Point3) -> Result<Orientation> {
    affine_dot3(normal.to_array(), point.to_array(), origin.to_array(), 0.0)
        .map(|value| value.sign())
        .ok_or(Error::InvalidGeometry {
            reason: "cylindrical-host exact affine predicate is indeterminate",
        })
}

fn axis_is_port_normal(plane: &Frame, cylinder: &Frame, bound_support: bool) -> Result<bool> {
    if exact_dot(plane.x(), cylinder.z())? == Orientation::Zero
        && exact_dot(plane.y(), cylinder.z())? == Orientation::Zero
    {
        return Ok(true);
    }
    if !bound_support {
        return Ok(false);
    }
    let shared_axis = plane.z() == cylinder.z() || plane.z() == cylinder.z() * -1.0;
    Ok(shared_axis
        || exact_dot(plane.z(), cylinder.x())? == Orientation::Zero
            && exact_dot(plane.z(), cylinder.y())? == Orientation::Zero)
}

fn resolution_port_endpoint(outward: Vec3, origin: Point3, centers: [Point3; 2]) -> Option<usize> {
    let projections = centers.map(|center| support_projection(outward, center, origin));
    let incident = projections.map(within_resolution);
    let endpoint = match incident {
        [true, false] => 0,
        [false, true] => 1,
        _ => return None,
    };
    let far = projections[1 - endpoint];
    (far.lo() > LINEAR_RESOLUTION || far.hi() < -LINEAR_RESOLUTION).then_some(endpoint)
}

fn support_projection(normal: Vec3, point: Point3, origin: Point3) -> Interval {
    interval_dot(normal, point - origin)
}

fn within_resolution(projection: Interval) -> bool {
    projection.lo() >= -LINEAR_RESOLUTION && projection.hi() <= LINEAR_RESOLUTION
}

fn interval_dot(left: Vec3, right: Vec3) -> Interval {
    Interval::point(left.x) * Interval::point(right.x)
        + Interval::point(left.y) * Interval::point(right.y)
        + Interval::point(left.z) * Interval::point(right.z)
}

fn preflight_circle_extent(circle: Circle) -> Result<()> {
    let bounds = circle.bounding_box(circle.param_range());
    check_in_size_box(bounds.min.to_array())?;
    check_in_size_box(bounds.max.to_array())?;
    Ok(())
}

fn as_u64(value: usize) -> Result<u64> {
    u64::try_from(value).map_err(|_| work_overflow())
}

fn work_overflow() -> Error {
    Error::InvalidGeometry {
        reason: "cylindrical-host preflight work count overflow",
    }
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::CheckOutcome;
    use crate::entity::{
        Body, Edge as RawEdge, Face as RawFace, Fin as RawFin, Loop as RawLoop, Region, Shell,
        Vertex as RawVertex,
    };
    use crate::planar::{
        PlanarFacePlaneBinding, PlanarSolidFace, PlanarSolidVertex, PlanarVertexKey,
    };
    use crate::store::Store;
    use crate::transaction::{FullCommitRequirement, LineageEvent};

    const CUBE_RINGS: [[usize; 4]; 6] = [
        [0, 2, 3, 1],
        [4, 5, 7, 6],
        [0, 1, 5, 4],
        [2, 6, 7, 3],
        [0, 4, 6, 2],
        [1, 3, 7, 5],
    ];

    fn cube(sources: Option<[FaceId; 6]>) -> PlanarSolidInput {
        cube_in_frame(Frame::world(), sources)
    }

    fn cube_in_frame(frame: Frame, sources: Option<[FaceId; 6]>) -> PlanarSolidInput {
        let points = [
            frame.point_at(-1.0, -1.0, -1.0),
            frame.point_at(1.0, -1.0, -1.0),
            frame.point_at(-1.0, 1.0, -1.0),
            frame.point_at(1.0, 1.0, -1.0),
            frame.point_at(-1.0, -1.0, 1.0),
            frame.point_at(1.0, -1.0, 1.0),
            frame.point_at(-1.0, 1.0, 1.0),
            frame.point_at(1.0, 1.0, 1.0),
        ];
        let keys = core::array::from_fn::<_, 8, _>(|index| PlanarVertexKey::new(index as u64));
        let vertices = keys
            .into_iter()
            .zip(points)
            .map(|(key, point)| PlanarSolidVertex::new(key, point))
            .collect();
        let faces = CUBE_RINGS
            .into_iter()
            .enumerate()
            .map(|(index, ring)| {
                let face = PlanarSolidFace::new(ring.map(|vertex| keys[vertex]).to_vec());
                sources.map_or(face.clone(), |sources| {
                    face.with_source(EntityRef::Face(sources[index]))
                })
            })
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    fn bound_cube_in_frame(store: &mut Store, frame: Frame) -> PlanarSolidInput {
        let source = crate::make::block(store, &frame, [2.0; 3]).unwrap();
        let surfaces = store
            .faces_of_body(source)
            .unwrap()
            .into_iter()
            .map(|face| store.get(face).unwrap().surface())
            .collect::<Vec<_>>();
        let input = cube_in_frame(frame, None);
        let faces = input
            .faces()
            .iter()
            .enumerate()
            .map(|(face_index, face)| {
                let carriers = (0..CUBE_RINGS[face_index].len())
                    .map(|edge_index| {
                        let a = CUBE_RINGS[face_index][edge_index];
                        let b =
                            CUBE_RINGS[face_index][(edge_index + 1) % CUBE_RINGS[face_index].len()];
                        let other = CUBE_RINGS
                            .iter()
                            .enumerate()
                            .find(|(candidate_index, candidate)| {
                                *candidate_index != face_index
                                    && candidate.contains(&a)
                                    && candidate.contains(&b)
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

    fn two_ended(
        host_sources: Option<[FaceId; 6]>,
        cylinder_sources: Option<[FaceId; 3]>,
    ) -> CylindricalHostSolidInput {
        let low = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::port(0),
                cylinder_sources.map_or_else(CylindricalHostEndpoint::cap, |sources| {
                    CylindricalHostEndpoint::cap_with_source(sources[1])
                }),
            ],
        );
        let high = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                cylinder_sources.map_or_else(CylindricalHostEndpoint::cap, |sources| {
                    CylindricalHostEndpoint::cap_with_source(sources[2])
                }),
                CylindricalHostEndpoint::port(1),
            ],
        );
        let [low, high] = match cylinder_sources {
            Some(sources) => [
                low.with_side_source(sources[0]),
                high.with_side_source(sources[0]),
            ],
            None => [low, high],
        };
        CylindricalHostSolidInput::new(cube(host_sources), vec![high, low])
    }

    fn topology_counts(store: &Store) -> [usize; 8] {
        [
            store.count::<Body>(),
            store.count::<Region>(),
            store.count::<Shell>(),
            store.count::<RawFace>(),
            store.count::<RawLoop>(),
            store.count::<RawFin>(),
            store.count::<RawEdge>(),
            store.count::<RawVertex>(),
        ]
    }

    #[test]
    fn dimension_work_matches_exact_formula_and_input_preflight() {
        let one_band = cylindrical_host_dimension_work(6, 8, 1, 4).unwrap();
        assert_eq!(
            [
                one_band.host_vertex_face_pairs(),
                one_band.endpoint_support_pairs(),
                one_band.band_pairs(),
                one_band.port_boundary_uses(),
                one_band.band_linear_work(),
                one_band.total(),
            ],
            [48, 12, 0, 4, 16, 80]
        );
        let two_bands = cylindrical_host_dimension_work(6, 8, 2, 8).unwrap();
        assert_eq!(
            [
                two_bands.host_vertex_face_pairs(),
                two_bands.endpoint_support_pairs(),
                two_bands.band_pairs(),
                two_bands.port_boundary_uses(),
                two_bands.band_linear_work(),
                two_bands.total(),
            ],
            [48, 24, 1, 8, 32, 113]
        );
        assert_eq!(
            cylindrical_host_preflight_work(&two_ended(None, None)).unwrap(),
            two_bands
        );
    }

    #[test]
    fn dimension_work_fails_closed_on_overflow() {
        for dimensions in [
            (usize::MAX, usize::MAX, 1, 0),
            (usize::MAX, 0, usize::MAX, 0),
        ] {
            assert!(
                cylindrical_host_dimension_work(
                    dimensions.0,
                    dimensions.1,
                    dimensions.2,
                    dimensions.3,
                )
                .is_err(),
                "dimensions {dimensions:?} must overflow"
            );
        }
    }

    #[test]
    fn two_outward_ends_share_one_host_and_allocate_in_geometric_order() {
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let input = two_ended(None, None);
        assert_eq!(
            cylindrical_host_preflight_work(&input).unwrap(),
            CylindricalHostPreflightWork {
                host_vertex_face_pairs: 48,
                endpoint_support_pairs: 24,
                band_pairs: 1,
                port_boundary_uses: 8,
                band_linear_work: 32,
                total: 113,
            }
        );
        let output = transaction.assemble_cylindrical_host_solid(&input).unwrap();
        assert_eq!(
            topology_counts(transaction.store()),
            [1, 2, 1, 10, 14, 32, 16, 8]
        );
        assert_eq!(output.host_faces().len(), 6);
        assert_eq!(output.bands().len(), 2);
        assert_eq!(
            output.bands()[0].endpoints(),
            [
                CylindricalHostEndpointOutput::Cap {
                    face: match output.bands()[0].endpoints()[0] {
                        CylindricalHostEndpointOutput::Cap { face } => face,
                        _ => panic!("low band must begin at its cap"),
                    },
                },
                CylindricalHostEndpointOutput::Port {
                    host_face: output.host_faces()[0],
                },
            ]
        );
        assert_eq!(
            output.bands()[1].endpoints()[0],
            CylindricalHostEndpointOutput::Port {
                host_face: output.host_faces()[1],
            }
        );
        assert!(matches!(
            output.bands()[1].endpoints()[1],
            CylindricalHostEndpointOutput::Cap { .. }
        ));
        let faces = transaction.store().faces_of_body(output.body()).unwrap();
        let mut loops = faces
            .iter()
            .map(|face| transaction.store().get(*face).unwrap().loops().len())
            .collect::<Vec<_>>();
        loops.sort_unstable();
        assert_eq!(loops, [1, 1, 1, 1, 1, 1, 2, 2, 2, 2]);
        let classes = faces.iter().fold((0, 0), |(planes, cylinders), face| {
            match transaction
                .store()
                .get(transaction.store().get(*face).unwrap().surface())
                .unwrap()
            {
                SurfaceGeom::Plane(_) => (planes + 1, cylinders),
                SurfaceGeom::Cylinder(_) => (planes, cylinders + 1),
                _ => (planes, cylinders),
            }
        });
        assert_eq!(classes, (8, 2));
        for band in output.bands() {
            assert_eq!(
                transaction.store().get(band.side_face()).unwrap().sense(),
                Sense::Forward
            );
            for edge in band.ring_edges() {
                let edge = transaction.store().get(edge).unwrap();
                assert_eq!(edge.vertices(), [None, None]);
                assert!(edge.bounds().is_none());
                assert_eq!(edge.fins().len(), 2);
            }
        }
    }

    #[test]
    fn exact_endpoint_incidence_derives_one_and_two_port_winding() {
        let cases = [
            (
                CylindricalHostSolidInput::new(
                    cube(None),
                    vec![CylindricalHostBandInput::new(
                        Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
                        0.5,
                        ParamRange::new(0.0, 1.0),
                        [
                            CylindricalHostEndpoint::cap(),
                            CylindricalHostEndpoint::port(1),
                        ],
                    )],
                ),
                Sense::Forward,
            ),
            (
                CylindricalHostSolidInput::new(
                    cube(None),
                    vec![CylindricalHostBandInput::new(
                        Frame::world(),
                        0.5,
                        ParamRange::new(0.0, 1.0),
                        [
                            CylindricalHostEndpoint::port(1),
                            CylindricalHostEndpoint::cap(),
                        ],
                    )],
                ),
                Sense::Reversed,
            ),
            (
                CylindricalHostSolidInput::new(
                    cube(None),
                    vec![CylindricalHostBandInput::new(
                        Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
                        0.5,
                        ParamRange::new(0.0, 2.0),
                        [
                            CylindricalHostEndpoint::port(1),
                            CylindricalHostEndpoint::port(0),
                        ],
                    )],
                ),
                Sense::Reversed,
            ),
        ];
        for (input, expected_sense) in cases {
            let mut store = Store::new();
            let mut transaction = store.transaction().unwrap();
            let output = transaction.assemble_cylindrical_host_solid(&input).unwrap();
            assert_eq!(output.bands().len(), 1);
            assert_eq!(
                transaction
                    .store()
                    .get(output.bands()[0].side_face())
                    .unwrap()
                    .sense(),
                expected_sense
            );
        }
    }

    #[test]
    fn off_center_oblique_one_port_host_is_full_valid() {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.0, 0.6, 0.8),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut store = Store::new();
        let band = CylindricalHostBandInput::new(
            frame.with_origin(frame.point_at(0.5, -0.25, 1.0)),
            0.25,
            ParamRange::new(0.0, 1.5),
            [
                CylindricalHostEndpoint::port(1),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let input =
            CylindricalHostSolidInput::new(bound_cube_in_frame(&mut store, frame), vec![band]);
        assert_eq!(cylindrical_host_preflight_work(&input).unwrap().total(), 80);

        let before = topology_counts(&store);
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_cylindrical_host_solid(&input).unwrap();
        let after = topology_counts(transaction.store());
        for (index, expected) in [1, 2, 1, 8, 10, 28, 14, 8].into_iter().enumerate() {
            assert_eq!(after[index] - before[index], expected);
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
    fn bound_port_beyond_resolution_envelope_is_rejected_atomically() {
        let frame = Frame::new(
            Point3::new(3.0, -2.0, 1.25),
            Vec3::new(0.0, 0.6, 0.8),
            Vec3::new(1.0, 0.0, 0.0),
        )
        .unwrap();
        let mut store = Store::new();
        let band = CylindricalHostBandInput::new(
            frame.with_origin(frame.point_at(0.5, -0.25, 1.0 + 4.0 * LINEAR_RESOLUTION)),
            0.25,
            ParamRange::new(0.0, 1.5),
            [
                CylindricalHostEndpoint::port(1),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let input =
            CylindricalHostSolidInput::new(bound_cube_in_frame(&mut store, frame), vec![band]);
        let before = topology_counts(&store);
        let mut transaction = store.transaction().unwrap();
        assert!(matches!(
            transaction.assemble_cylindrical_host_solid(&input),
            Err(Error::InvalidGeometry {
                reason: "each cylindrical-host port must support exactly one band endpoint"
            })
        ));
        assert_eq!(topology_counts(transaction.store()), before);
    }

    #[test]
    fn lineage_is_face_only_complete_and_canonical() {
        let mut store = Store::new();
        let host_source = crate::make::block(&mut store, &Frame::world(), [2.0; 3]).unwrap();
        let host_sources: [FaceId; 6] = store
            .faces_of_body(host_source)
            .unwrap()
            .try_into()
            .unwrap();
        let cylinder_source = crate::make::cylinder(
            &mut store,
            &Frame::world().with_origin(Point3::new(0.0, 0.0, -2.0)),
            0.5,
            4.0,
        )
        .unwrap();
        let cylinder_sources: [FaceId; 3] = store
            .faces_of_body(cylinder_source)
            .unwrap()
            .try_into()
            .unwrap();
        let input = two_ended(Some(host_sources), Some(cylinder_sources));
        let mut transaction = store.transaction().unwrap();
        let output = transaction.assemble_cylindrical_host_solid(&input).unwrap();
        let result_faces = transaction.store().faces_of_body(output.body()).unwrap();
        let journal = transaction.commit_checked_body(output.body()).unwrap();
        assert_eq!(journal.lineage().len(), 10);
        let expected_sources = host_sources
            .into_iter()
            .chain([
                cylinder_sources[0],
                cylinder_sources[1],
                cylinder_sources[0],
                cylinder_sources[2],
            ])
            .collect::<Vec<_>>();
        for (event, expected_source) in journal.lineage().iter().zip(expected_sources) {
            let LineageEvent::DerivedFrom {
                derived: EntityRef::Face(derived),
                source: EntityRef::Face(source),
            } = event
            else {
                panic!("cylindrical-host lineage must remain face-only: {event:?}");
            };
            assert!(result_faces.contains(derived));
            assert_eq!(*source, expected_source);
        }
    }

    #[test]
    fn duplicate_ports_and_touching_or_overlapping_bands_fail_before_allocation() {
        let top_pocket = CylindricalHostBandInput::new(
            Frame::world(),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::port(1),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let touching_bottom = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 1.0),
            [
                CylindricalHostEndpoint::cap(),
                CylindricalHostEndpoint::port(0),
            ],
        );
        let overlapping_bottom = CylindricalHostBandInput::new(
            Frame::world().with_origin(Point3::new(0.0, 0.0, -1.0)),
            0.5,
            ParamRange::new(0.0, 1.5),
            [
                CylindricalHostEndpoint::port(0),
                CylindricalHostEndpoint::cap(),
            ],
        );
        let duplicate = CylindricalHostSolidInput::new(
            cube(None),
            vec![top_pocket.clone(), top_pocket.clone()],
        );
        let touching =
            CylindricalHostSolidInput::new(cube(None), vec![top_pocket.clone(), touching_bottom]);
        let overlapping =
            CylindricalHostSolidInput::new(cube(None), vec![top_pocket, overlapping_bottom]);

        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let before = topology_counts(transaction.store());
        for proposal in [duplicate, touching, overlapping] {
            assert!(matches!(
                transaction.assemble_cylindrical_host_solid(&proposal),
                Err(Error::InvalidGeometry { .. })
            ));
            assert_eq!(topology_counts(transaction.store()), before);
        }
        let valid = transaction
            .assemble_cylindrical_host_solid(&two_ended(None, None))
            .unwrap();
        assert_eq!(valid.bands().len(), 2);
    }
}
