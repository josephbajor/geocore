//! The B-rep entity hierarchy, mirroring the Parasolid/XT topology model.
//!
//! ```text
//! BODY → REGION → SHELL → FACE → LOOP → FIN → EDGE → VERTEX
//! ```
//!
//! Entities are plain data stored in [`crate::store::Store`] arenas and
//! reference each other by typed handles. Invariants (enforced by
//! [`crate::check`], maintained by [`crate::euler`]):
//!
//! - Child/parent back-pointers agree (`face.shell` ↔ `shell.faces`, …).
//! - A [`Loop`]'s fins form a closed ring: fin `i`'s head vertex is fin
//!   `i+1`'s tail vertex (wrapping).
//! - **Loop orientation**: walking a loop's fins with the *face normal* up
//!   (surface normal composed with `face.sense`), the face interior lies to
//!   the left — outer loops counterclockwise, holes clockwise.
//! - **Face orientation**: the face normal points *away from the material*
//!   of the region owning its shell (outward for the outer shell of a
//!   solid).
//! - In a manifold solid, every edge has exactly two fins with opposite
//!   traversal directions along the edge.
//! - A face on a *closed* surface (sphere, torus) may have **zero loops**,
//!   meaning it covers the entire surface.
//! - An [`Edge`] with `bounds: None` is a *ring edge*: it spans one full
//!   period of a closed curve and has no vertices.

use crate::geom::SurfaceGeom;
use crate::tolerance::EntityTolerance;
use kcore::arena::Handle;
use kcore::error::{Error, Result};
use kgeom::curve2d::Curve2d;
use kgeom::param::ParamRange;
use kgeom::vec::{Point2, Point3};

/// Handle to a [`Body`].
pub type BodyId = Handle<Body>;
/// Handle to a [`Region`].
pub type RegionId = Handle<Region>;
/// Handle to a [`Shell`].
pub type ShellId = Handle<Shell>;
/// Handle to a [`Face`].
pub type FaceId = Handle<Face>;
/// Handle to a [`Loop`].
pub type LoopId = Handle<Loop>;
/// Handle to a [`Fin`].
pub type FinId = Handle<Fin>;
/// Handle to an [`Edge`].
pub type EdgeId = Handle<Edge>;
/// Handle to a [`Vertex`].
pub type VertexId = Handle<Vertex>;
/// Handle to an attached curve node in the geometry graph.
pub type CurveId = kgraph::CurveHandle;
/// Handle to an attached surface node in the geometry graph.
pub type SurfaceId = kgraph::SurfaceHandle;
/// Handle to an attached point.
pub type PointId = Handle<Point3>;
/// Handle to an attached parameter-space curve node in the geometry graph.
pub type Curve2dId = kgraph::Curve2dHandle;

/// Relative orientation of one entity's direction against another's
/// (fin vs. edge, face vs. surface normal).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Sense {
    /// Same direction.
    Forward,
    /// Opposite direction.
    Reversed,
}

impl Sense {
    /// The opposite sense.
    pub fn flipped(self) -> Sense {
        match self {
            Sense::Forward => Sense::Reversed,
            Sense::Reversed => Sense::Forward,
        }
    }

    /// True if `Forward`.
    pub fn is_forward(self) -> bool {
        self == Sense::Forward
    }

    /// Composition: two reversals cancel.
    pub fn times(self, other: Sense) -> Sense {
        if self == other {
            Sense::Forward
        } else {
            Sense::Reversed
        }
    }
}

/// What a body's point-set is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyKind {
    /// A closed 3-manifold-with-boundary: watertight shells enclosing
    /// material.
    Solid,
    /// A 2D point-set: faces not enclosing material.
    Sheet,
    /// A 1D point-set: edges and vertices only.
    Wire,
    /// A single vertex ("minimal" body).
    Acorn,
}

/// Whether a region contains material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegionKind {
    /// Material.
    Solid,
    /// Empty space (every body has an infinite void exterior region,
    /// stored first).
    Void,
}

/// The root topological entity: a connected point-set with its containing
/// space, partitioned into regions.
#[derive(Debug, Clone, PartialEq)]
pub struct Body {
    /// What the body's point-set is.
    pub kind: BodyKind,
    /// The regions partitioning space; `regions[0]` is the infinite void
    /// exterior.
    pub regions: Vec<RegionId>,
}

impl Body {
    /// What this body's point-set is.
    pub const fn kind(&self) -> BodyKind {
        self.kind
    }

    /// Regions partitioning space, in stored ownership order.
    pub fn regions(&self) -> &[RegionId] {
        &self.regions
    }
}

/// A connected open subset of space, bounded by shells.
#[derive(Debug, Clone, PartialEq)]
pub struct Region {
    /// Owning body.
    pub body: BodyId,
    /// Solid (material) or void.
    pub kind: RegionKind,
    /// Shells bounding this region.
    pub shells: Vec<ShellId>,
}

impl Region {
    /// Body that owns this region.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Whether this region contains material or void space.
    pub const fn kind(&self) -> RegionKind {
        self.kind
    }

    /// Shells bounding this region, in stored ownership order.
    pub fn shells(&self) -> &[ShellId] {
        &self.shells
    }
}

/// A connected boundary component of a region.
#[derive(Debug, Clone, PartialEq)]
pub struct Shell {
    /// Owning region.
    pub region: RegionId,
    /// Faces of the shell. Face normals point away from the owning
    /// region's material (see module docs).
    pub faces: Vec<FaceId>,
    /// Wireframe edges attached to the shell (wire bodies and mixed-
    /// dimension parts); empty for pure sheet/solid shells.
    pub edges: Vec<EdgeId>,
    /// The lone vertex of an acorn body's shell, if any.
    pub vertex: Option<VertexId>,
}

impl Shell {
    /// Region that owns this shell.
    pub const fn region(&self) -> RegionId {
        self.region
    }

    /// Faces belonging to this shell, in stored ownership order.
    pub fn faces(&self) -> &[FaceId] {
        &self.faces
    }

    /// Wireframe edges belonging to this shell, in stored ownership order.
    pub fn edges(&self) -> &[EdgeId] {
        &self.edges
    }

    /// Lone vertex of an acorn shell, when present.
    pub const fn vertex(&self) -> Option<VertexId> {
        self.vertex
    }
}

/// A finite conservative parameter-space work box for one face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceDomain {
    /// Conservative finite range in the surface's first parameter.
    pub u: ParamRange,
    /// Conservative finite range in the surface's second parameter.
    pub v: ParamRange,
}

impl FaceDomain {
    /// Construct a finite, positive-area parameter-space work box.
    pub fn new(u: ParamRange, v: ParamRange) -> Result<Self> {
        let (u_width, v_width) = (u.width(), v.width());
        if !u.is_finite()
            || !v.is_finite()
            || !u_width.is_finite()
            || !v_width.is_finite()
            || u_width <= 0.0
            || v_width <= 0.0
        {
            return Err(Error::InvalidGeometry {
                reason: "face domain ranges must be finite and increasing",
            });
        }
        Ok(Self { u, v })
    }

    /// Construct from `(u_min, u_max, v_min, v_max)` without panicking on
    /// untrusted values.
    pub fn from_bounds(u_min: f64, u_max: f64, v_min: f64, v_max: f64) -> Result<Self> {
        if [u_min, u_max, v_min, v_max]
            .iter()
            .any(|value| value.is_nan())
            || u_min > u_max
            || v_min > v_max
        {
            return Err(Error::InvalidGeometry {
                reason: "face domain bounds are invalid",
            });
        }
        Self::new(ParamRange::new(u_min, u_max), ParamRange::new(v_min, v_max))
    }

    /// The surface's natural parameter box when both ranges are finite.
    /// Unbounded analytic surfaces deliberately return `None` rather than
    /// inventing a trim domain from samples.
    pub fn natural(surface: &SurfaceGeom) -> Option<Self> {
        let [u, v] = surface.as_leaf_surface()?.param_range();
        Self::new(u, v).ok()
    }

    /// Smallest domain containing both inputs; errors if combining the
    /// finite ranges would overflow.
    pub fn union(self, other: Self) -> Result<Self> {
        Self::from_bounds(
            self.u.lo.min(other.u.lo),
            self.u.hi.max(other.u.hi),
            self.v.lo.min(other.v.lo),
            self.v.hi.max(other.v.hi),
        )
    }

    /// Whether a parameter lies in this conservative box.
    pub fn contains(self, uv: [f64; 2]) -> bool {
        self.u.contains(uv[0]) && self.v.contains(uv[1])
    }
}

/// A bounded, connected subset of one surface.
#[derive(Debug, Clone, PartialEq)]
pub struct Face {
    /// Owning shell.
    pub shell: ShellId,
    /// Bounding loops. Zero loops ⇔ the face covers a closed surface.
    /// When meaningful, the outer loop is stored first.
    pub loops: Vec<LoopId>,
    /// Supporting surface geometry.
    pub surface: SurfaceId,
    /// Face normal vs. surface normal: `Forward` means they agree.
    pub sense: Sense,
    /// Finite conservative UV work box. `None` means the domain is not yet
    /// known; consumers must not replace it with sampled guesses.
    pub domain: Option<FaceDomain>,
    /// Optional validated imported/operation tolerance with retained origin
    /// and growth provenance. The published XT FACE field is normally null;
    /// unlike tolerant edges/vertices, this does not change the face's
    /// exact/tolerant classification.
    pub tolerance: Option<EntityTolerance>,
}

impl Face {
    /// Shell that owns this face.
    pub const fn shell(&self) -> ShellId {
        self.shell
    }

    /// Boundary loops in stored topological order.
    pub fn loops(&self) -> &[LoopId] {
        &self.loops
    }

    /// Supporting surface geometry.
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    /// Face-normal orientation relative to the supporting surface normal.
    pub const fn sense(&self) -> Sense {
        self.sense
    }

    /// Finite conservative parameter-space work box, when known.
    pub const fn domain(&self) -> Option<FaceDomain> {
        self.domain
    }

    /// Imported or operation tolerance, when present.
    pub const fn tolerance(&self) -> Option<EntityTolerance> {
        self.tolerance
    }
}

/// A closed ring of fins bounding a face.
#[derive(Debug, Clone, PartialEq)]
pub struct Loop {
    /// Owning face.
    pub face: FaceId,
    /// Fins in traversal order (see module docs for orientation).
    pub fins: Vec<FinId>,
}

impl Loop {
    /// Face that owns this loop.
    pub const fn face(&self) -> FaceId {
        self.face
    }

    /// Fins in loop-traversal order.
    pub fn fins(&self) -> &[FinId] {
        &self.fins
    }
}

/// A validated affine correspondence from a 3D edge parameter `t` to a
/// pcurve parameter `q = scale * t + offset`.
///
/// The nonzero scale makes the map invertible. Its sign is the explicit
/// orientation of pcurve parameterization relative to edge parameterization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamMap1d {
    scale: f64,
    offset: f64,
}

/// Integer branch selection for a pcurve on a periodic surface.
///
/// Pcurve geometry remains reusable in its authored coordinates. For each
/// periodic surface direction, the corresponding integer adds whole periods
/// before the UV is consumed. A nonzero shift in a non-periodic direction is
/// invalid and is rejected by checked incidence consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PcurveChart {
    period_shifts: [i32; 2],
}

/// Topological meaning of one pcurve endpoint in increasing edge-parameter
/// direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PcurveEndpointKind {
    /// An ordinary endpoint where the supporting surface is regular.
    #[default]
    Regular,
    /// The endpoint lies on a degenerate surface iso-line (sphere pole,
    /// cone apex, or an equivalent procedural singularity).
    SurfaceSingularity,
}

/// One of the two surface-parameter directions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SurfaceParameter {
    /// First surface parameter (`u`).
    U,
    /// Second surface parameter (`v`).
    V,
}

/// Which boundary of a full-period face chart represents a seam use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SeamSide {
    /// Lower bound of the face domain in the seam direction.
    Lower,
    /// Upper bound of the face domain in the seam direction.
    Upper,
}

/// Explicit role of a pcurve that lies on a periodic face-chart cut.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PcurveSeam {
    direction: SurfaceParameter,
    side: SeamSide,
}

impl PcurveSeam {
    /// Declare a lower/upper seam use in one surface direction.
    pub const fn new(direction: SurfaceParameter, side: SeamSide) -> Self {
        Self { direction, side }
    }

    /// Periodic surface direction containing the chart cut.
    pub const fn direction(self) -> SurfaceParameter {
        self.direction
    }

    /// Lower or upper boundary of the face chart.
    pub const fn side(self) -> SeamSide {
        self.side
    }
}

impl PcurveChart {
    /// The authored pcurve branch, with no period translation.
    pub const fn identity() -> Self {
        Self {
            period_shifts: [0, 0],
        }
    }

    /// Select branches by integer period shifts in surface `(u, v)`.
    pub const fn shifted(period_shifts: [i32; 2]) -> Self {
        Self { period_shifts }
    }

    /// Integer period shifts in surface `(u, v)`.
    pub const fn period_shifts(self) -> [i32; 2] {
        self.period_shifts
    }

    /// Whether the authored pcurve branch is used unchanged.
    pub const fn is_identity(self) -> bool {
        self.period_shifts[0] == 0 && self.period_shifts[1] == 0
    }

    /// Translate `uv` onto this chart using the surface periods.
    pub fn apply(self, mut uv: Point2, periods: [Option<f64>; 2]) -> Result<Point2> {
        for (direction, period) in periods.into_iter().enumerate() {
            let shift = self.period_shifts[direction];
            if shift == 0 {
                continue;
            }
            let Some(period) = period else {
                return Err(Error::InvalidGeometry {
                    reason: "pcurve chart shifts a non-periodic surface direction",
                });
            };
            if !period.is_finite() || period <= 0.0 {
                return Err(Error::InvalidGeometry {
                    reason: "pcurve chart references an invalid surface period",
                });
            }
            let delta = f64::from(shift) * period;
            if direction == 0 {
                uv.x += delta;
            } else {
                uv.y += delta;
            }
        }
        if !uv.x.is_finite() || !uv.y.is_finite() {
            return Err(Error::InvalidGeometry {
                reason: "pcurve chart produced non-finite surface parameters",
            });
        }
        Ok(uv)
    }
}

impl ParamMap1d {
    /// Identity correspondence.
    pub const fn identity() -> Self {
        Self {
            scale: 1.0,
            offset: 0.0,
        }
    }

    /// Construct an invertible affine correspondence.
    pub fn affine(scale: f64, offset: f64) -> Result<Self> {
        if !scale.is_finite() || scale == 0.0 || !offset.is_finite() {
            return Err(Error::InvalidGeometry {
                reason: "pcurve parameter map must be finite and invertible",
            });
        }
        Ok(Self { scale, offset })
    }

    /// Map an edge parameter to the pcurve parameter.
    pub fn map(self, edge_parameter: f64) -> f64 {
        self.scale * edge_parameter + self.offset
    }

    /// Map a pcurve parameter back to the edge parameter.
    pub fn inverse(self, pcurve_parameter: f64) -> f64 {
        (pcurve_parameter - self.offset) / self.scale
    }

    /// Scale of the affine map.
    pub fn scale(self) -> f64 {
        self.scale
    }

    /// Offset of the affine map.
    pub fn offset(self) -> f64 {
        self.offset
    }

    /// Pcurve parameter direction relative to increasing edge parameter.
    pub fn sense(self) -> Sense {
        if self.scale > 0.0 {
            Sense::Forward
        } else {
            Sense::Reversed
        }
    }
}

/// One fin's use of a curve in its owning face's parameter space.
///
/// Different fins of the same edge deliberately carry independent pcurves:
/// seam uses and the two sides of an intersection edge generally have
/// different `(u, v)` representations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinPcurve {
    curve: Curve2dId,
    range: ParamRange,
    edge_to_pcurve: ParamMap1d,
    chart: PcurveChart,
    endpoint_kinds: [PcurveEndpointKind; 2],
    closure_winding: Option<[i32; 2]>,
    seam: Option<PcurveSeam>,
}

impl FinPcurve {
    /// Construct a pcurve use over a finite, increasing curve range.
    pub fn new(curve: Curve2dId, range: ParamRange, edge_to_pcurve: ParamMap1d) -> Result<Self> {
        if !range.is_finite() || range.lo >= range.hi {
            return Err(Error::InvalidGeometry {
                reason: "pcurve use range must be finite and increasing",
            });
        }
        Ok(Self {
            curve,
            range,
            edge_to_pcurve,
            chart: PcurveChart::identity(),
            endpoint_kinds: [PcurveEndpointKind::Regular; 2],
            closure_winding: None,
            seam: None,
        })
    }

    /// Parameter-space curve handle.
    pub fn curve(self) -> Curve2dId {
        self.curve
    }

    /// Active parameter range on the pcurve.
    pub fn range(self) -> ParamRange {
        self.range
    }

    /// Edge-to-pcurve parameter correspondence.
    pub fn edge_to_pcurve(self) -> ParamMap1d {
        self.edge_to_pcurve
    }

    /// Select an explicit integer-period chart for this fin use.
    ///
    /// Validation that nonzero shifts correspond to periodic surface
    /// directions occurs when the use is attached to a face.
    pub const fn with_chart(mut self, chart: PcurveChart) -> Self {
        self.chart = chart;
        self
    }

    /// Explicit periodic chart selection for this fin use.
    pub const fn chart(self) -> PcurveChart {
        self.chart
    }

    /// Mark endpoint semantics in increasing edge-parameter direction.
    pub const fn with_endpoint_kinds(mut self, endpoint_kinds: [PcurveEndpointKind; 2]) -> Self {
        self.endpoint_kinds = endpoint_kinds;
        self
    }

    /// Endpoint semantics in increasing edge-parameter direction.
    pub const fn endpoint_kinds(self) -> [PcurveEndpointKind; 2] {
        self.endpoint_kinds
    }

    /// Declare the whole-period displacement of a closed pcurve use in
    /// increasing edge-parameter direction.
    ///
    /// This metadata is meaningful only for a ring edge or an edge whose
    /// start and end vertex are the same. Checked incidence rejects it on an
    /// open edge.
    pub const fn with_closure_winding(mut self, winding: [i32; 2]) -> Self {
        self.closure_winding = Some(winding);
        self
    }

    /// Declared whole-period displacement of a closed use, if explicit.
    pub const fn closure_winding(self) -> Option<[i32; 2]> {
        self.closure_winding
    }

    /// Declare this use to lie on one side of a periodic face-chart seam.
    pub const fn with_seam(mut self, seam: PcurveSeam) -> Self {
        self.seam = Some(seam);
        self
    }

    /// Remove an explicit seam role while preserving other use metadata.
    pub const fn without_seam(mut self) -> Self {
        self.seam = None;
        self
    }

    /// Explicit periodic seam role, if any.
    pub const fn seam(self) -> Option<PcurveSeam> {
        self.seam
    }

    /// Pcurve parameter direction relative to the edge direction.
    pub fn sense(self) -> Sense {
        self.edge_to_pcurve.sense()
    }

    /// Evaluate the pcurve parameter corresponding to an edge parameter.
    pub fn parameter_at_edge(self, edge_parameter: f64) -> f64 {
        self.edge_to_pcurve.map(edge_parameter)
    }

    /// Evaluate this use in its selected surface chart.
    pub fn evaluate_uv(
        self,
        curve: &dyn Curve2d,
        edge_parameter: f64,
        periods: [Option<f64>; 2],
    ) -> Result<Point2> {
        self.chart
            .apply(curve.eval(self.parameter_at_edge(edge_parameter)), periods)
    }
}

/// One face's oriented use of an edge (a half-edge / co-edge).
#[derive(Debug, Clone, PartialEq)]
pub struct Fin {
    /// Owning loop.
    pub parent: LoopId,
    /// The edge being traversed.
    pub edge: EdgeId,
    /// Traversal direction vs. the edge's own direction (which is the
    /// direction of its curve).
    pub sense: Sense,
    /// This edge use's curve in the owning face's `(u, v)` space, including
    /// its active range and parameter correspondence. `None` is retained
    /// temporarily for legacy/imported topology while M2.5 migration is in
    /// progress; boolean-ready topology requires `Some`.
    pub pcurve: Option<FinPcurve>,
}

impl Fin {
    /// Loop that owns this fin.
    pub const fn parent(&self) -> LoopId {
        self.parent
    }

    /// Edge traversed by this fin.
    pub const fn edge(&self) -> EdgeId {
        self.edge
    }

    /// Traversal direction relative to the edge direction.
    pub const fn sense(&self) -> Sense {
        self.sense
    }

    /// Parameter-space curve use attached to this fin, when authored.
    pub const fn pcurve(&self) -> Option<FinPcurve> {
        self.pcurve
    }
}

/// A bounded, connected subset of one curve.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    /// Supporting curve geometry; its parameterization direction *is* the
    /// edge direction. `None` denotes a tolerant edge whose 3D realization
    /// is defined by its fins' pcurves lifted through their face surfaces.
    pub curve: Option<CurveId>,
    /// Start and end vertices in curve direction. `[Some, Some]` for an
    /// ordinary edge (equal handles ⇔ closed edge through one vertex),
    /// `[None, None]` for a ring edge.
    pub vertices: [Option<VertexId>; 2],
    /// Edge-parameter interval `(t0, t1)`, `t0 < t1`, matching the vertices.
    /// For an exact edge this is its 3D curve parameter (possibly unwrapped
    /// across a periodic seam). For a curve-less tolerant edge it is a
    /// logical correspondence domain, conventionally `[0, 1]`, mapped into
    /// every fin pcurve by [`FinPcurve`]. `None` denotes an exact ring edge
    /// spanning one full period; curve-less ring edges are not supported.
    pub bounds: Option<(f64, f64)>,
    /// Fins using this edge, in creation order (2 for a manifold interior
    /// edge, 1 on a sheet boundary, 0 for wireframe).
    pub fins: Vec<FinId>,
    /// Validated tolerance and provenance for a tolerant edge (≥ session
    /// linear resolution), or `None` for an exact edge.
    pub tolerance: Option<EntityTolerance>,
}

impl Edge {
    /// Supporting curve geometry, absent for a curve-less tolerant edge.
    pub const fn curve(&self) -> Option<CurveId> {
        self.curve
    }

    /// Start and end vertices in edge direction.
    pub const fn vertices(&self) -> [Option<VertexId>; 2] {
        self.vertices
    }

    /// Active edge-parameter interval, absent for a full-period ring edge.
    pub const fn bounds(&self) -> Option<(f64, f64)> {
        self.bounds
    }

    /// Fins using this edge, in stored creation order.
    pub fn fins(&self) -> &[FinId] {
        &self.fins
    }

    /// Validated tolerant-edge metric data, when present.
    pub const fn tolerance(&self) -> Option<EntityTolerance> {
        self.tolerance
    }
}

/// A point of the model, shared by all edges that end there.
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    /// Position geometry.
    pub point: PointId,
    /// Validated tolerance and provenance for a tolerant vertex (≥ session
    /// linear resolution), or `None` for an exact vertex.
    pub tolerance: Option<EntityTolerance>,
}

impl Vertex {
    /// Attached point geometry.
    pub const fn point(&self) -> PointId {
        self.point
    }

    /// Validated tolerant-vertex metric data, when present.
    pub const fn tolerance(&self) -> Option<EntityTolerance> {
        self.tolerance
    }
}

/// A type-erased reference to any entity, for diagnostics ([`crate::check`])
/// and journaling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityRef {
    /// A body.
    Body(BodyId),
    /// A region.
    Region(RegionId),
    /// A shell.
    Shell(ShellId),
    /// A face.
    Face(FaceId),
    /// A loop.
    Loop(LoopId),
    /// A fin.
    Fin(FinId),
    /// An edge.
    Edge(EdgeId),
    /// A vertex.
    Vertex(VertexId),
    /// An attached curve.
    Curve(CurveId),
    /// An attached surface.
    Surface(SurfaceId),
    /// An attached point.
    Point(PointId),
    /// An attached parameter-space curve.
    Curve2d(Curve2dId),
}

#[cfg(test)]
mod tests {
    use super::*;
    use kgeom::frame::Frame;
    use kgeom::surface::Sphere;

    #[test]
    fn face_domain_is_finite_positive_and_composable() {
        assert!(FaceDomain::from_bounds(0.0, 0.0, 0.0, 1.0).is_err());
        assert!(FaceDomain::from_bounds(f64::NEG_INFINITY, 1.0, 0.0, 1.0).is_err());
        assert!(FaceDomain::from_bounds(-f64::MAX, f64::MAX, 0.0, 1.0).is_err());
        let a = FaceDomain::from_bounds(0.0, 1.0, -1.0, 0.0).unwrap();
        let b = FaceDomain::from_bounds(0.5, 2.0, -0.5, 1.0).unwrap();
        let union = a.union(b).unwrap();
        assert_eq!(union, FaceDomain::from_bounds(0.0, 2.0, -1.0, 1.0).unwrap());
        assert!(union.contains([1.5, 0.5]));
        assert!(!union.contains([2.5, 0.5]));
    }

    #[test]
    fn finite_natural_surface_domain_is_available() {
        let sphere = SurfaceGeom::Sphere(Sphere::new(Frame::world(), 1.0).unwrap());
        let domain = FaceDomain::natural(&sphere).unwrap();
        assert_eq!(domain.u.width(), core::f64::consts::TAU);
        assert_eq!(domain.v.width(), core::f64::consts::PI);
    }

    #[test]
    fn pcurve_chart_applies_only_valid_period_shifts() {
        let uv = Point2::new(0.25, -0.5);
        assert_eq!(
            PcurveChart::identity()
                .apply(uv, [Some(2.0), None])
                .unwrap(),
            uv
        );
        assert_eq!(
            PcurveChart::shifted([2, 0])
                .apply(uv, [Some(3.0), None])
                .unwrap(),
            Point2::new(6.25, -0.5)
        );
        assert!(
            PcurveChart::shifted([0, 1])
                .apply(uv, [Some(3.0), None])
                .is_err()
        );
    }
}
