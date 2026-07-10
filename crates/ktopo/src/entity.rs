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

use crate::geom::{Curve2dGeom, CurveGeom, SurfaceGeom};
use kcore::arena::Handle;
use kcore::error::{Error, Result};
use kgeom::param::ParamRange;
use kgeom::vec::Point3;

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
/// Handle to an attached curve.
pub type CurveId = Handle<CurveGeom>;
/// Handle to an attached surface.
pub type SurfaceId = Handle<SurfaceGeom>;
/// Handle to an attached point.
pub type PointId = Handle<Point3>;
/// Handle to an attached parameter-space curve.
pub type Curve2dId = Handle<Curve2dGeom>;

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
}

/// A closed ring of fins bounding a face.
#[derive(Debug, Clone, PartialEq)]
pub struct Loop {
    /// Owning face.
    pub face: FaceId,
    /// Fins in traversal order (see module docs for orientation).
    pub fins: Vec<FinId>,
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

    /// Pcurve parameter direction relative to the edge direction.
    pub fn sense(self) -> Sense {
        self.edge_to_pcurve.sense()
    }

    /// Evaluate the pcurve parameter corresponding to an edge parameter.
    pub fn parameter_at_edge(self, edge_parameter: f64) -> f64 {
        self.edge_to_pcurve.map(edge_parameter)
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
    /// Tolerance for a tolerant edge (≥ session linear resolution), or
    /// `None` for an exact edge.
    pub tolerance: Option<f64>,
}

/// A point of the model, shared by all edges that end there.
#[derive(Debug, Clone, PartialEq)]
pub struct Vertex {
    /// Position geometry.
    pub point: PointId,
    /// Tolerance for a tolerant vertex (≥ session linear resolution), or
    /// `None` for an exact vertex.
    pub tolerance: Option<f64>,
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
