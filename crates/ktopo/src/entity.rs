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

use crate::geom::{CurveGeom, SurfaceGeom};
use kcore::arena::Handle;
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
}

/// A bounded, connected subset of one curve.
#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    /// Supporting curve geometry; its parameterization direction *is* the
    /// edge direction. `None` only for tolerant edges with no exact curve
    /// (not constructed before M3c).
    pub curve: Option<CurveId>,
    /// Start and end vertices in curve direction. `[Some, Some]` for an
    /// ordinary edge (equal handles ⇔ closed edge through one vertex),
    /// `[None, None]` for a ring edge.
    pub vertices: [Option<VertexId>; 2],
    /// Curve-parameter interval `(t0, t1)`, `t0 < t1`, matching the
    /// vertices; for periodic curves possibly unwrapped past the period
    /// seam (`t1 - t0 ≤ period`). `None` ⇔ ring edge spanning the full
    /// period.
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
}
