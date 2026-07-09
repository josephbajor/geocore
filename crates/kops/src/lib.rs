//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line, 3D line/circle, 3D
//! line/ellipse, 3D circle/circle, 3D circle/ellipse, and 3D ellipse/ellipse
//! intersections. These cover transverse and tangent contacts, periodic arc
//! filtering, misses, and oriented coincident overlaps. General curve/curve,
//! curve/surface, SSI, imprinting, and body operations remain future M4 work.

pub mod intersect;
