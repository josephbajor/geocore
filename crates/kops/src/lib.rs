//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line and 3D line/circle
//! intersections. These cover transverse and tangent contacts, periodic arc
//! filtering, misses, and oriented line overlaps. Other analytic curve/curve
//! cases, curve/surface, SSI, imprinting, and body operations remain future M4
//! work.

pub mod intersect;
