//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line, 3D line/circle, 3D
//! line/ellipse, 3D circle/circle, and 3D circle/ellipse intersections. These
//! cover transverse and tangent contacts, periodic arc filtering, misses, and
//! oriented coincident overlaps. Other analytic curve/curve cases,
//! curve/surface, SSI, imprinting, and body operations remain future M4 work.

pub mod intersect;
