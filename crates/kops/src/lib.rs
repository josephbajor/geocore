//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line intersections, including
//! transverse contacts, endpoint contacts, misses, and oriented coincident
//! intervals. General curve/curve, curve/surface, SSI, imprinting, and body
//! operations remain future M4 work.

pub mod intersect;
