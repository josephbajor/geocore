//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line, 3D line/circle, 3D
//! line/ellipse, 3D circle/circle, 3D circle/ellipse, and 3D ellipse/ellipse
//! intersections, plus a general analytic dispatcher over those classes.
//! Curve/surface has started with bounded line/plane, line/cylinder,
//! line/cone, line/sphere, and line/torus plus bounded circle/plane and
//! ellipse/plane, circle/cylinder, circle/cone, circle/sphere, and
//! ellipse/sphere. These cover transverse and tangent contacts, periodic arc
//! filtering, misses, and oriented coincident overlaps. General
//! NURBS/procedural curve/curve, broader curve/surface, SSI, imprinting, and
//! body operations remain future M4 work.

pub mod intersect;
