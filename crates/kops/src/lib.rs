//! `kops` - Layer 3 intersection and modeling-operation foundation.
//!
//! Currently implements parameter-rich curve/curve result contracts and
//! deterministic, tolerance-aware bounded line/line, 3D line/circle, 3D
//! line/ellipse, 3D circle/circle, 3D circle/ellipse, and 3D ellipse/ellipse
//! intersections, plus a general analytic dispatcher over those classes.
//! Curve/surface has started with bounded line/plane, line/cylinder,
//! line/cone, line/sphere, and line/torus plus bounded circle/plane and
//! ellipse/plane, circle/cylinder, circle/cone, circle/sphere, and
//! circle/torus plus ellipse/sphere, ellipse/cylinder, ellipse/cone, and
//! ellipse/torus. These cover transverse and tangent contacts, periodic arc
//! filtering, misses, and oriented coincident overlaps. Bounded surface/surface
//! intersections have started with plane/plane, plane/sphere, plane/cylinder,
//! plane/cone circular and elliptic slices, coaxial cylinder/sphere,
//! cylinder/torus, cone/cone, cone/cylinder, cone/sphere, cone/torus,
//! sphere/torus, and torus/torus circles, parallel cylinder/cylinder rulings,
//! plane/torus latitude and meridian circles, sphere/sphere closed forms, and
//! initial marched plane/sphere/cylinder/cone/NURBS-surface branches. General
//! NURBS/procedural curve/curve, broader curve/surface, broader SSI,
//! imprinting, and body operations remain future M4 work; SSI branch results
//! can carry exact NURBS curves for broader intersection edge geometry.

pub mod intersect;
