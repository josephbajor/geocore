//! Internal foundations for the first planar solid Boolean slice.
//!
//! No public modeling operation is exposed yet. The bounded BSP below owns
//! the symbolic face-fragment contract that later imprint, classification,
//! and topology assembly stages will consume.

// This bounded rung-3 foundation remains internal until the topology
// assembler can consume its certified fragments without exposing a partial
// Boolean API.
#[allow(dead_code)]
mod component_layout;
#[allow(dead_code)]
mod components;
#[allow(dead_code)]
mod extract;
#[allow(dead_code)]
mod pipeline;
#[allow(dead_code)]
mod planar_bsp;
#[allow(dead_code)]
mod realize;
#[allow(dead_code)]
mod select;
