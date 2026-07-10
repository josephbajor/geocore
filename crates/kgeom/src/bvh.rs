//! Deterministic conservative axis-aligned bounding-volume hierarchy.
//!
//! The hierarchy stores primitive boxes by caller-provided index and uses a
//! stable median split of box centers. It is deliberately geometry-agnostic:
//! NURBS patches, topology faces, bodies, and interrogation primitives can
//! share the same broad-phase without creating layer dependencies.

use crate::aabb::Aabb3;
use crate::vec::Vec3;
use kcore::error::{Error, Result};

/// Balanced deterministic hierarchy over finite, non-empty primitive boxes.
#[derive(Debug, Clone, PartialEq)]
pub struct AabbBvh {
    primitive_bounds: Vec<Aabb3>,
    nodes: Vec<Node>,
    root: Option<usize>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum NodeKind {
    Leaf { primitive: usize },
    Branch { left: usize, right: usize },
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Node {
    bounds: Aabb3,
    kind: NodeKind,
}

impl AabbBvh {
    /// Build a deterministic hierarchy. Primitive indices in all query
    /// results refer to positions in `primitive_bounds`.
    pub fn build(primitive_bounds: &[Aabb3]) -> Result<Self> {
        if primitive_bounds
            .iter()
            .any(|bounds| bounds.is_empty() || !bounds.is_finite())
        {
            return Err(Error::InvalidGeometry {
                reason: "BVH primitive bounds must be finite and non-empty",
            });
        }
        if primitive_bounds.is_empty() {
            return Ok(Self {
                primitive_bounds: Vec::new(),
                nodes: Vec::new(),
                root: None,
            });
        }

        let primitive_bounds = primitive_bounds.to_vec();
        let mut primitives: Vec<_> = (0..primitive_bounds.len()).collect();
        let mut nodes = Vec::with_capacity(2 * primitive_bounds.len() - 1);
        let root = build_node(&mut primitives, &primitive_bounds, &mut nodes);
        Ok(Self {
            primitive_bounds,
            nodes,
            root: Some(root),
        })
    }

    /// Number of indexed primitives.
    pub fn primitive_count(&self) -> usize {
        self.primitive_bounds.len()
    }

    /// Number of hierarchy nodes. A non-empty binary hierarchy has
    /// `2 * primitive_count - 1` nodes.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Original bound for one primitive.
    pub fn primitive_bounds(&self, primitive: usize) -> Option<Aabb3> {
        self.primitive_bounds.get(primitive).copied()
    }

    /// Root bound, or `None` for an empty hierarchy.
    pub fn root_bounds(&self) -> Option<Aabb3> {
        self.root.map(|root| self.nodes[root].bounds)
    }

    /// Primitive indices whose boxes intersect `query` after each primitive
    /// box is grown by `margin`. Results are sorted by original index.
    pub fn query_aabb(&self, query: Aabb3, margin: f64) -> Result<Vec<usize>> {
        validate_margin(margin)?;
        if query.is_empty() {
            return Ok(Vec::new());
        }
        if !query.is_finite() {
            return Err(Error::InvalidGeometry {
                reason: "BVH query bounds must be finite or empty",
            });
        }
        Ok(self.query_pruned(|bounds| bounds.inflated(margin).intersects(query)))
    }

    /// Candidate primitive pairs whose boxes overlap within
    /// `max_separation`. An empty result is a conservative proof that no pair
    /// of represented points is closer than that separation. Results are
    /// lexicographically sorted by original primitive indices.
    pub fn overlapping_pairs(
        &self,
        other: &AabbBvh,
        max_separation: f64,
    ) -> Result<Vec<(usize, usize)>> {
        validate_margin(max_separation)?;
        let (Some(root_a), Some(root_b)) = (self.root, other.root) else {
            return Ok(Vec::new());
        };
        let padding = if max_separation == 0.0 {
            0.0
        } else {
            (0.5 * max_separation).next_up()
        };
        let mut stack = vec![(root_a, root_b)];
        let mut pairs = Vec::new();
        while let Some((node_a, node_b)) = stack.pop() {
            let a = self.nodes[node_a];
            let b = other.nodes[node_b];
            if !a
                .bounds
                .inflated(padding)
                .intersects(b.bounds.inflated(padding))
            {
                continue;
            }
            match (a.kind, b.kind) {
                (
                    NodeKind::Leaf {
                        primitive: primitive_a,
                    },
                    NodeKind::Leaf {
                        primitive: primitive_b,
                    },
                ) => pairs.push((primitive_a, primitive_b)),
                (NodeKind::Branch { left, right }, NodeKind::Leaf { .. }) => {
                    stack.push((right, node_b));
                    stack.push((left, node_b));
                }
                (NodeKind::Leaf { .. }, NodeKind::Branch { left, right }) => {
                    stack.push((node_a, right));
                    stack.push((node_a, left));
                }
                (
                    NodeKind::Branch {
                        left: left_a,
                        right: right_a,
                    },
                    NodeKind::Branch {
                        left: left_b,
                        right: right_b,
                    },
                ) => {
                    stack.push((right_a, right_b));
                    stack.push((right_a, left_b));
                    stack.push((left_a, right_b));
                    stack.push((left_a, left_b));
                }
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        Ok(pairs)
    }

    /// Traverse nodes for which `could_match` remains true and return their
    /// leaf primitive indices. The predicate must be conservative and
    /// monotone: returning false for a parent must imply false for every
    /// descendant. Used by geometry-specific certified exclusion layers.
    pub(crate) fn query_pruned(&self, mut could_match: impl FnMut(Aabb3) -> bool) -> Vec<usize> {
        let Some(root) = self.root else {
            return Vec::new();
        };
        let mut stack = vec![root];
        let mut primitives = Vec::new();
        while let Some(index) = stack.pop() {
            let node = self.nodes[index];
            if !could_match(node.bounds) {
                continue;
            }
            match node.kind {
                NodeKind::Leaf { primitive } => primitives.push(primitive),
                NodeKind::Branch { left, right } => {
                    stack.push(right);
                    stack.push(left);
                }
            }
        }
        primitives.sort_unstable();
        primitives
    }
}

fn build_node(
    primitives: &mut [usize],
    primitive_bounds: &[Aabb3],
    nodes: &mut Vec<Node>,
) -> usize {
    let bounds = primitives.iter().fold(Aabb3::empty(), |combined, &index| {
        combined.union(primitive_bounds[index])
    });
    if let [primitive] = primitives {
        let index = nodes.len();
        nodes.push(Node {
            bounds,
            kind: NodeKind::Leaf {
                primitive: *primitive,
            },
        });
        return index;
    }

    let center_bounds = primitives.iter().fold(Aabb3::empty(), |combined, &index| {
        combined.including(center(primitive_bounds[index]))
    });
    let extent = center_bounds.max - center_bounds.min;
    let axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    primitives.sort_by(|&a, &b| {
        component(center(primitive_bounds[a]), axis)
            .total_cmp(&component(center(primitive_bounds[b]), axis))
            .then(a.cmp(&b))
    });
    let middle = primitives.len() / 2;
    let (left_primitives, right_primitives) = primitives.split_at_mut(middle);
    let left = build_node(left_primitives, primitive_bounds, nodes);
    let right = build_node(right_primitives, primitive_bounds, nodes);
    let index = nodes.len();
    nodes.push(Node {
        bounds,
        kind: NodeKind::Branch { left, right },
    });
    index
}

fn center(bounds: Aabb3) -> Vec3 {
    bounds.min * 0.5 + bounds.max * 0.5
}

fn component(vector: Vec3, axis: usize) -> f64 {
    match axis {
        0 => vector.x,
        1 => vector.y,
        _ => vector.z,
    }
}

fn validate_margin(margin: f64) -> Result<()> {
    if !margin.is_finite() || margin < 0.0 {
        return Err(Error::InvalidGeometry {
            reason: "BVH margin must be finite and non-negative",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn box_at(x: f64, y: f64) -> Aabb3 {
        Aabb3::from_points(&[Vec3::new(x, y, -0.5), Vec3::new(x + 0.75, y + 0.75, 0.5)])
    }

    #[test]
    fn build_and_queries_are_deterministic_and_index_ordered() {
        let bounds = [
            box_at(0.0, 0.0),
            box_at(2.0, 0.0),
            box_at(0.0, 2.0),
            box_at(2.0, 2.0),
        ];
        let first = AabbBvh::build(&bounds).unwrap();
        let second = AabbBvh::build(&bounds).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.primitive_count(), 4);
        assert_eq!(first.node_count(), 7);
        assert_eq!(
            first.root_bounds(),
            Some(bounds.into_iter().fold(Aabb3::empty(), Aabb3::union))
        );
        assert_eq!(
            first
                .query_aabb(
                    Aabb3::from_points(&[Vec3::new(-0.1, -0.1, -1.0), Vec3::new(2.1, 0.5, 1.0),]),
                    0.0,
                )
                .unwrap(),
            vec![0, 1]
        );
        assert!(
            first
                .query_aabb(Aabb3::from_point(Vec3::new(10.0, 10.0, 10.0)), 0.0)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn pair_query_proves_separation_and_honors_distance_margin() {
        let a = AabbBvh::build(&[box_at(0.0, 0.0), box_at(3.0, 0.0)]).unwrap();
        let b = AabbBvh::build(&[box_at(1.0, 0.0), box_at(10.0, 0.0)]).unwrap();
        assert!(a.overlapping_pairs(&b, 0.0).unwrap().is_empty());
        assert_eq!(a.overlapping_pairs(&b, 0.25).unwrap(), vec![(0, 0)]);
        assert!(
            a.overlapping_pairs(&AabbBvh::build(&[]).unwrap(), 0.0)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn invalid_bounds_and_margins_are_rejected() {
        assert!(AabbBvh::build(&[Aabb3::empty()]).is_err());
        assert!(AabbBvh::build(&[Aabb3::from_point(Vec3::new(f64::INFINITY, 0.0, 0.0))]).is_err());
        let bvh = AabbBvh::build(&[box_at(0.0, 0.0)]).unwrap();
        assert!(bvh.query_aabb(box_at(0.0, 0.0), -1.0).is_err());
        assert!(bvh.overlapping_pairs(&bvh, f64::NAN).is_err());
    }
}
