//! Legacy affine-only helpers retained for focused parent-module tests.

use super::*;

fn component_bounds(component: &SectionPeriodicComponentEmbedding) -> Bounds2 {
    let mut fragments = component.fragments.iter();
    let first = fragments.next().expect("certified component is nonempty");
    let mut bounds = public_fragment_bounds(first);
    for fragment in fragments {
        let next = public_fragment_bounds(fragment);
        bounds.u = hull(bounds.u, next.u);
        bounds.v = hull(bounds.v, next.v);
    }
    bounds
}

fn public_fragment_bounds(fragment: &SectionPeriodicFragmentEmbedding) -> Bounds2 {
    let endpoints = fragment.endpoints();
    let start_u = Interval::new(endpoints[0][0].lo(), endpoints[0][0].hi());
    let end_u = Interval::new(endpoints[1][0].lo(), endpoints[1][0].hi());
    let start_v = Interval::new(endpoints[0][1].lo(), endpoints[0][1].hi());
    let end_v = Interval::new(endpoints[1][1].lo(), endpoints[1][1].hi());
    Bounds2 {
        u: hull(start_u, end_u),
        v: hull(start_v, end_v),
    }
}

pub(super) fn certify_component_separation(
    components: &[SectionPeriodicComponentEmbedding],
) -> Result<(), SectionPeriodicEmbeddingGap> {
    for first in 0..components.len() {
        for second in (first + 1)..components.len() {
            let first_bounds = component_bounds(&components[first]);
            let second_bounds = component_bounds(&components[second]);
            if strictly_disjoint(first_bounds, second_bounds) {
                continue;
            }
            for left in &components[first].fragments {
                for right in &components[second].fragments {
                    if !strictly_disjoint(
                        public_fragment_bounds(left),
                        public_fragment_bounds(right),
                    ) {
                        return Err(
                            SectionPeriodicEmbeddingGap::ComponentIntersectionProofRequired {
                                first: components[first].component,
                                second: components[second].component,
                            },
                        );
                    }
                }
            }
            return Err(
                SectionPeriodicEmbeddingGap::ContainmentClassificationRequired {
                    first: components[first].component,
                    second: components[second].component,
                },
            );
        }
    }
    Ok(())
}
