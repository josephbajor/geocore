//! Assembly of one connected planar solid region with cavity shells.
//!
//! The caller supplies one positive outer shell and one or more negative
//! cavity shells. Every shell is preflighted independently before topology is
//! allocated. Geometric nesting is deliberately not trusted here: a later
//! Full check must reconstruct the bound semantic planes and certify the
//! complete region layout before the transaction can commit.

use crate::entity::{BodyId, Region, RegionKind, Shell, ShellId};
use crate::planar::{PlanarSolidInput, PreparedShellWinding, PreparedSolid};
use crate::transaction::Transaction;
use kcore::error::{Error, Result};
use std::collections::BTreeSet;

/// One connected solid proposal with direct cavity boundary components.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanarMultiShellSolidInput {
    outer: PlanarSolidInput,
    cavities: Vec<PlanarSolidInput>,
}

impl PlanarMultiShellSolidInput {
    /// Construct an input. Complete preflight occurs during assembly.
    pub fn new(outer: PlanarSolidInput, cavities: Vec<PlanarSolidInput>) -> Self {
        Self { outer, cavities }
    }

    /// Positive outer boundary proposal.
    pub const fn outer(&self) -> &PlanarSolidInput {
        &self.outer
    }

    /// Negative direct cavity boundary proposals.
    pub fn cavities(&self) -> &[PlanarSolidInput] {
        &self.cavities
    }
}

/// Handles produced for one multi-shell solid proposal.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanarMultiShellSolidOutput {
    body: BodyId,
    outer_shell: ShellId,
    cavity_shells: Vec<ShellId>,
}

impl PlanarMultiShellSolidOutput {
    /// Assembled connected solid body.
    pub const fn body(&self) -> BodyId {
        self.body
    }

    /// Positive outer boundary shell.
    pub const fn outer_shell(&self) -> ShellId {
        self.outer_shell
    }

    /// Negative cavity boundary shells in input order.
    pub fn cavity_shells(&self) -> &[ShellId] {
        &self.cavity_shells
    }
}

impl Transaction<'_> {
    /// Assemble one positive outer shell plus direct negative cavity shells.
    ///
    /// This method proves only per-shell combinatorial and signed-volume
    /// preconditions. The caller must use a Full checked commit to establish
    /// cross-shell separation, containment, and region ownership.
    pub fn assemble_planar_multishell_solid(
        &mut self,
        input: &PlanarMultiShellSolidInput,
    ) -> Result<PlanarMultiShellSolidOutput> {
        if input.cavities.is_empty() {
            return invalid("a planar multishell solid requires at least one cavity shell");
        }
        validate_disjoint_keys(input)?;
        let outer = PreparedSolid::new_with_winding(
            &input.outer,
            self.store(),
            PreparedShellWinding::Positive,
        )?;
        let cavities = input
            .cavities
            .iter()
            .map(|cavity| {
                PreparedSolid::new_with_winding(
                    cavity,
                    self.store(),
                    PreparedShellWinding::Negative,
                )
            })
            .collect::<Result<Vec<_>>>()?;

        let (body, outer_shell) = crate::make::solid_body_scaffold(self.store_mut());
        let solid_region = self.store().get(outer_shell)?.region;
        let mut cavity_shells = Vec::with_capacity(cavities.len());
        for _ in &cavities {
            let cavity_shell = self.store_mut().add(Shell {
                region: solid_region,
                faces: Vec::new(),
                edges: Vec::new(),
                vertex: None,
            });
            self.store_mut()
                .get_mut(solid_region)?
                .shells
                .push(cavity_shell);
            let void = self.store_mut().add(Region {
                body,
                kind: RegionKind::Void,
                shells: Vec::new(),
            });
            self.store_mut().get_mut(body)?.regions.push(void);
            cavity_shells.push(cavity_shell);
        }

        let outer = self.allocate_prepared_planar_shell(outer, outer_shell)?;
        debug_assert_eq!(outer.shell, outer_shell);
        for (prepared, shell) in cavities.into_iter().zip(cavity_shells.iter().copied()) {
            let allocated = self.allocate_prepared_planar_shell(prepared, shell)?;
            debug_assert_eq!(allocated.shell, shell);
        }

        Ok(PlanarMultiShellSolidOutput {
            body,
            outer_shell,
            cavity_shells,
        })
    }
}

fn validate_disjoint_keys(input: &PlanarMultiShellSolidInput) -> Result<()> {
    let mut keys = BTreeSet::new();
    for shell in core::iter::once(&input.outer).chain(&input.cavities) {
        for vertex in shell.vertices() {
            if !keys.insert(vertex.key()) {
                return invalid("planar multishell components must use disjoint vertex keys");
            }
        }
    }
    Ok(())
}

fn invalid<T>(reason: &'static str) -> Result<T> {
    Err(Error::InvalidGeometry { reason })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::check::check_body;
    use crate::entity::{Body, Face, Region, Shell, Vertex};
    use crate::planar::{PlanarSolidFace, PlanarSolidVertex, PlanarVertexKey};
    use crate::store::Store;
    use kgeom::vec::Point3;

    fn box_shell(first_key: u64, half: f64, negative: bool) -> PlanarSolidInput {
        let points = [
            [-half, -half, -half],
            [half, -half, -half],
            [-half, half, -half],
            [half, half, -half],
            [-half, -half, half],
            [half, -half, half],
            [-half, half, half],
            [half, half, half],
        ];
        let keys: [PlanarVertexKey; 8] =
            core::array::from_fn(|index| PlanarVertexKey::new(first_key + index as u64));
        let vertices = points
            .into_iter()
            .enumerate()
            .map(|(index, [x, y, z])| PlanarSolidVertex::new(keys[index], Point3::new(x, y, z)))
            .collect();
        let rings = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        let faces = rings
            .into_iter()
            .map(|ring| {
                let mut ring = ring.map(|index| keys[index]).to_vec();
                if negative {
                    ring.reverse();
                }
                PlanarSolidFace::new(ring)
            })
            .collect();
        PlanarSolidInput::new(vertices, faces)
    }

    #[test]
    fn outer_and_negative_cavity_preflight_before_allocating_clean_topology() {
        let input = PlanarMultiShellSolidInput::new(
            box_shell(10, 2.0, false),
            vec![box_shell(100, 0.75, true)],
        );
        let mut store = Store::new();
        let mut transaction = store.transaction().unwrap();
        let output = transaction
            .assemble_planar_multishell_solid(&input)
            .unwrap();
        assert!(
            check_body(transaction.store(), output.body())
                .unwrap()
                .is_empty()
        );
        assert_eq!(transaction.store().count::<Body>(), 1);
        assert_eq!(transaction.store().count::<Region>(), 3);
        assert_eq!(transaction.store().count::<Shell>(), 2);
        assert_eq!(transaction.store().count::<Face>(), 12);
        assert_eq!(transaction.store().count::<Vertex>(), 16);
        let body = transaction.store().get(output.body()).unwrap();
        let solid = body
            .regions
            .iter()
            .copied()
            .find(|region| transaction.store().get(*region).unwrap().kind == RegionKind::Solid)
            .unwrap();
        assert_eq!(transaction.store().get(solid).unwrap().shells.len(), 2);
        assert_eq!(output.cavity_shells().len(), 1);
    }

    #[test]
    fn wrong_cavity_winding_and_shared_keys_fail_before_allocation() {
        for input in [
            PlanarMultiShellSolidInput::new(
                box_shell(10, 2.0, false),
                vec![box_shell(100, 0.75, false)],
            ),
            PlanarMultiShellSolidInput::new(
                box_shell(10, 2.0, false),
                vec![box_shell(10, 0.75, true)],
            ),
        ] {
            let mut store = Store::new();
            let mut transaction = store.transaction().unwrap();
            assert!(
                transaction
                    .assemble_planar_multishell_solid(&input)
                    .is_err()
            );
            assert_eq!(transaction.store().count::<Body>(), 0);
            assert_eq!(transaction.store().count::<Region>(), 0);
            assert_eq!(transaction.store().count::<Shell>(), 0);
        }
    }
}
