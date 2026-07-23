//! Failure-atomic work accounting for periodic embedding proofs.

use super::*;

/// Precharge the geometry-independent ceiling for all pairwise simplicity
/// and separation candidates plus every sealed procedural pcurve reissue.
pub(super) fn charge_pair_candidates(
    fragment_count: usize,
    component_count: usize,
    periodic_faces: usize,
    procedural_face_uses: usize,
    scope: &mut OperationScope<'_, '_>,
) -> KernelResult<bool> {
    let Some(amount) = periodic_embedding_work(
        fragment_count,
        component_count,
        periodic_faces,
        procedural_face_uses,
    ) else {
        return Ok(false);
    };
    scope
        .ledger_mut()
        .charge(SECTION_WORK, amount)
        .map_err(Error::from)?;
    Ok(true)
}

pub(super) fn periodic_embedding_work(
    fragment_count: usize,
    component_count: usize,
    periodic_faces: usize,
    procedural_face_uses: usize,
) -> Option<u64> {
    periodic_pair_candidate_work(fragment_count, component_count, periodic_faces)?.checked_add(
        u64::try_from(procedural_face_uses)
            .ok()?
            .checked_mul(procedural::FRAGMENT_WORK)?,
    )
}

pub(super) fn bounded_procedural_face_uses(
    branches: &[SectionBranch],
    fragments: &[SectionCurveFragment],
    faces: &[(usize, FaceId)],
) -> Option<usize> {
    let mut uses = 0_usize;
    for &(operand, ref face) in faces {
        for fragment in fragments {
            if !matches!(
                fragment.span(),
                SectionCurveFragmentSpan::BoundedProcedural { .. }
            ) {
                continue;
            }
            let branch = branches.get(fragment.branch())?;
            if branch.faces().get(operand) == Some(face) {
                uses = uses.checked_add(1)?;
            }
        }
    }
    Some(uses)
}

pub(super) fn periodic_pair_candidate_work(
    fragment_count: usize,
    component_count: usize,
    periodic_faces: usize,
) -> Option<u64> {
    let fragment_uses = u64::try_from(fragment_count).ok()?;
    let component_count = u64::try_from(component_count).ok()?;
    let face_count = u64::try_from(periodic_faces).ok()?;
    unordered_pairs(fragment_uses)?
        .checked_add(unordered_pairs(component_count)?)?
        .checked_mul(face_count)
}

pub(super) fn unordered_pairs(count: u64) -> Option<u64> {
    if count < 2 {
        return Some(0);
    }
    let predecessor = count.checked_sub(1)?;
    let (left, right) = if count.is_multiple_of(2) {
        (count / 2, predecessor)
    } else {
        (count, predecessor / 2)
    };
    left.checked_mul(right)
}

/// Exact operation-local work ceiling for recertifying one face-local
/// fragment subset. This legacy adapter is used only by affine parallel-
/// cylinder subsets; nonlinear consumers debit [`periodic_embedding_work`].
pub(crate) fn periodic_face_fragment_subset_work(fragment_count: usize) -> Option<u64> {
    let fragments = u64::try_from(fragment_count).ok()?;
    2_u64
        .checked_add(fragments)?
        .checked_add(unordered_pairs(fragments)?)
}
