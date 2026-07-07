//! Deterministic parallel primitives.
//!
//! The kernel's determinism contract requires that results never depend on
//! thread count or scheduling. These primitives guarantee that by
//! construction: work items are computed independently (each by deterministic
//! code) and results are assembled in **input index order**, so `par_map`
//! is observably identical to `items.iter().map(f).collect()` at any
//! parallelism level, including 1.
//!
//! Reductions that are not index-ordered (e.g. summing floats in completion
//! order) are forbidden in the kernel; when a parallel reduction is needed,
//! map first, then reduce sequentially in index order.
//!
//! Built on `std::thread::scope` — no dependencies, no global thread pool,
//! no work stealing. Tessellation and face-pair intersection (roadmap M4/M5)
//! are the intended consumers; revisit pooling if spawn overhead ever shows
//! up in profiles.

use std::num::NonZeroUsize;
use std::thread;

/// Effective parallelism level: the smaller of available hardware
/// parallelism and `work_len`, and at least 1.
fn thread_count(work_len: usize) -> usize {
    let hw = thread::available_parallelism()
        .map(NonZeroUsize::get)
        .unwrap_or(1);
    hw.min(work_len).max(1)
}

/// Parallel map with deterministic, index-ordered results.
///
/// Equivalent to `items.iter().map(f).collect()` for any thread count.
/// `f` must itself be deterministic (true for all kernel code by contract).
pub fn par_map<T, U, F>(items: &[T], f: F) -> Vec<U>
where
    T: Sync,
    U: Send,
    F: Fn(&T) -> U + Sync,
{
    let n = items.len();
    if n == 0 {
        return Vec::new();
    }
    let threads = thread_count(n);
    if threads == 1 {
        return items.iter().map(f).collect();
    }
    // Split into `threads` contiguous chunks (sizes differ by at most one);
    // each chunk's results come back tagged with its start index and are
    // reassembled in order.
    let chunk_len = n.div_ceil(threads);
    let mut results: Vec<Vec<U>> = thread::scope(|scope| {
        let handles: Vec<_> = items
            .chunks(chunk_len)
            .map(|chunk| scope.spawn(|| chunk.iter().map(&f).collect::<Vec<U>>()))
            .collect();
        handles
            .into_iter()
            .map(|h| h.join().expect("kernel worker thread panicked"))
            .collect()
    });
    let mut out = Vec::with_capacity(n);
    for part in &mut results {
        out.append(part);
    }
    out
}

/// Parallel map over an index range, deterministic and index-ordered.
///
/// Convenience for producers that generate work by index rather than from a
/// slice (e.g. per-face tessellation over face numbers).
pub fn par_map_indices<U, F>(count: usize, f: F) -> Vec<U>
where
    U: Send,
    F: Fn(usize) -> U + Sync,
{
    let indices: Vec<usize> = (0..count).collect();
    par_map(&indices, |&i| f(i))
}

#[cfg(test)]
#[allow(clippy::disallowed_methods)] // tests may cross-check against platform libm
mod tests {
    use super::*;

    #[test]
    fn matches_serial_map_exactly() {
        let items: Vec<u64> = (0..10_007).collect();
        let f = |&x: &u64| x.wrapping_mul(0x9E37_79B9_7F4A_7C15).rotate_left(17);
        let serial: Vec<u64> = items.iter().map(f).collect();
        assert_eq!(par_map(&items, f), serial);
    }

    #[test]
    fn float_results_are_bit_identical_to_serial() {
        // The determinism contract is about bits, not approximate equality.
        let items: Vec<f64> = (0..5_000).map(|i| (i as f64).sin() * 100.0).collect();
        let f = |&x: &f64| (x * 1.000_000_1).sqrt().to_bits();
        let serial: Vec<u64> = items.iter().map(f).collect();
        assert_eq!(par_map(&items, f), serial);
    }

    #[test]
    fn empty_and_single_inputs() {
        let empty: Vec<i32> = Vec::new();
        assert!(par_map(&empty, |&x| x).is_empty());
        assert_eq!(par_map(&[42], |&x| x * 2), vec![84]);
    }

    #[test]
    fn index_variant_matches_direct_computation() {
        let direct: Vec<usize> = (0..1_000).map(|i| i * i).collect();
        assert_eq!(par_map_indices(1_000, |i| i * i), direct);
    }
}
