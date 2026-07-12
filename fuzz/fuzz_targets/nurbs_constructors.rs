//! libFuzzer entry point for bounded NURBS construction and queries.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|input: &[u8]| {
    kernel_fuzz_contracts::nurbs_constructors::exercise(input);
});
