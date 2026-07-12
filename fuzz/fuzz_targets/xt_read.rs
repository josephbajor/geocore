//! libFuzzer entry point for the bounded X_T read contract.

#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|input: &[u8]| {
    kernel_fuzz_contracts::xt_read::exercise(input);
});
