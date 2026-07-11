//! Shared validation for stable, namespaced kernel identifiers.

/// Returns whether `value` is a lower-case ASCII dotted identifier.
///
/// Segments may contain digits and internal hyphens. Dots and hyphens cannot
/// be adjacent, leading, or trailing. Namespace ownership is checked by each
/// crate's known-identifier tests rather than by this syntax-only primitive.
pub(crate) const fn valid_identifier(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() < 3 {
        return false;
    }
    let mut index = 0;
    let mut has_namespace_separator = false;
    let mut previous_was_separator = true;
    while index < bytes.len() {
        let byte = bytes[index];
        let is_alphanumeric = byte.is_ascii_lowercase() || byte.is_ascii_digit();
        if is_alphanumeric {
            previous_was_separator = false;
        } else if byte == b'.' {
            if previous_was_separator {
                return false;
            }
            has_namespace_separator = true;
            previous_was_separator = true;
        } else if byte == b'-' {
            if previous_was_separator {
                return false;
            }
            previous_was_separator = true;
        } else {
            return false;
        }
        index += 1;
    }
    has_namespace_separator && !previous_was_separator
}
