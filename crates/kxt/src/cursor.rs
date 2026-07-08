//! Wire-format cursors: typed readers over the text and neutral-binary
//! encodings of the node stream.
//!
//! Both encodings share one logical field model (see [`crate::schema`]);
//! the cursors expose typed reads and the parser drives them from the
//! schema. Encoding rules implemented here, per the XT Format Reference:
//!
//! **Text**: newline and carriage-return characters are ignored entirely
//! (writers wrap at ~80 columns, splitting tokens arbitrarily). Every
//! number is followed by a single space; fields of type `c` and `l` are
//! not. Logicals are `T`/`F`. The null int (−32764) and null double
//! (−3.14158e13) are written as `?` (a null vector is a single `?`).
//! Strings escape NUL, CR, LF and backslash as `\0 \n \r \\`, and `\9`
//! denotes nine spaces.
//!
//! **Neutral binary**: big-endian IEEE. Bytes are bytes; shorts 2 bytes;
//! ints 4; doubles 8. Pointer indices, node indices, and "positive
//! integers" use a short-based variable encoding: values < 32767 are
//! written as one short holding `value + 1`; larger values as the pair
//! `(-(value % 32767 + 1), value / 32767)`.

use crate::error::{Result, XtError};

/// The special "unset" int value.
pub const NULL_INT: i32 = -32764;
/// The special "unset" double value.
pub const NULL_DOUBLE: f64 = -3.14158e13;

/// A scalar read from the stream: either a value or the null marker.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Scalar {
    /// An integral value (int, short, byte, pointer…).
    Int(i64),
    /// A floating-point value.
    Double(f64),
    /// The unset marker (`?` in text, sentinel values in binary).
    Null,
}

/// Typed reads over one of the two wire encodings.
pub trait Cursor {
    /// Current byte offset (diagnostics).
    fn offset(&self) -> usize;
    /// True if only trailing whitespace remains.
    fn at_end(&mut self) -> bool;
    /// A number that may be null: int-valued context.
    fn int(&mut self) -> Result<Scalar>;
    /// A number that may be null: double-valued context.
    fn double(&mut self) -> Result<Scalar>;
    /// A short int.
    fn short(&mut self) -> Result<i32>;
    /// An unsigned byte.
    fn byte(&mut self) -> Result<u32>;
    /// A pointer / node index (variable short encoding in binary).
    fn ptr(&mut self) -> Result<u32>;
    /// A "positive integer" (same encoding as pointers).
    fn positive(&mut self) -> Result<u32>;
    /// A node type (2-byte integer in binary, plain number in text).
    fn node_type(&mut self) -> Result<u16>;
    /// A variable-length count (4-byte integer in binary).
    fn varlen(&mut self) -> Result<u32>;
    /// A single character field (`c`).
    fn ch(&mut self) -> Result<char>;
    /// A logical (`l`).
    fn logical(&mut self) -> Result<bool>;
    /// Exactly `n` characters of string data.
    fn chars(&mut self, n: usize) -> Result<String>;
    /// A short string: length then characters.
    fn short_string(&mut self) -> Result<String> {
        let n = self.byte()? as usize;
        self.chars(n)
    }
    /// A vector: three doubles, or a lone `?` in text when all three are
    /// unset. Returns `None` for the null vector.
    fn vector(&mut self) -> Result<Option<[f64; 3]>>;
}

fn parse_err<T>(offset: usize, what: &'static str) -> Result<T> {
    Err(XtError::Parse { offset, what })
}

// ---------------------------------------------------------------- text --

/// Cursor over the text encoding. Construct with the raw data-section
/// bytes; newlines and carriage returns are stripped up front.
pub struct TextCursor {
    data: Vec<u8>,
    pos: usize,
}

impl TextCursor {
    /// New cursor over the data section (everything after the header).
    pub fn new(raw: &[u8]) -> TextCursor {
        TextCursor {
            data: raw
                .iter()
                .copied()
                .filter(|&b| b != b'\n' && b != b'\r')
                .collect(),
            pos: 0,
        }
    }

    fn skip_spaces(&mut self) {
        while self.data.get(self.pos) == Some(&b' ') {
            self.pos += 1;
        }
    }

    fn peek(&self) -> Option<u8> {
        self.data.get(self.pos).copied()
    }

    fn bump(&mut self) -> Result<u8> {
        let b = self.peek().ok_or(XtError::Parse {
            offset: self.pos,
            what: "unexpected end of data",
        })?;
        self.pos += 1;
        Ok(b)
    }

    /// Read a numeric token (or `?`). Numbers are terminated by their
    /// single trailing space or by any character that cannot continue
    /// them (e.g. a following `c`/`l` field).
    fn number(&mut self) -> Result<Scalar> {
        self.skip_spaces();
        if self.peek() == Some(b'?') {
            self.pos += 1;
            return Ok(Scalar::Null);
        }
        let start = self.pos;
        let mut seen_e = false;
        let mut prev_e = false;
        while let Some(b) = self.peek() {
            let ok = match b {
                b'0'..=b'9' => true,
                b'.' => !seen_e,
                b'+' | b'-' => self.pos == start || prev_e,
                b'e' | b'E' => {
                    if seen_e || self.pos == start {
                        false
                    } else {
                        seen_e = true;
                        true
                    }
                }
                _ => false,
            };
            if !ok {
                break;
            }
            prev_e = b == b'e' || b == b'E';
            self.pos += 1;
        }
        if self.pos == start {
            return parse_err(start, "expected a number");
        }
        let tok =
            core::str::from_utf8(&self.data[start..self.pos]).expect("numeric bytes are ASCII");
        // Numbers are followed by a single space separator when more data
        // follows.
        if self.peek() == Some(b' ') {
            self.pos += 1;
        }
        if tok.bytes().any(|b| matches!(b, b'.' | b'e' | b'E')) {
            let v: f64 = tok.parse().map_err(|_| XtError::Parse {
                offset: start,
                what: "malformed floating-point token",
            })?;
            Ok(Scalar::Double(v))
        } else {
            match tok.parse::<i64>() {
                Ok(v) => Ok(Scalar::Int(v)),
                // Very long digit strings are doubles written without
                // exponent.
                Err(_) => match tok.parse::<f64>() {
                    Ok(v) => Ok(Scalar::Double(v)),
                    Err(_) => parse_err(start, "malformed numeric token"),
                },
            }
        }
    }

    fn int_exact(&mut self, what: &'static str) -> Result<i64> {
        match self.number()? {
            Scalar::Int(v) => Ok(v),
            _ => parse_err(self.pos, what),
        }
    }
}

impl Cursor for TextCursor {
    fn offset(&self) -> usize {
        self.pos
    }

    fn at_end(&mut self) -> bool {
        self.skip_spaces();
        self.pos >= self.data.len()
    }

    fn int(&mut self) -> Result<Scalar> {
        match self.number()? {
            Scalar::Double(_) => parse_err(self.pos, "expected an integer, found a double"),
            s => Ok(s),
        }
    }

    fn double(&mut self) -> Result<Scalar> {
        Ok(match self.number()? {
            Scalar::Int(v) => Scalar::Double(v as f64),
            s => s,
        })
    }

    fn short(&mut self) -> Result<i32> {
        Ok(self.int_exact("expected a short")? as i32)
    }

    fn byte(&mut self) -> Result<u32> {
        let v = self.int_exact("expected a byte")?;
        if (0..=255).contains(&v) {
            Ok(v as u32)
        } else {
            parse_err(self.pos, "byte out of range")
        }
    }

    fn ptr(&mut self) -> Result<u32> {
        let v = self.int_exact("expected a pointer index")?;
        u32::try_from(v).map_err(|_| XtError::Parse {
            offset: self.pos,
            what: "negative pointer index",
        })
    }

    fn positive(&mut self) -> Result<u32> {
        self.ptr()
    }

    fn node_type(&mut self) -> Result<u16> {
        let v = self.int_exact("expected a node type")?;
        u16::try_from(v).map_err(|_| XtError::Parse {
            offset: self.pos,
            what: "node type out of range",
        })
    }

    fn varlen(&mut self) -> Result<u32> {
        self.ptr()
    }

    fn ch(&mut self) -> Result<char> {
        let b = self.bump()?;
        if b == b'\\' {
            let e = self.bump()?;
            Ok(match e {
                b'0' => '\0',
                b'n' => '\r', // per spec: "\n" escapes carriage return
                b'r' => '\n', // and "\r" escapes line feed
                b'\\' => '\\',
                _ => {
                    return parse_err(self.pos, "unknown escape in character field");
                }
            })
        } else {
            Ok(b as char)
        }
    }

    fn logical(&mut self) -> Result<bool> {
        match self.bump()? {
            b'T' | b'1' => Ok(true),
            b'F' | b'0' => Ok(false),
            _ => parse_err(self.pos, "expected logical T/F"),
        }
    }

    fn chars(&mut self, n: usize) -> Result<String> {
        let mut out = String::with_capacity(n);
        while out.len() < n {
            let b = self.bump()?;
            if b == b'\\' {
                match self.bump()? {
                    b'0' => out.push('\0'),
                    b'n' => out.push('\r'),
                    b'r' => out.push('\n'),
                    b'\\' => out.push('\\'),
                    b'9' => {
                        for _ in 0..9 {
                            out.push(' ');
                        }
                    }
                    _ => return parse_err(self.pos, "unknown escape in string"),
                }
            } else {
                out.push(b as char);
            }
        }
        if out.len() != n {
            return parse_err(self.pos, "string escape overshot declared length");
        }
        Ok(out)
    }

    fn short_string(&mut self) -> Result<String> {
        let n = self.positive()? as usize;
        self.chars(n)
    }

    fn vector(&mut self) -> Result<Option<[f64; 3]>> {
        self.skip_spaces();
        if self.peek() == Some(b'?') {
            self.pos += 1;
            return Ok(None);
        }
        let mut v = [0.0; 3];
        for c in &mut v {
            match self.double()? {
                Scalar::Double(x) => *c = x,
                Scalar::Null => return parse_err(self.pos, "partially null vector"),
                Scalar::Int(_) => unreachable!("double() never returns Int"),
            }
        }
        Ok(Some(v))
    }
}

// -------------------------------------------------------------- binary --

/// Cursor over the neutral binary encoding (big-endian IEEE).
pub struct BinCursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> BinCursor<'a> {
    /// New cursor over the data section (everything after the `PS\0\0`
    /// flag).
    pub fn new(data: &'a [u8]) -> BinCursor<'a> {
        BinCursor { data, pos: 0 }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8]> {
        let end = self.pos + n;
        if end > self.data.len() {
            return parse_err(self.pos, "unexpected end of binary data");
        }
        let s = &self.data[self.pos..end];
        self.pos = end;
        Ok(s)
    }

    fn i16be(&mut self) -> Result<i16> {
        let b = self.take(2)?;
        Ok(i16::from_be_bytes([b[0], b[1]]))
    }

    fn i32be(&mut self) -> Result<i32> {
        let b = self.take(4)?;
        Ok(i32::from_be_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn f64be(&mut self) -> Result<f64> {
        let b = self.take(8)?;
        Ok(f64::from_be_bytes([
            b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7],
        ]))
    }

    /// The variable short encoding shared by pointers, node indices, and
    /// positive integers.
    fn short_index(&mut self) -> Result<u32> {
        let r = self.i16be()?;
        if r >= 0 {
            if r == 0 {
                return parse_err(self.pos, "zero in short-index encoding");
            }
            Ok(r as u32 - 1)
        } else {
            let q = self.i16be()?;
            if q <= 0 {
                return parse_err(self.pos, "bad quotient in short-index encoding");
            }
            Ok(q as u32 * 32767 + (-r) as u32 - 1)
        }
    }
}

impl Cursor for BinCursor<'_> {
    fn offset(&self) -> usize {
        self.pos
    }

    fn at_end(&mut self) -> bool {
        self.pos >= self.data.len()
    }

    fn int(&mut self) -> Result<Scalar> {
        let v = self.i32be()?;
        Ok(if v == NULL_INT {
            Scalar::Null
        } else {
            Scalar::Int(v as i64)
        })
    }

    fn double(&mut self) -> Result<Scalar> {
        let v = self.f64be()?;
        Ok(if v == NULL_DOUBLE {
            Scalar::Null
        } else {
            Scalar::Double(v)
        })
    }

    fn short(&mut self) -> Result<i32> {
        Ok(self.i16be()? as i32)
    }

    fn byte(&mut self) -> Result<u32> {
        Ok(self.take(1)?[0] as u32)
    }

    fn ptr(&mut self) -> Result<u32> {
        self.short_index()
    }

    fn positive(&mut self) -> Result<u32> {
        self.short_index()
    }

    fn node_type(&mut self) -> Result<u16> {
        let v = self.i16be()?;
        u16::try_from(v).map_err(|_| XtError::Parse {
            offset: self.pos,
            what: "negative node type",
        })
    }

    fn varlen(&mut self) -> Result<u32> {
        let v = self.i32be()?;
        u32::try_from(v).map_err(|_| XtError::Parse {
            offset: self.pos,
            what: "negative variable length",
        })
    }

    fn ch(&mut self) -> Result<char> {
        Ok(self.take(1)?[0] as char)
    }

    fn logical(&mut self) -> Result<bool> {
        match self.take(1)?[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => parse_err(self.pos, "logical byte not 0/1"),
        }
    }

    fn chars(&mut self, n: usize) -> Result<String> {
        Ok(self.take(n)?.iter().map(|&b| b as char).collect())
    }

    fn vector(&mut self) -> Result<Option<[f64; 3]>> {
        let x = self.f64be()?;
        let y = self.f64be()?;
        let z = self.f64be()?;
        if x == NULL_DOUBLE && y == NULL_DOUBLE && z == NULL_DOUBLE {
            Ok(None)
        } else {
            Ok(Some([x, y, z]))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_numbers_chars_and_nulls() {
        // Mirrors real transmit content: numbers with single trailing
        // spaces, chars glued to numbers, ? nulls, F/T logicals, and a
        // newline splitting a token.
        let mut c = TextCursor::new(b"16 7 6 0 ?10 0 0 5e-4 V1e\n3 FT");
        assert_eq!(c.node_type().unwrap(), 16);
        assert_eq!(c.ptr().unwrap(), 7);
        assert_eq!(c.int().unwrap(), Scalar::Int(6));
        assert_eq!(c.int().unwrap(), Scalar::Int(0));
        assert_eq!(c.double().unwrap(), Scalar::Null);
        assert_eq!(c.ptr().unwrap(), 10);
        assert_eq!(c.ptr().unwrap(), 0);
        assert_eq!(c.ptr().unwrap(), 0);
        assert_eq!(c.double().unwrap(), Scalar::Double(5e-4));
        assert_eq!(c.ch().unwrap(), 'V');
        assert_eq!(c.double().unwrap(), Scalar::Double(1e3));
        assert!(!c.logical().unwrap());
        assert!(c.logical().unwrap());
        assert!(c.at_end());
    }

    #[test]
    fn text_short_string_and_null_vector() {
        let mut c = TextCursor::new(b"5 owner1040 0 ?3 1 2 ");
        assert_eq!(c.short_string().unwrap(), "owner");
        assert_eq!(c.ptr().unwrap(), 1040);
        assert_eq!(c.ptr().unwrap(), 0);
        assert_eq!(c.vector().unwrap(), None);
        assert_eq!(c.vector().unwrap(), Some([3.0, 1.0, 2.0]));
    }

    #[test]
    fn binary_short_index_roundtrip() {
        // value < 32767: one short (value + 1); larger: remainder pair.
        let enc_small = 41i16.to_be_bytes();
        let mut c = BinCursor::new(&enc_small);
        assert_eq!(c.ptr().unwrap(), 40);
        let big: u32 = 5 * 32767 + 122;
        let r = (-((big % 32767 + 1) as i16)).to_be_bytes();
        let q = ((big / 32767) as i16).to_be_bytes();
        let bytes = [r[0], r[1], q[0], q[1]];
        let mut c = BinCursor::new(&bytes);
        assert_eq!(c.ptr().unwrap(), big);
    }

    #[test]
    fn binary_null_sentinels() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&NULL_INT.to_be_bytes());
        bytes.extend_from_slice(&NULL_DOUBLE.to_be_bytes());
        let mut c = BinCursor::new(&bytes);
        assert_eq!(c.int().unwrap(), Scalar::Null);
        assert_eq!(c.double().unwrap(), Scalar::Null);
    }
}
