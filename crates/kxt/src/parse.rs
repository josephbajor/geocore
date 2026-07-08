//! Transmit-file parsing: header, flag sequence, embedded schemas, and the
//! node stream, producing an [`XtFile`] node graph.
//!
//! One empirical deviation from the 2006 Format Reference, validated
//! against files written by modern Parasolid versions (V27, V28): when
//! `USFLD_SIZE > 0`, user fields follow **every** node, not only nodes
//! visible at the PK interface.

use crate::cursor::{BinCursor, Cursor, Scalar, TextCursor};
use crate::error::{Result, XtError};
use crate::schema::{self, EditOp, EmbeddedField, FieldType, NodeDef};
use std::collections::BTreeMap;

/// Runaway-input backstop: a Tier-0 part file will not have this many
/// nodes.
const MAX_NODES: usize = 4_000_000;

/// One field value of a parsed node.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// Unset (`?` / sentinel).
    Null,
    /// Integral value (int, short, byte).
    Int(i64),
    /// Floating-point value.
    Double(f64),
    /// Character.
    Char(char),
    /// Logical.
    Logical(bool),
    /// Pointer index (0 = null pointer).
    Ptr(u32),
    /// Vector (`None` = null vector).
    Vector(Option<[f64; 3]>),
    /// Interval (`None` = both ends unset).
    Interval(Option<[f64; 2]>),
    /// String data (char arrays).
    Str(String),
    /// Fixed or variable array of scalar values.
    Arr(Vec<Value>),
}

impl Value {
    /// The pointer index, if this is a pointer.
    pub fn as_ptr(&self) -> Option<u32> {
        match self {
            Value::Ptr(p) => Some(*p),
            _ => None,
        }
    }
    /// The numeric value as f64, if numeric (`None` for `Null`).
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Double(v) => Some(*v),
            Value::Int(v) => Some(*v as f64),
            _ => None,
        }
    }
    /// The integral value, if integral.
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Value::Int(v) => Some(*v),
            _ => None,
        }
    }
    /// The character, if a char field.
    pub fn as_char(&self) -> Option<char> {
        match self {
            Value::Char(c) => Some(*c),
            _ => None,
        }
    }
    /// The vector, if a (non-null) vector field.
    pub fn as_vector(&self) -> Option<[f64; 3]> {
        match self {
            Value::Vector(v) => *v,
            _ => None,
        }
    }
}

/// A parsed node: its type and field values in schema order.
#[derive(Debug, Clone)]
pub struct Node {
    /// Node type code.
    pub code: u16,
    /// Field values, parallel to the file's [`NodeDef`] for `code`.
    pub values: Vec<Value>,
}

/// The `**PART1/2/3` header key-value data.
#[derive(Debug, Clone, Default)]
pub struct Header {
    /// All `NAME=value` pairs from parts 1–3, in file order.
    pub pairs: Vec<(String, String)>,
}

impl Header {
    /// Look up a header key (first occurrence).
    pub fn get(&self, key: &str) -> Option<&str> {
        self.pairs
            .iter()
            .find(|(k, _)| k == key)
            .map(|(_, v)| v.as_str())
    }
}

/// A parsed transmit file: the node graph plus its schema context.
pub struct XtFile {
    /// Header key-values (`FORMAT`, `SCH`, …).
    pub header: Header,
    /// The full schema key from the data section, e.g.
    /// `SCH_2700142_26105_13006`.
    pub schema: String,
    /// User-field size (integers appended to nodes).
    pub usfld_size: usize,
    /// Nodes by index.
    pub nodes: BTreeMap<u32, Node>,
    /// The per-file node layouts (base schema plus embedded edits).
    pub defs: BTreeMap<u16, NodeDef>,
    /// Node types that appeared in the file but are not part of the base
    /// schema (they parsed via their embedded descriptions and are
    /// ignored by Tier-0 reconstruction).
    pub foreign_codes: Vec<u16>,
}

impl XtFile {
    /// The node at `index`, if present.
    pub fn node(&self, index: u32) -> Option<&Node> {
        self.nodes.get(&index)
    }

    /// A named field of a node, resolved through the file's layout.
    pub fn field<'a>(&'a self, node: &'a Node, name: &str) -> Option<&'a Value> {
        let def = self.defs.get(&node.code)?;
        node.values.get(def.field_index(name)?)
    }
}

/// Split the common `**…` header from the data section and parse its
/// key-value pairs. Returns (header, data-section bytes).
fn split_header(bytes: &[u8]) -> Result<(Header, &[u8])> {
    const END: &[u8] = b"**END_OF_HEADER";
    let start = bytes
        .windows(END.len())
        .position(|w| w == END)
        .ok_or(XtError::BadHeader {
            what: "missing **END_OF_HEADER",
        })?;
    // Skip the trailer record: asterisks, then the record's line break.
    let mut pos = start + END.len();
    while pos < bytes.len() && bytes[pos] == b'*' {
        pos += 1;
    }
    while pos < bytes.len() && (bytes[pos] == b'\n' || bytes[pos] == b'\r') {
        pos += 1;
    }
    let mut header = Header::default();
    let text: String = bytes[..start]
        .iter()
        .map(|&b| b as char)
        .filter(|&c| c != '\n' && c != '\r')
        .collect();
    for chunk in text.split(';') {
        let chunk = chunk.trim_start_matches('*');
        if let Some(eq) = chunk.find('=') {
            let (k, v) = chunk.split_at(eq);
            let key_ok = !k.is_empty()
                && k.bytes()
                    .all(|b| b.is_ascii_uppercase() || b == b'_' || b.is_ascii_digit());
            if key_ok {
                header.pairs.push((k.to_owned(), v[1..].to_owned()));
            }
        }
    }
    Ok((header, &bytes[pos..]))
}

fn parse_schema_key(key: &str) -> Result<(u32, Option<u32>)> {
    let err = || XtError::UnsupportedSchema {
        schema: key.to_owned(),
    };
    let rest = key.strip_prefix("SCH_").ok_or_else(err)?;
    let parts: Vec<&str> = rest.split('_').collect();
    match parts.as_slice() {
        [_, schema] => Ok((schema.parse().map_err(|_| err())?, None)),
        [_, schema, base] => Ok((
            schema.parse().map_err(|_| err())?,
            Some(base.parse().map_err(|_| err())?),
        )),
        _ => Err(err()),
    }
}

/// Parse a complete transmit file (text or neutral binary).
pub fn read_xt(bytes: &[u8]) -> Result<XtFile> {
    let (header, data) = split_header(bytes)?;
    match data.first() {
        Some(b'T') => parse_stream(header, &mut TextCursor::new(&data[1..])),
        Some(b'P') if data.get(1..4) == Some(&b"S\0\0"[..]) => {
            parse_stream(header, &mut BinCursor::new(&data[4..]))
        }
        Some(b'B') => Err(XtError::Unsupported {
            what: "bare binary transmit files (machine-dependent format)",
        }),
        _ => Err(XtError::BadHeader {
            what: "unrecognized format flag after header",
        }),
    }
}

struct Parser<'a, C: Cursor> {
    cur: &'a mut C,
    base: BTreeMap<u16, NodeDef>,
    defs: BTreeMap<u16, NodeDef>,
    embedded: bool,
    usfld: usize,
    foreign: Vec<u16>,
}

fn parse_stream<C: Cursor>(header: Header, cur: &mut C) -> Result<XtFile> {
    // Flag sequence: modeller version (2-byte length + chars), schema key
    // (4-byte length + chars), then — for embedded-schema files — the
    // maximum node-type count (short), and finally the user-field size
    // (4-byte int). Text writes all lengths/counts as plain numbers.
    let version_len = cur.node_type()? as usize;
    let _version = cur.chars(version_len)?;
    let schema_len = cur.varlen()? as usize;
    let schema = cur.chars(schema_len)?;
    let (schema_num, base_num) = parse_schema_key(&schema)?;
    let embedded = match base_num {
        Some(13006) => true,
        None if schema_num == 13006 => false,
        _ => {
            return Err(XtError::UnsupportedSchema { schema });
        }
    };
    if embedded {
        let _max_node_types = cur.node_type()?;
    }
    let usfld_size = cur.varlen()? as usize;

    let base: BTreeMap<u16, NodeDef> = schema::base_schema()
        .into_iter()
        .map(|d| (d.code, d))
        .collect();
    let mut p = Parser {
        cur,
        base,
        defs: BTreeMap::new(),
        embedded,
        usfld: usfld_size,
        foreign: Vec::new(),
    };
    let mut nodes = BTreeMap::new();
    loop {
        let code = p.cur.node_type()?;
        if code == 1 {
            // Terminator: type value 1 followed by index 0.
            let idx = p.cur.ptr()?;
            if idx != 0 {
                return Err(XtError::Parse {
                    offset: p.cur.offset(),
                    what: "terminator index is not 0",
                });
            }
            break;
        }
        let (index, node) = p.node(code)?;
        if nodes.insert(index, node).is_some() {
            return Err(XtError::Parse {
                offset: p.cur.offset(),
                what: "duplicate node index",
            });
        }
        if nodes.len() > MAX_NODES {
            return Err(XtError::Parse {
                offset: p.cur.offset(),
                what: "node count exceeds sanity limit",
            });
        }
    }
    Ok(XtFile {
        header,
        schema,
        usfld_size,
        nodes,
        defs: p.defs,
        foreign_codes: p.foreign,
    })
}

impl<C: Cursor> Parser<'_, C> {
    /// Resolve the layout for `code`, consuming embedded-schema info on
    /// its first occurrence.
    fn resolve_def(&mut self, code: u16) -> Result<()> {
        if self.defs.contains_key(&code) {
            return Ok(());
        }
        let def = if self.embedded {
            match self.base.get(&code).cloned() {
                Some(base) => {
                    let n = self.cur.byte()?;
                    if n == 255 {
                        base
                    } else {
                        let ops = self.edit_script()?;
                        schema::apply_edits(&base, n as usize, &ops)?
                    }
                }
                None => {
                    // Full description of a node type the base schema
                    // does not know.
                    let n_fields = self.cur.byte()? as usize;
                    let name = self.cur.short_string()?;
                    let _description = self.cur.short_string()?;
                    let mut fields = Vec::with_capacity(n_fields);
                    for _ in 0..n_fields {
                        if let Some(spec) = self.embedded_field()?.to_spec()? {
                            fields.push(spec);
                        }
                    }
                    self.foreign.push(code);
                    NodeDef { code, name, fields }
                }
            }
        } else {
            self.base
                .get(&code)
                .cloned()
                .ok_or(XtError::UnknownNodeType { code })?
        };
        self.defs.insert(code, def);
        Ok(())
    }

    /// One field of an embedded-schema description: name, ptr_class,
    /// n_elts, then type (absent for pointers) and transmit flag (absent
    /// for fixed-length fields).
    fn embedded_field(&mut self) -> Result<EmbeddedField> {
        let name = self.cur.short_string()?;
        let ptr_class = {
            let s = self.cur.short()?;
            u32::try_from(s).map_err(|_| XtError::Parse {
                offset: self.cur.offset(),
                what: "negative pointer class",
            })?
        };
        let n_elts = self.cur.positive()?;
        let ty = if ptr_class == 0 {
            self.cur.short_string()?
        } else {
            String::new()
        };
        let xmt = if n_elts == 1 {
            self.cur.logical()?
        } else {
            true
        };
        Ok(EmbeddedField {
            name,
            ptr_class,
            n_elts,
            ty,
            xmt,
        })
    }

    /// The C/D/I/A/Z edit script of an embedded schema.
    fn edit_script(&mut self) -> Result<Vec<EditOp>> {
        let mut ops = Vec::new();
        loop {
            match self.cur.ch()? {
                'C' => ops.push(EditOp::Copy),
                'D' => ops.push(EditOp::Delete),
                'I' => ops.push(EditOp::Insert(self.embedded_field()?)),
                'A' => ops.push(EditOp::Append(self.embedded_field()?)),
                'Z' => return Ok(ops),
                _ => {
                    return Err(XtError::Parse {
                        offset: self.cur.offset(),
                        what: "unknown embedded-schema edit instruction",
                    });
                }
            }
        }
    }

    /// Parse one node (after its type code was read).
    fn node(&mut self, code: u16) -> Result<(u32, Node)> {
        self.resolve_def(code)?;
        let def = self.defs.get(&code).expect("just resolved").clone();
        let varlen = if def.is_variable() {
            self.cur.varlen()? as usize
        } else {
            0
        };
        let index = self.cur.ptr()?;
        if index == 0 {
            return Err(XtError::Parse {
                offset: self.cur.offset(),
                what: "node index 0",
            });
        }
        let mut values = Vec::with_capacity(def.fields.len());
        for f in &def.fields {
            let count = match f.n_elts {
                0 => {
                    values.push(self.scalar(f.ty)?);
                    continue;
                }
                1 => varlen,
                n => n as usize,
            };
            if f.ty == FieldType::Char {
                values.push(Value::Str(self.cur.chars(count)?));
            } else {
                let mut arr = Vec::with_capacity(count);
                for _ in 0..count {
                    arr.push(self.scalar(f.ty)?);
                }
                values.push(Value::Arr(arr));
            }
        }
        // Empirical rule (see module docs): user fields follow every node.
        for _ in 0..self.usfld {
            let _ = self.cur.int()?;
        }
        Ok((index, Node { code, values }))
    }

    fn scalar(&mut self, ty: FieldType) -> Result<Value> {
        Ok(match ty {
            FieldType::Byte => Value::Int(self.cur.byte()? as i64),
            FieldType::Char => Value::Char(self.cur.ch()?),
            FieldType::Logical => Value::Logical(self.cur.logical()?),
            FieldType::Short | FieldType::Unicode => Value::Int(self.cur.short()? as i64),
            FieldType::Int => match self.cur.int()? {
                Scalar::Int(v) => Value::Int(v),
                Scalar::Null => Value::Null,
                Scalar::Double(_) => unreachable!("int() never yields Double"),
            },
            FieldType::Ptr => Value::Ptr(self.cur.ptr()?),
            FieldType::Double => match self.cur.double()? {
                Scalar::Double(v) => Value::Double(v),
                Scalar::Null => Value::Null,
                Scalar::Int(_) => unreachable!("double() never yields Int"),
            },
            FieldType::Interval => {
                let lo = self.cur.double()?;
                let hi = self.cur.double()?;
                match (lo, hi) {
                    (Scalar::Double(a), Scalar::Double(b)) => Value::Interval(Some([a, b])),
                    _ => Value::Interval(None),
                }
            }
            // Only the position vector of an hvec is transmitted.
            FieldType::Vector | FieldType::Hvec => Value::Vector(self.cur.vector()?),
            FieldType::Box6 => {
                let mut b = [0.0; 6];
                for x in &mut b {
                    match self.cur.double()? {
                        Scalar::Double(v) => *x = v,
                        _ => {
                            return Err(XtError::Parse {
                                offset: self.cur.offset(),
                                what: "null component in box field",
                            });
                        }
                    }
                }
                Value::Arr(b.iter().map(|&v| Value::Double(v)).collect())
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The spec's own worked example (a sheet circle with a colour
    /// attribute), transcribed to base schema 13006: the ATTRIB_DEF gains
    /// the `field_names` pointer and a 14th `legal_owners` entry.
    pub const SHEET_CIRCLE: &str = concat!(
        "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz",
        "**************************\n",
        "**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789",
        "**************************\n",
        "**PART1;MC=unknown;APPL=kxt-tests;FORMAT=text;GUISE=transmit;\n",
        "**PART2;SCH=SCH_1300000_13006;USFLD_SIZE=0;\n",
        "**PART3;\n",
        "**END_OF_HEADER*****************************************************\n",
        "T51 : TRANSMIT FILE created by modeller version 1300000 ",
        "17 SCH_1300000_13006 0 ",
        "12 1 12 0 2 0 0 0 0 1e3 1e-8 0 0 0 1 0 3 1 3 4 5 0 6 7 0 ",
        "70 2 0 1 0 0 4 1 20 8 8 8 1 T",
        "13 3 3 0 1 0 9 0 0 6 9 ",
        "50 4 11 0 9 0 0 0 +0 0 0 0 0 1 1 0 0 ",
        "31 5 10 0 7 0 0 0 +0 0 0 0 0 1 1 0 0 1 ",
        "19 6 5 0 1 0 0 3 V",
        "16 7 6 0 ?10 0 0 5 0 0 1 ",
        "17 10 0 11 10 10 0 12 7 0 0 +",
        "15 11 7 0 10 9 0 ",
        "17 12 0 0 0 0 0 10 7 0 0 -",
        "14 9 2 13 ?0 0 11 3 4 +0 0 0 0 3 ",
        "81 1 13 12 14 9 0 0 0 0 15 ",
        "80 1 14 0 16 8001 0 0 0 0 3 5 0 0 0 FFFFTFTFFFFFFF2 ",
        "83 3 15 1 2 3 ",
        "79 15 16 SDL/TYSA_COLOUR",
        "74 20 8 1 0 13 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ",
        "1 0 ",
    );

    #[test]
    fn parses_the_spec_example() {
        let file = read_xt(SHEET_CIRCLE.as_bytes()).unwrap();
        assert_eq!(file.header.get("FORMAT"), Some("text"));
        assert_eq!(file.schema, "SCH_1300000_13006");
        assert_eq!(file.usfld_size, 0);
        assert_eq!(file.nodes.len(), 16);

        let body = file.node(1).unwrap();
        assert_eq!(body.code, schema::code::BODY);
        assert_eq!(file.field(body, "body_type").unwrap().as_int(), Some(3));
        assert_eq!(file.field(body, "res_size").unwrap().as_f64(), Some(1e3));
        assert_eq!(file.field(body, "region").unwrap().as_ptr(), Some(6));

        let circle = file.node(5).unwrap();
        assert_eq!(circle.code, schema::code::CIRCLE);
        assert_eq!(file.field(circle, "radius").unwrap().as_f64(), Some(1.0));
        assert_eq!(
            file.field(circle, "normal").unwrap().as_vector(),
            Some([0.0, 0.0, 1.0])
        );

        let edge = file.node(7).unwrap();
        assert_eq!(*file.field(edge, "tolerance").unwrap(), Value::Null);
        assert_eq!(file.field(edge, "curve").unwrap().as_ptr(), Some(5));

        let fin = file.node(10).unwrap();
        assert_eq!(file.field(fin, "sense").unwrap().as_char(), Some('+'));
        assert_eq!(file.field(fin, "edge").unwrap().as_ptr(), Some(7));

        let att_id = file.node(16).unwrap();
        assert_eq!(
            *file.field(att_id, "string").unwrap(),
            Value::Str("SDL/TYSA_COLOUR".to_owned())
        );

        let region = file.node(6).unwrap();
        assert_eq!(file.field(region, "type").unwrap().as_char(), Some('V'));
    }
}
