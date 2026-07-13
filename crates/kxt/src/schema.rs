//! The XT node schema: field layouts per node type.
//!
//! The wire format of every node is dictated by a *schema*. This module
//! hard-codes the base schema **13006** (Parasolid V13) exactly as
//! documented in the published *XT Format Reference*, and implements the
//! *embedded schema* mechanism by which every later Parasolid version
//! describes its layout as an edit script against 13006 in the transmit
//! file itself. Files therefore parse without any external schema files:
//!
//! - a file written at exactly schema 13006 uses the base table directly;
//! - a file whose schema key carries the base suffix (`SCH_x_y_13006`)
//!   carries, before the first node of each type, either the flag 255
//!   ("same as base") or a Copy/Delete/Insert/Append edit script — or a
//!   full description for node types the base schema does not know.

use crate::error::{Result, XtError};

/// Scalar wire types of node fields (the spec's `$` codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FieldType {
    /// `u` — unsigned byte.
    Byte,
    /// `c` — character.
    Char,
    /// `l` — logical (0/1, written `F`/`T` in text).
    Logical,
    /// `n` — short int.
    Short,
    /// `w` — unicode character (short int on the wire).
    Unicode,
    /// `d` — int.
    Int,
    /// `p` — pointer index (0 = null).
    Ptr,
    /// `f` — double.
    Double,
    /// `i` — interval (two doubles).
    Interval,
    /// `v` — vector (three doubles).
    Vector,
    /// `b` — box (six doubles).
    Box6,
    /// `h` — hvec; only the position vector is transmitted.
    Hvec,
}

impl FieldType {
    fn from_code(c: u8) -> Option<FieldType> {
        Some(match c {
            b'u' => FieldType::Byte,
            b'c' => FieldType::Char,
            b'l' => FieldType::Logical,
            b'n' => FieldType::Short,
            b'w' => FieldType::Unicode,
            b'd' => FieldType::Int,
            b'p' => FieldType::Ptr,
            b'f' => FieldType::Double,
            b'i' => FieldType::Interval,
            b'v' => FieldType::Vector,
            b'b' => FieldType::Box6,
            b'h' => FieldType::Hvec,
            _ => return None,
        })
    }
}

/// Element count semantics: 0 = scalar, 1 = variable-length (must be the
/// last field), n > 1 = fixed array.
pub type NElts = u32;

/// One transmitted field of a node type.
#[derive(Debug, Clone, PartialEq)]
pub struct FieldSpec {
    /// Field name (the reconstruction layer addresses fields by name).
    pub name: String,
    /// Wire type.
    pub ty: FieldType,
    /// 0 scalar, 1 variable, n array.
    pub n_elts: NElts,
}

/// The transmitted layout of one node type.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeDef {
    /// Numeric node type.
    pub code: u16,
    /// Node type name (diagnostics only).
    pub name: String,
    /// Transmitted fields, in wire order.
    pub fields: Vec<FieldSpec>,
}

impl NodeDef {
    /// True if the last field is variable-length (the node is preceded by
    /// an explicit length on the wire).
    pub fn is_variable(&self) -> bool {
        self.fields.last().is_some_and(|f| f.n_elts == 1)
    }

    /// Index of a field by name.
    pub fn field_index(&self, name: &str) -> Option<usize> {
        self.fields.iter().position(|f| f.name == name)
    }
}

/// Well-known node type codes (base schema 13006).
pub mod code {
    /// ASSEMBLY node type.
    pub const ASSEMBLY: u16 = 10;
    /// INSTANCE node type.
    pub const INSTANCE: u16 = 11;
    /// BODY node type.
    pub const BODY: u16 = 12;
    /// SHELL node type.
    pub const SHELL: u16 = 13;
    /// FACE node type.
    pub const FACE: u16 = 14;
    /// LOOP node type.
    pub const LOOP: u16 = 15;
    /// EDGE node type.
    pub const EDGE: u16 = 16;
    /// FIN node type.
    pub const FIN: u16 = 17;
    /// VERTEX node type.
    pub const VERTEX: u16 = 18;
    /// REGION node type.
    pub const REGION: u16 = 19;
    /// POINT node type.
    pub const POINT: u16 = 29;
    /// LINE node type.
    pub const LINE: u16 = 30;
    /// CIRCLE node type.
    pub const CIRCLE: u16 = 31;
    /// ELLIPSE node type.
    pub const ELLIPSE: u16 = 32;
    /// INTERSECTION curve node type.
    pub const INTERSECTION: u16 = 38;
    /// CHART node type.
    pub const CHART: u16 = 40;
    /// LIMIT node type.
    pub const LIMIT: u16 = 41;
    /// Modern embedded-schema INTERSECTION_DATA node type used by the
    /// supported transmitted chart subset. It is not part of base 13006.
    pub const INTERSECTION_DATA: u16 = 204;
    /// BSPLINE_VERTICES node type.
    pub const BSPLINE_VERTICES: u16 = 45;
    /// PLANE node type.
    pub const PLANE: u16 = 50;
    /// CYLINDER node type.
    pub const CYLINDER: u16 = 51;
    /// CONE node type.
    pub const CONE: u16 = 52;
    /// SPHERE node type.
    pub const SPHERE: u16 = 53;
    /// TORUS node type.
    pub const TORUS: u16 = 54;
    /// BLENDED_EDGE node type.
    pub const BLENDED_EDGE: u16 = 56;
    /// BLEND_BOUND node type.
    pub const BLEND_BOUND: u16 = 59;
    /// OFFSET_SURF node type.
    pub const OFFSET_SURF: u16 = 60;
    /// SWEPT_SURF node type.
    pub const SWEPT_SURF: u16 = 67;
    /// SPUN_SURF node type.
    pub const SPUN_SURF: u16 = 68;
    /// LIST node type.
    pub const LIST: u16 = 70;
    /// POINTER_LIS_BLOCK node type.
    pub const POINTER_LIS_BLOCK: u16 = 74;
    /// ATT_DEF_ID node type.
    pub const ATT_DEF_ID: u16 = 79;
    /// ATTRIB_DEF node type.
    pub const ATTRIB_DEF: u16 = 80;
    /// ATTRIBUTE node type.
    pub const ATTRIBUTE: u16 = 81;
    /// INT_VALUES node type.
    pub const INT_VALUES: u16 = 82;
    /// REAL_VALUES node type.
    pub const REAL_VALUES: u16 = 83;
    /// CHAR_VALUES node type.
    pub const CHAR_VALUES: u16 = 84;
    /// POINT_VALUES node type.
    pub const POINT_VALUES: u16 = 85;
    /// VECTOR_VALUES node type.
    pub const VECTOR_VALUES: u16 = 86;
    /// AXIS_VALUES node type.
    pub const AXIS_VALUES: u16 = 87;
    /// TAG_VALUES node type.
    pub const TAG_VALUES: u16 = 88;
    /// DIRECTION_VALUES node type.
    pub const DIRECTION_VALUES: u16 = 89;
    /// GROUP node type.
    pub const GROUP: u16 = 90;
    /// MEMBER_OF_GROUP node type.
    pub const MEMBER_OF_GROUP: u16 = 91;
    /// UNICODE_VALUES node type.
    pub const UNICODE_VALUES: u16 = 98;
    /// FIELD_NAMES node type.
    pub const FIELD_NAMES: u16 = 99;
    /// TRANSFORM node type.
    pub const TRANSFORM: u16 = 100;
    /// WORLD node type.
    pub const WORLD: u16 = 101;
    /// KEY node type.
    pub const KEY: u16 = 102;
    /// PE_SURF (foreign geometry surface) node type.
    pub const PE_SURF: u16 = 120;
    /// B_SURFACE node type.
    pub const B_SURFACE: u16 = 124;
    /// SURFACE_DATA node type.
    pub const SURFACE_DATA: u16 = 125;
    /// NURBS_SURF node type.
    pub const NURBS_SURF: u16 = 126;
    /// KNOT_MULT node type.
    pub const KNOT_MULT: u16 = 127;
    /// KNOT_SET node type.
    pub const KNOT_SET: u16 = 128;
    /// PE_CURVE (foreign geometry curve) node type.
    pub const PE_CURVE: u16 = 130;
    /// TRIMMED_CURVE node type.
    pub const TRIMMED_CURVE: u16 = 133;
    /// B_CURVE node type.
    pub const B_CURVE: u16 = 134;
    /// CURVE_DATA node type.
    pub const CURVE_DATA: u16 = 135;
    /// NURBS_CURVE node type.
    pub const NURBS_CURVE: u16 = 136;
    /// SP_CURVE node type.
    pub const SP_CURVE: u16 = 137;
    /// GEOMETRIC_OWNER node type.
    pub const GEOMETRIC_OWNER: u16 = 141;
}

/// Parse one entry of the compact layout DSL: `name:t` (scalar),
/// `name:t:8` (array of 8), `name:t:V` (variable-length).
fn field(entry: &str) -> FieldSpec {
    let mut parts = entry.split(':');
    let name = parts.next().expect("field name");
    let ty = parts.next().expect("field type");
    debug_assert_eq!(ty.len(), 1);
    let ty = FieldType::from_code(ty.as_bytes()[0]).expect("known field type code");
    let n_elts = match parts.next() {
        None => 0,
        Some("V") => 1,
        Some(n) => n.parse().expect("array length"),
    };
    FieldSpec {
        name: name.to_owned(),
        ty,
        n_elts,
    }
}

fn def(code: u16, name: &str, layout: &str) -> NodeDef {
    NodeDef {
        code,
        name: name.to_owned(),
        fields: layout.split_whitespace().map(field).collect(),
    }
}

/// The seven fields shared by every curve and surface node.
const GEOM_COMMON: &str =
    "node_id:d attributes_groups:p owner:p next:p previous:p geometric_owner:p sense:c";

/// Build the base schema 13006 table: layouts transcribed from the
/// published XT Format Reference and empirically validated against real
/// transmit files (see `tests/`).
pub fn base_schema() -> Vec<NodeDef> {
    let g = |rest: &str| format!("{GEOM_COMMON} {rest}");
    vec![
        def(
            code::ASSEMBLY,
            "ASSEMBLY",
            "highest_node_id:d attributes_groups:p attribute_chains:p list:p surface:p curve:p \
             point:p key:p res_size:f res_linear:f ref_instance:p next:p previous:p state:u \
             owner:p type:u sub_instance:p",
        ),
        def(
            code::INSTANCE,
            "INSTANCE",
            "node_id:d attributes_groups:p type:u part:p transform:p assembly:p next_in_part:p \
             prev_in_part:p next_of_part:p prev_of_part:p",
        ),
        def(
            code::BODY,
            "BODY",
            "highest_node_id:d attributes_groups:p attribute_chains:p surface:p curve:p point:p \
             key:p res_size:f res_linear:f ref_instance:p next:p previous:p state:u owner:p \
             body_type:u nom_geom_state:u shell:p boundary_surface:p boundary_curve:p \
             boundary_point:p region:p edge:p vertex:p",
        ),
        def(
            code::SHELL,
            "SHELL",
            "node_id:d attributes_groups:p body:p next:p face:p edge:p vertex:p region:p \
             front_face:p",
        ),
        def(
            code::FACE,
            "FACE",
            "node_id:d attributes_groups:p tolerance:f next:p previous:p loop:p shell:p \
             surface:p sense:c next_on_surface:p previous_on_surface:p next_front:p \
             previous_front:p front_shell:p",
        ),
        def(
            code::LOOP,
            "LOOP",
            "node_id:d attributes_groups:p fin:p face:p next:p",
        ),
        def(
            code::EDGE,
            "EDGE",
            "node_id:d attributes_groups:p tolerance:f fin:p previous:p next:p curve:p \
             next_on_curve:p previous_on_curve:p owner:p",
        ),
        def(
            code::FIN,
            "FIN",
            "attributes_groups:p loop:p forward:p backward:p vertex:p other:p edge:p curve:p \
             next_at_vx:p sense:c",
        ),
        def(
            code::VERTEX,
            "VERTEX",
            "node_id:d attributes_groups:p fin:p previous:p next:p point:p tolerance:f owner:p",
        ),
        def(
            code::REGION,
            "REGION",
            "node_id:d attributes_groups:p body:p next:p previous:p shell:p type:c",
        ),
        def(
            code::POINT,
            "POINT",
            "node_id:d attributes_groups:p owner:p next:p previous:p pvec:v",
        ),
        def(code::LINE, "LINE", &g("pvec:v direction:v")),
        def(
            code::CIRCLE,
            "CIRCLE",
            &g("centre:v normal:v x_axis:v radius:f"),
        ),
        def(
            code::ELLIPSE,
            "ELLIPSE",
            &g("centre:v normal:v x_axis:v major_radius:f minor_radius:f"),
        ),
        def(
            code::INTERSECTION,
            "INTERSECTION",
            &g("surface:p:2 chart:p start:p end:p"),
        ),
        def(
            code::CHART,
            "CHART",
            "base_parameter:f base_scale:f chart_count:d chordal_error:f angular_error:f \
             parameter_error:f:2 hvec:h:V",
        ),
        def(code::LIMIT, "LIMIT", "type:c hvec:h:V"),
        def(code::BSPLINE_VERTICES, "BSPLINE_VERTICES", "vertices:f:V"),
        def(code::PLANE, "PLANE", &g("pvec:v normal:v x_axis:v")),
        def(
            code::CYLINDER,
            "CYLINDER",
            &g("pvec:v axis:v radius:f x_axis:v"),
        ),
        def(
            code::CONE,
            "CONE",
            &g("pvec:v axis:v radius:f sin_half_angle:f cos_half_angle:f x_axis:v"),
        ),
        def(
            code::SPHERE,
            "SPHERE",
            &g("centre:v radius:f axis:v x_axis:v"),
        ),
        def(
            code::TORUS,
            "TORUS",
            &g("centre:v axis:v major_radius:f minor_radius:f x_axis:v"),
        ),
        def(
            code::BLENDED_EDGE,
            "BLENDED_EDGE",
            &g(
                "blend_type:c surface:p:2 spine:p range:f:2 thumb_weight:f:2 boundary:p:2 \
                start:p end:p",
            ),
        ),
        def(code::BLEND_BOUND, "BLEND_BOUND", &g("boundary:n blend:p")),
        def(
            code::OFFSET_SURF,
            "OFFSET_SURF",
            &g("check:c true_offset:l surface:p offset:f scale:f"),
        ),
        def(
            code::SWEPT_SURF,
            "SWEPT_SURF",
            &g("section:p sweep:v scale:f"),
        ),
        def(
            code::SPUN_SURF,
            "SPUN_SURF",
            &g(
                "profile:p base:v axis:v start:v end:v start_param:f end_param:f x_axis:v \
                scale:f",
            ),
        ),
        def(
            code::LIST,
            "LIST",
            "node_id:d owner:p next:p previous:p list_type:d list_length:d block_length:d \
             size_of_entry:d list_block:p finger_block:p finger_index:d notransmit:l",
        ),
        def(
            code::POINTER_LIS_BLOCK,
            "POINTER_LIS_BLOCK",
            "n_entries:d next_block:p entries:p:V",
        ),
        def(code::ATT_DEF_ID, "ATT_DEF_ID", "string:c:V"),
        def(
            code::ATTRIB_DEF,
            "ATTRIB_DEF",
            "next:p identifier:p type_id:d actions:u:8 field_names:p legal_owners:l:14 \
             fields:u:V",
        ),
        def(
            code::ATTRIBUTE,
            "ATTRIBUTE",
            "node_id:d definition:p owner:p next:p previous:p next_of_type:p \
             previous_of_type:p fields:p:V",
        ),
        def(code::INT_VALUES, "INT_VALUES", "values:d:V"),
        def(code::REAL_VALUES, "REAL_VALUES", "values:f:V"),
        def(code::CHAR_VALUES, "CHAR_VALUES", "values:c:V"),
        def(code::POINT_VALUES, "POINT_VALUES", "values:v:V"),
        def(code::VECTOR_VALUES, "VECTOR_VALUES", "values:v:V"),
        def(code::AXIS_VALUES, "AXIS_VALUES", "values:v:V"),
        def(code::TAG_VALUES, "TAG_VALUES", "values:d:V"),
        def(code::DIRECTION_VALUES, "DIRECTION_VALUES", "values:v:V"),
        def(
            code::GROUP,
            "GROUP",
            "node_id:d attributes_groups:p owner:p next:p previous:p type:u first_member:p",
        ),
        def(
            code::MEMBER_OF_GROUP,
            "MEMBER_OF_GROUP",
            "dummy_node_id:d owning_group:p owner:p next:p previous:p next_member:p \
             previous_member:p",
        ),
        def(code::UNICODE_VALUES, "UNICODE_VALUES", "values:w:V"),
        def(code::FIELD_NAMES, "FIELD_NAMES", "names:p:V"),
        def(
            code::TRANSFORM,
            "TRANSFORM",
            "node_id:d owner:p next:p previous:p rotation_matrix:f:9 translation_vector:v \
             scale:f flag:d perspective_vector:v",
        ),
        def(
            code::WORLD,
            "WORLD",
            "assembly:p attribute:p body:p transform:p surface:p curve:p point:p alive:l \
             attrib_def:p highest_id:d current_id:d",
        ),
        def(code::KEY, "KEY", "string:c:V"),
        def(
            code::PE_SURF,
            "PE_SURF",
            &g("type:c data:p tf:p internal_geom:p:V"),
        ),
        def(
            code::PE_CURVE,
            "PE_CURVE",
            &g("type:c data:p tf:p internal_geom:p:V"),
        ),
        def(code::B_SURFACE, "B_SURFACE", &g("nurbs:p data:p")),
        def(
            code::SURFACE_DATA,
            "SURFACE_DATA",
            "original_uint:i original_vint:i extended_uint:i extended_vint:i self_int:u \
             original_u_start:c original_u_end:c original_v_start:c original_v_end:c \
             extended_u_start:c extended_u_end:c extended_v_start:c extended_v_end:c \
             analytic_form_type:c swept_form_type:c spun_form_type:c blend_form_type:c \
             analytic_form:p swept_form:p spun_form:p blend_form:p",
        ),
        def(
            code::NURBS_SURF,
            "NURBS_SURF",
            "u_periodic:l v_periodic:l u_degree:n v_degree:n n_u_vertices:d n_v_vertices:d \
             u_knot_type:u v_knot_type:u n_u_knots:d n_v_knots:d rational:l u_closed:l \
             v_closed:l surface_form:u vertex_dim:n bspline_vertices:p u_knot_mult:p \
             v_knot_mult:p u_knots:p v_knots:p",
        ),
        def(code::KNOT_MULT, "KNOT_MULT", "mult:n:V"),
        def(code::KNOT_SET, "KNOT_SET", "knots:f:V"),
        def(
            code::TRIMMED_CURVE,
            "TRIMMED_CURVE",
            &g("basis_curve:p point_1:v point_2:v parm_1:f parm_2:f"),
        ),
        def(code::B_CURVE, "B_CURVE", &g("nurbs:p data:p")),
        def(code::CURVE_DATA, "CURVE_DATA", "self_int:u analytic_form:p"),
        def(
            code::NURBS_CURVE,
            "NURBS_CURVE",
            "degree:n n_vertices:d vertex_dim:n n_knots:d knot_type:u periodic:l closed:l \
             rational:l curve_form:u bspline_vertices:p knot_mult:p knots:p",
        ),
        def(
            code::SP_CURVE,
            "SP_CURVE",
            &g("surface:p b_curve:p original:p tolerance_to_original:f"),
        ),
        def(
            code::GEOMETRIC_OWNER,
            "GEOMETRIC_OWNER",
            "owner:p next:p previous:p shared_geometry:p",
        ),
    ]
}

/// A field description read from an embedded schema (`I`/`A` instruction
/// or full new-type description).
#[derive(Debug, Clone)]
pub struct EmbeddedField {
    /// Field name.
    pub name: String,
    /// Pointer class (0 for non-pointer fields).
    pub ptr_class: u32,
    /// Element count (0 scalar, 1 variable, n array).
    pub n_elts: u32,
    /// Wire type code string (empty for pointers, which are implied).
    pub ty: String,
    /// Whether the field is transmitted (only meaningful for
    /// variable-length fields; scalar/array inserted fields are always
    /// transmitted).
    pub xmt: bool,
}

impl EmbeddedField {
    /// Resolve to a [`FieldSpec`], or `None` for non-transmitted fields.
    pub fn to_spec(&self) -> Result<Option<FieldSpec>> {
        if !self.xmt {
            return Ok(None);
        }
        let ty = if self.ptr_class != 0 {
            FieldType::Ptr
        } else {
            let b = self.ty.as_bytes();
            if b.len() != 1 {
                return Err(XtError::Parse {
                    offset: 0,
                    what: "embedded field type is not a single code character",
                });
            }
            FieldType::from_code(b[0]).ok_or(XtError::Parse {
                offset: 0,
                what: "unknown embedded field type code",
            })?
        };
        Ok(Some(FieldSpec {
            name: self.name.clone(),
            ty,
            n_elts: self.n_elts,
        }))
    }
}

/// One instruction of an embedded-schema edit script.
#[derive(Debug, Clone)]
pub enum EditOp {
    /// Copy the next base field.
    Copy,
    /// Delete (skip) the next base field.
    Delete,
    /// Insert a field before the next base field.
    Insert(EmbeddedField),
    /// Append a field after all base fields.
    Append(EmbeddedField),
}

/// Apply an embedded-schema edit script to a base node definition,
/// producing the layout used by the file.
pub fn apply_edits(base: &NodeDef, n_effective: usize, ops: &[EditOp]) -> Result<NodeDef> {
    let mut fields = Vec::with_capacity(n_effective);
    let mut next_base = 0usize;
    for op in ops {
        match op {
            EditOp::Copy => {
                let f = base.fields.get(next_base).ok_or(XtError::Parse {
                    offset: 0,
                    what: "embedded edit script copies past the end of the base layout",
                })?;
                fields.push(f.clone());
                next_base += 1;
            }
            EditOp::Delete => {
                if next_base >= base.fields.len() {
                    return Err(XtError::Parse {
                        offset: 0,
                        what: "embedded edit script deletes past the end of the base layout",
                    });
                }
                next_base += 1;
            }
            EditOp::Insert(f) | EditOp::Append(f) => {
                if let Some(spec) = f.to_spec()? {
                    fields.push(spec);
                }
            }
        }
    }
    // Base fields beyond the last instruction are implicitly deleted: the
    // writer's edit loop stops as soon as the current fields run out.
    if fields.len() != n_effective {
        return Err(XtError::Parse {
            offset: 0,
            what: "embedded edit script yields wrong effective field count",
        });
    }
    Ok(NodeDef {
        code: base.code,
        name: base.name.clone(),
        fields,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn base_schema_layouts_have_expected_arity() {
        let table = base_schema();
        let get = |c: u16| table.iter().find(|d| d.code == c).unwrap();
        assert_eq!(get(code::BODY).fields.len(), 23);
        assert_eq!(get(code::FACE).fields.len(), 14);
        assert_eq!(get(code::FIN).fields.len(), 10);
        assert_eq!(get(code::REGION).fields.len(), 7);
        assert_eq!(get(code::CIRCLE).fields.len(), 11);
        assert_eq!(get(code::CONE).fields.len(), 13);
        assert!(get(code::ATTRIB_DEF).is_variable());
        assert!(!get(code::BODY).is_variable());
        assert_eq!(get(code::LIST).fields.len(), 12);
    }

    #[test]
    fn edit_script_reshapes_base_layout() {
        let base = def(70, "LIST", "a:d b:p c:d d:l");
        let ins = |name: &str| {
            EditOp::Insert(EmbeddedField {
                name: name.into(),
                ptr_class: 0,
                n_elts: 0,
                ty: "u".into(),
                xmt: true,
            })
        };
        // C I(x) C D C D A(y): a, x, b, (skip c), (skip d)… wrong count on
        // purpose first.
        let ops = vec![
            EditOp::Copy,
            ins("x"),
            EditOp::Copy,
            EditOp::Delete,
            EditOp::Copy,
            EditOp::Append(EmbeddedField {
                name: "y".into(),
                ptr_class: 12,
                n_elts: 0,
                ty: String::new(),
                xmt: true,
            }),
        ];
        let out = apply_edits(&base, 5, &ops).unwrap();
        let names: Vec<&str> = out.fields.iter().map(|f| f.name.as_str()).collect();
        assert_eq!(names, ["a", "x", "b", "d", "y"]);
        assert_eq!(out.fields[4].ty, FieldType::Ptr);
        assert!(apply_edits(&base, 4, &ops).is_err());
    }
}
