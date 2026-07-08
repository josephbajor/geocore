#!/usr/bin/env python3
"""Generate hand-authored XT fixtures: a solid block, text and neutral
binary, at exactly base schema 13006 (no embedded schemas), per the
published XT Format Reference. Layout mirrors ktopo::make::block."""
import struct

DX, DY, DZ = 0.2, 0.3, 0.4
HX, HY, HZ = DX / 2, DY / 2, DZ / 2

corners = []
for i in range(8):
    x = HX if i & 1 else -HX
    y = HY if i & 2 else -HY
    z = HZ if i & 4 else -HZ
    corners.append((x, y, z))

FACES = [
    ([0, 2, 3, 1], (0.0, 0.0, -1.0)),
    ([4, 5, 7, 6], (0.0, 0.0, 1.0)),
    ([0, 1, 5, 4], (0.0, -1.0, 0.0)),
    ([2, 6, 7, 3], (0.0, 1.0, 0.0)),
    ([0, 4, 6, 2], (-1.0, 0.0, 0.0)),
    ([1, 3, 7, 5], (1.0, 0.0, 0.0)),
]

# Node indices
BODY, RVOID, RSOLID, SHSOLID, SHVOID = 1, 2, 3, 4, 5
FACE0, LOOP0, PLANE0, EDGE0, FIN0, VERT0, PT0, LINE0 = 10, 20, 30, 40, 60, 90, 100, 110

edge_key_to_idx = {}   # (lo,hi) -> edge node idx
edge_first_fin = {}
edge_fins = {}         # edge idx -> [fin idx]
fin_records = {}       # fin idx -> dict
vertex_fins = {i: [] for i in range(8)}

for f, (ring, nrm) in enumerate(FACES):
    loop_idx = LOOP0 + f
    fins_of_loop = []
    for k in range(4):
        a, b = ring[k], ring[(k + 1) % 4]
        key = (min(a, b), max(a, b))
        if key not in edge_key_to_idx:
            edge_key_to_idx[key] = EDGE0 + len(edge_key_to_idx)
        e = edge_key_to_idx[key]
        fin_idx = FIN0 + 4 * f + k
        sense = '+' if a < b else '-'
        fin_records[fin_idx] = dict(loop=loop_idx, vertex=VERT0 + b, edge=e, sense=sense)
        edge_fins.setdefault(e, []).append(fin_idx)
        vertex_fins[b].append(fin_idx)
        fins_of_loop.append(fin_idx)
    for k, fin_idx in enumerate(fins_of_loop):
        fin_records[fin_idx]['forward'] = fins_of_loop[(k + 1) % 4]
        fin_records[fin_idx]['backward'] = fins_of_loop[(k - 1) % 4]

for e, fins in edge_fins.items():
    assert len(fins) == 2
    fin_records[fins[0]]['other'] = fins[1]
    fin_records[fins[1]]['other'] = fins[0]

# next_at_vx chains per vertex; head is vertex.fin
vertex_fin_head = {}
for v, fins in vertex_fins.items():
    vertex_fin_head[v] = fins[0]
    for i, fin in enumerate(fins):
        fin_records[fin]['next_at_vx'] = fins[i + 1] if i + 1 < len(fins) else 0

edges = sorted(edge_key_to_idx.items(), key=lambda kv: kv[1])  # [(key, idx)]

NULL = '?'
nodes = []  # (type, index, [field values])  value: int | float | str-char | '?'

def vec(t):
    return list(t)

def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

def norm(v):
    n = (v[0]**2 + v[1]**2 + v[2]**2) ** 0.5
    return (v[0] / n, v[1] / n, v[2] / n)

max_id = LINE0 + len(edges) - 1

# BODY (type 12): 23 fields
nodes.append((12, BODY, [
    max_id, 0, 0, 0, 0, 0, 0, 1e3, 1e-8, 0, 0, 0, 1, 0,
    1,  # body_type solid
    1,  # nom_geom_state
    SHSOLID,
    PLANE0,        # boundary_surface chain head
    LINE0,         # boundary_curve chain head
    PT0,           # boundary_point chain head
    RVOID,         # region chain head (infinite first)
    EDGE0,         # edge chain head
    VERT0,         # vertex chain head
]))
# REGIONs (19): node_id ag body next previous shell type
nodes.append((19, RVOID, [RVOID, 0, BODY, RSOLID, 0, SHVOID, 'V']))
nodes.append((19, RSOLID, [RSOLID, 0, BODY, 0, RVOID, SHSOLID, 'S']))
# SHELLs (13): node_id ag body next face edge vertex region front_face
nodes.append((13, SHSOLID, [SHSOLID, 0, BODY, 0, FACE0, 0, 0, RSOLID, 0]))
nodes.append((13, SHVOID, [SHVOID, 0, 0, 0, 0, 0, 0, RVOID, FACE0]))

for f, (ring, nrm) in enumerate(FACES):
    face_idx, loop_idx, plane_idx = FACE0 + f, LOOP0 + f, PLANE0 + f
    nxt = FACE0 + f + 1 if f < 5 else 0
    prv = FACE0 + f - 1 if f > 0 else 0
    # FACE (14): node_id ag tolerance next previous loop shell surface sense
    #            next_on_surface previous_on_surface next_front previous_front front_shell
    nodes.append((14, face_idx, [face_idx, 0, NULL, nxt, prv, loop_idx, SHSOLID,
                                 plane_idx, '+', 0, 0, nxt, prv, SHVOID]))
    # LOOP (15): node_id ag fin face next
    nodes.append((15, loop_idx, [loop_idx, 0, FIN0 + 4 * f, face_idx, 0]))
    # PLANE (50): common7 + pvec normal x_axis
    center = [sum(corners[c][i] for c in ring) / 4 for i in range(3)]
    xdir = norm(sub(corners[ring[1]], corners[ring[0]]))
    nxt_s = PLANE0 + f + 1 if f < 5 else 0
    prv_s = PLANE0 + f - 1 if f > 0 else 0
    nodes.append((50, plane_idx, [plane_idx, 0, face_idx, nxt_s, prv_s, 0, '+',
                                  *center, *nrm, *xdir]))

for fin_idx, r in sorted(fin_records.items()):
    # FIN (17): ag loop forward backward vertex other edge curve next_at_vx sense
    nodes.append((17, fin_idx, [0, r['loop'], r['forward'], r['backward'], r['vertex'],
                                r['other'], r['edge'], 0, r['next_at_vx'], r['sense']]))

for n, ((lo, hi), e) in enumerate(edges):
    line_idx = LINE0 + n
    nxt = EDGE0 + n + 1 if n < len(edges) - 1 else 0
    prv = EDGE0 + n - 1 if n > 0 else 0
    # EDGE (16): node_id ag tolerance fin previous next curve
    #            next_on_curve previous_on_curve owner
    nodes.append((16, e, [e, 0, NULL, edge_fins[e][0], prv, nxt, line_idx, 0, 0, BODY]))
    # LINE (30): common7 + pvec direction
    nxt_c = LINE0 + n + 1 if n < len(edges) - 1 else 0
    prv_c = LINE0 + n - 1 if n > 0 else 0
    d = norm(sub(corners[hi], corners[lo]))
    nodes.append((30, line_idx, [line_idx, 0, e, nxt_c, prv_c, 0, '+',
                                 *corners[lo], *d]))

for v in range(8):
    vert_idx, pt_idx = VERT0 + v, PT0 + v
    nxt = VERT0 + v + 1 if v < 7 else 0
    prv = VERT0 + v - 1 if v > 0 else 0
    # VERTEX (18): node_id ag fin previous next point tolerance owner
    nodes.append((18, vert_idx, [vert_idx, 0, vertex_fin_head[v], prv, nxt, pt_idx,
                                 NULL, BODY]))
    nxt_p = PT0 + v + 1 if v < 7 else 0
    prv_p = PT0 + v - 1 if v > 0 else 0
    # POINT (29): node_id ag owner next previous pvec
    nodes.append((29, pt_idx, [pt_idx, 0, BODY, nxt_p, prv_p, *corners[v]]))

# ---- text emission ----
def fmt_num(v):
    if isinstance(v, int):
        return str(v)
    s = repr(float(v))
    return s

VERSION = ": TRANSMIT FILE created by modeller version 1300000"
SCHEMA = "SCH_1300000_13006"

def text_body():
    toks = []
    toks.append("T")            # flag, no space
    toks.append(f"{len(VERSION)} {VERSION}")
    toks.append(f"{len(SCHEMA)} {SCHEMA} ")
    toks.append("0 ")           # usfld
    for (ty, idx, fields) in nodes:
        toks.append(f"{ty} {idx} ")
        for v in fields:
            if v == NULL:
                toks.append("?")
            elif isinstance(v, str):
                toks.append(v)  # char, no space
            else:
                toks.append(fmt_num(v) + " ")
    toks.append("1 0 ")
    return "".join(toks)

HEADER = (
    "**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz**************************\n"
    "**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789**************************\n"
    "**PART1;MC=none;APPL=cad_prototype-kxt;SITE=none;USER=none;FORMAT=__FMT__;GUISE=transmit;\n"
    "**PART2;SCH=SCH_1300000_13006;USFLD_SIZE=0;\n"
    "**PART3;\n"
    "**END_OF_HEADER*****************************************************************\n"
)

body = text_body()
wrapped = "\n".join(body[i:i + 79] for i in range(0, len(body), 79))
with open("block.x_t", "w") as f:
    f.write(HEADER.replace("__FMT__", "text"))
    f.write(wrapped)
    f.write("\n")

# ---- binary emission ----
FIELD_TYPES = {
    12: "d p p p p p p f f p p p u p u u p p p p p p p",
    19: "d p p p p p c",
    13: "d p p p p p p p p",
    14: "d p f p p p p p c p p p p p",
    15: "d p p p p",
    17: "p p p p p p p p p c",
    16: "d p f p p p p p p p",
    18: "d p p p p p f p",
    29: "d p p p p v",
    50: "d p p p p p c v v v",
    30: "d p p p p p c v v",
}

def enc_index(i):
    assert 0 <= i
    if i < 32767:
        return struct.pack(">h", i + 1)
    return struct.pack(">hh", -(i % 32767 + 1), i // 32767)

NULLD = struct.pack(">d", -3.14158e13)

def bin_body():
    out = bytearray()
    out += b"PS\0\0"
    out += struct.pack(">h", len(VERSION)) + VERSION.encode()
    out += struct.pack(">i", len(SCHEMA)) + SCHEMA.encode()
    out += struct.pack(">i", 0)  # usfld
    for (ty, idx, fields) in nodes:
        out += struct.pack(">h", ty)
        out += enc_index(idx)
        kinds = FIELD_TYPES[ty].split()
        fi = 0
        for kind in kinds:
            if kind == "v":
                for _ in range(3):
                    out += struct.pack(">d", float(fields[fi])); fi += 1
            elif kind == "f":
                v = fields[fi]; fi += 1
                out += NULLD if v == NULL else struct.pack(">d", float(v))
            elif kind == "d":
                out += struct.pack(">i", int(fields[fi])); fi += 1
            elif kind == "p":
                out += enc_index(int(fields[fi])); fi += 1
            elif kind == "u":
                out += struct.pack(">B", int(fields[fi])); fi += 1
            elif kind == "c":
                out += fields[fi].encode(); fi += 1
        assert fi == len(fields), (ty, fi, len(fields))
    out += struct.pack(">h", 1)   # terminator type
    out += enc_index(0)           # terminator index
    return bytes(out)

with open("block.x_b", "wb") as f:
    f.write(HEADER.replace("__FMT__", "binary").encode())
    f.write(bin_body())

print("wrote block.x_t", "and block.x_b;", len(nodes), "nodes; highest id", max_id)
