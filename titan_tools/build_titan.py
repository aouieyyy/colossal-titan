#!/usr/bin/env python3
"""
Builds the "Muscular Titan" Teardown mod from muscle_titan.vox.

  python3 titan_tools/build_titan.py

What it does
  1. Loads muscle_titan.vox (52 tiled MagicaVoxel models -> one 300x255x810 voxel grid).
  2. Splits it into 13 animatable body parts (pelvis, chest, head, 2x upper arm, forearm,
     thigh, shin, foot) and adds a small overlap at every joint so the seams stay closed
     while the limbs move.
  3. Rescales the model so the titan is TARGET_KONG_MULTIPLE x as tall as King Kong
     (King Kong measured from kong.xml + mech-basic2.vox: KONG_HEIGHT_M).
  4. Re-maps the 255 palette colours onto Teardown's *tough* material index ranges
     (colours are kept, materials are not weak glass/foliage/unphysical any more).
  5. Writes one .vox per part, titan.xml (prefab), script/titan.lua (from the template
     + generated LAYOUT table) and the manifest files.

Pure python 3 standard library.
"""
import collections
import json
import math
import os
import random
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC_VOX = os.path.join(REPO, "muscle_titan.vox")
MOD = os.path.join(REPO, "Muscular Titan")

# ----------------------------------------------------------------------------------
# Tunables (build time). Runtime tunables live in the CFG table at the top of titan.lua.
# ----------------------------------------------------------------------------------
KONG_HEIGHT_M = 19.7            # measured: bounds of kong.xml (+ mech-basic2.vox object sizes)
TARGET_KONG_MULTIPLE = 3.0      # requested: ~3x King Kong
VOXEL_M = 0.1                   # Teardown voxel edge
DENSITY = 5000000               # xml density  (heavy)
STRENGTH = 100                  # xml strength (1 = normal, Kong uses ~1e54 = unbreakable)
OVERLAP = 6                     # original voxels of overlap into the parent part at every seam
PALETTE_K = 96                  # number of tough palette slots used
SEED = 12345

# Teardown palette index -> material (community converter: metarmask/teardown editor-format)
TOUGH_SLOTS = (
    list(range(169, 177)) +     # hard metal   (8)
    list(range(177, 185)) +     # hard masonry (8)
    list(range(137, 153)) +     # heavy metal  (16)
    list(range(121, 137)) +     # metal        (16)
    list(range(41, 57)) +       # rock         (16)
    list(range(73, 105))        # masonry      (32)
)
assert len(TOUGH_SLOTS) == PALETTE_K

# ----------------------------------------------------------------------------------
# Landmarks in the ORIGINAL MagicaVoxel coordinates of muscle_titan.vox
#   (titan faces -y, up is +z, the titan's right side is -x)
# ----------------------------------------------------------------------------------
GROUND_Z = -405
PIVOTS = {   # name: (x, y, z)  right side has negative x
    "pelvis":    (0, 8, -95),
    "chest":     (0, 14, 140),
    "head":      (0, 12, 340),
    "thighR":    (-52, 8, -95),   "thighL":    (52, 8, -95),
    "shinR":     (-54, 16, -255), "shinL":     (54, 16, -255),
    "footR":     (-54, -16, -352), "footL":    (54, -16, -352),
    "upperarmR": (-123, 62, 236), "upperarmL": (123, 62, 236),
    "forearmR":  (-124, 24, 30),  "forearmL":  (124, 24, 30),
}
PARENT = {
    "chest": "pelvis", "head": "chest",
    "thighR": "pelvis", "thighL": "pelvis",
    "shinR": "thighR", "shinL": "thighL",
    "footR": "shinR", "footL": "shinL",
    "upperarmR": "chest", "upperarmL": "chest",
    "forearmR": "upperarmR", "forearmL": "upperarmL",
}
ORDER = ["pelvis", "chest", "head",
         "upperarmR", "upperarmL", "forearmR", "forearmL",
         "thighR", "thighL", "shinR", "shinL", "footR", "footL"]

KNEE_Z, ANKLE_Z, ELBOW_Z = -255, -352, 30
CHEST_Z, NECK_Z = 150, 340


def arm_bx(z):
    if z <= 150:
        return 99
    if z >= 262:
        return 999
    return 99 + (z - 150) / (262 - 150) * (138 - 99)


def leg_cut(x):
    return -106 + min(abs(x), 100) / 100 * 58


N26 = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)]


# ----------------------------------------------------------------------------------
# VOX reading / writing
# ----------------------------------------------------------------------------------
def load_vox(path):
    d = open(path, "rb").read()
    off = 8
    n, m = struct.unpack("<ii", d[off + 4:off + 12])
    off += 12 + n
    end = off + m
    sizes, models, trn, shp, pal = [], [], {}, {}, None

    def rstr(o):
        l = struct.unpack("<i", d[o:o + 4])[0]
        return d[o + 4:o + 4 + l].decode("latin1"), o + 4 + l

    def rdict(o):
        k = struct.unpack("<i", d[o:o + 4])[0]
        o += 4
        dd = {}
        for _ in range(k):
            a, o = rstr(o)
            v, o = rstr(o)
            dd[a] = v
        return dd, o

    while off < end:
        cid = d[off:off + 4]
        n, m = struct.unpack("<ii", d[off + 4:off + 12])
        c = off + 12
        if cid == b"SIZE":
            sizes.append(struct.unpack("<iii", d[c:c + 12]))
        elif cid == b"XYZI":
            k = struct.unpack("<i", d[c:c + 4])[0]
            models.append(d[c + 4:c + 4 + 4 * k])
        elif cid == b"RGBA":
            pal = [tuple(d[c + 4 * i:c + 4 * i + 4]) for i in range(256)]
        elif cid == b"nTRN":
            at, o = rdict(c + 4)
            child = struct.unpack("<i", d[o:o + 4])[0]
            o += 16
            fr, o = rdict(o)
            trn[child] = fr
        elif cid == b"nSHP":
            nid = struct.unpack("<i", d[c:c + 4])[0]
            at, o = rdict(c + 4)
            o += 4
            shp[nid] = struct.unpack("<i", d[o:o + 4])[0]
        off = c + n + m
    vox = {}
    for nid, mid in shp.items():
        t = [int(x) for x in trn[nid].get("_t", "0 0 0").split()]
        sx, sy, sz = sizes[mid]
        ox, oy, oz = t[0] - sx // 2, t[1] - sy // 2, t[2] - sz // 2
        b = models[mid]
        for i in range(0, len(b), 4):
            vox[(b[i] + ox, b[i + 1] + oy, b[i + 2] + oz)] = b[i + 3]
    return vox, pal


def vox_chunk(cid, content, children=b""):
    return cid + struct.pack("<ii", len(content), len(children)) + content + children


def vox_dict(d):
    out = struct.pack("<i", len(d))
    for k, v in d.items():
        kb, vb = k.encode(), v.encode()
        out += struct.pack("<i", len(kb)) + kb + struct.pack("<i", len(vb)) + vb
    return out


def write_vox(path, name, cells, palette):
    """cells: dict (x,y,z)->palette index (1..255), coordinates >= 0."""
    sx = max(k[0] for k in cells) + 1
    sy = max(k[1] for k in cells) + 1
    sz = max(k[2] for k in cells) + 1
    assert max(sx, sy, sz) <= 256, (name, sx, sy, sz)
    xyzi = struct.pack("<i", len(cells)) + b"".join(struct.pack("<BBBB", x, y, z, i) for (x, y, z), i in cells.items())
    rgba = b"".join(bytes(palette[i][:3]) + b"\xff" for i in range(256))
    # scene graph: root transform -> group -> named transform (no translation) -> shape
    ntrn0 = vox_chunk(b"nTRN", struct.pack("<i", 0) + vox_dict({}) + struct.pack("<iiii", 1, -1, -1, 1) + vox_dict({}))
    ngrp = vox_chunk(b"nGRP", struct.pack("<i", 1) + vox_dict({}) + struct.pack("<ii", 1, 2))
    ntrn2 = vox_chunk(b"nTRN", struct.pack("<i", 2) + vox_dict({"_name": name}) + struct.pack("<iiii", 3, -1, 0, 1) + vox_dict({}))
    nshp = vox_chunk(b"nSHP", struct.pack("<i", 3) + vox_dict({}) + struct.pack("<ii", 1, 0) + vox_dict({}))
    children = (vox_chunk(b"SIZE", struct.pack("<iii", sx, sy, sz)) + vox_chunk(b"XYZI", xyzi) +
                ntrn0 + ngrp + ntrn2 + nshp + vox_chunk(b"RGBA", rgba))
    with open(path, "wb") as f:
        f.write(b"VOX " + struct.pack("<i", 150) + vox_chunk(b"MAIN", b"", children))
    return sx, sy, sz


# ----------------------------------------------------------------------------------
# Segmentation
# ----------------------------------------------------------------------------------
def segment(vox):
    s = set(vox)
    part = {}
    for side, sg in (("R", -1), ("L", 1)):
        seeds = [k for k in s if 160 <= k[2] <= 240 and k[0] * sg >= arm_bx(k[2])]
        stack = list(seeds)
        seen = set(seeds)
        while stack:
            x, y, z = stack.pop()
            for dx, dy, dz in N26:
                n = (x + dx, y + dy, z + dz)
                if n in s and n not in seen:
                    nx, ny, nz = n
                    if nz > 262 or nx * sg < 60:
                        continue
                    if nz >= 150 and nx * sg < arm_bx(nz):
                        continue
                    if nz < 150 and nx * sg < 95:
                        continue
                    if nz < 20 and nx * sg < 106:
                        continue
                    if nz < -140:
                        continue
                    seen.add(n)
                    stack.append(n)
        for k in seen:
            part[k] = ("upperarm" if k[2] >= ELBOW_Z else "forearm") + side
    for k in s:
        if k in part:
            continue
        x, y, z = k
        if z < leg_cut(x):
            side = "R" if x < 0 else "L"
            part[k] = ("foot" if z < ANKLE_Z else "shin" if z < KNEE_Z else "thigh") + side
        else:
            part[k] = "head" if z >= NECK_Z else "chest" if z >= CHEST_Z else "pelvis"
    return part


def add_overlaps(part):
    """Every child part also owns the OVERLAP voxels of its parent next to the seam."""
    members = collections.defaultdict(set)
    for k, p in part.items():
        members[p].add(k)
    by_parent = {p: members[p] for p in members}
    result = {p: set(v) for p, v in members.items()}
    for child, parent in PARENT.items():
        pset = by_parent[parent]
        frontier = set()
        for k in members[child]:
            for dx, dy, dz in N26:
                n = (k[0] + dx, k[1] + dy, k[2] + dz)
                if n in pset:
                    frontier.add(n)
        grown = set(frontier)
        for _ in range(OVERLAP - 1):
            nxt = set()
            for k in frontier:
                for dx, dy, dz in N26:
                    n = (k[0] + dx, k[1] + dy, k[2] + dz)
                    if n in pset and n not in grown:
                        nxt.add(n)
            grown |= nxt
            frontier = nxt
        result[child] |= grown
    return result


# ----------------------------------------------------------------------------------
# Palette: keep the colours, move them onto tough material slots
# ----------------------------------------------------------------------------------
def quantize_palette(vox, pal):
    usage = collections.Counter(vox.values())
    cols = {i: pal[i - 1][:3] for i in usage}
    ids = sorted(cols)
    rnd = random.Random(SEED)

    def d2(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2

    # weighted k-means++ init
    centers = [cols[rnd.choices(ids, weights=[usage[i] for i in ids])[0]]]
    while len(centers) < PALETTE_K:
        dist = [min(d2(cols[i], c) for c in centers) * usage[i] for i in ids]
        if sum(dist) == 0:
            break
        centers.append(cols[rnd.choices(ids, weights=dist)[0]])
    centers = [tuple(float(v) for v in c) for c in centers]
    for _ in range(40):
        acc = [[0.0, 0.0, 0.0, 0] for _ in centers]
        assign = {}
        for i in ids:
            best = min(range(len(centers)), key=lambda k: d2(cols[i], centers[k]))
            assign[i] = best
            w = usage[i]
            for c in range(3):
                acc[best][c] += cols[i][c] * w
            acc[best][3] += w
        moved = 0
        for k, a in enumerate(acc):
            if a[3]:
                nc = (a[0] / a[3], a[1] / a[3], a[2] / a[3])
                moved += d2(nc, centers[k])
                centers[k] = nc
        if moved < 1e-3:
            break
    weight = collections.Counter()
    for i in ids:
        weight[assign[i]] += usage[i]
    ranked = [k for k, _ in weight.most_common()]
    ranked += [k for k in range(len(centers)) if k not in weight]
    slot_of = {k: TOUGH_SLOTS[r] for r, k in enumerate(ranked)}   # most used colours -> hardest materials
    new_pal = [(90, 90, 90, 255)] * 256
    for k, slot in slot_of.items():
        c = centers[k]
        new_pal[slot - 1] = (int(round(c[0])), int(round(c[1])), int(round(c[2])), 255)
    remap = {i: slot_of[assign[i]] for i in ids}
    err = sum(usage[i] * math.sqrt(d2(cols[i], new_pal[remap[i] - 1][:3])) for i in ids) / sum(usage.values())
    return remap, new_pal, err


# ----------------------------------------------------------------------------------
# Main build
# ----------------------------------------------------------------------------------
def main():
    random.seed(SEED)
    print("loading", SRC_VOX)
    vox, pal = load_vox(SRC_VOX)
    xs = [k[0] for k in vox]
    ys = [k[1] for k in vox]
    zs = [k[2] for k in vox]
    X0, Y0, Z0 = min(xs), min(ys), min(zs)
    assert Z0 == GROUND_Z, Z0
    height_vox = max(zs) - Z0 + 1
    target_h = KONG_HEIGHT_M * TARGET_KONG_MULTIPLE
    F = target_h / (height_vox * VOXEL_M)
    F = round(F, 3)
    print("source voxels", len(vox), "height", height_vox, "vox ->", height_vox * VOXEL_M, "m at scale 1")
    print("target height %.1f m (= %.1f x King Kong %.1f m) -> resample factor %.3f" % (target_h, TARGET_KONG_MULTIPLE, KONG_HEIGHT_M, F))

    remap, new_pal, err = quantize_palette(vox, pal)
    print("palette: %d colours -> %d tough material slots, mean colour error %.2f / 255" % (len(set(vox.values())), PALETTE_K, err))

    part = segment(vox)
    members = add_overlaps(part)
    print({p: len(v) for p, v in sorted(members.items())})

    def cell(k):
        return (int((k[0] - X0) * F), int((k[1] - Y0) * F), int((k[2] - Z0) * F))

    # global per-cell colour (majority) so overlapping parts share exactly the same colours
    votes = collections.defaultdict(collections.Counter)
    for k, c in vox.items():
        votes[cell(k)][remap[c]] += 1
    cell_color = {c: v.most_common(1)[0][0] for c, v in votes.items()}
    out_parts = {}
    for p in ORDER:
        cs = set()
        for k in members[p]:
            cs.add(cell(k))
        out_parts[p] = cs
        print("  %-10s %7d source voxels -> %6d cells" % (p, len(members[p]), len(cs)))

    # pivots / sizes in output-grid metres, then converted to body-local "D" space:
    #   D = (-(gx - px), gz, (gy - py)) : x = titan's right, y = up, front = -z (Teardown forward)
    pel = PIVOTS["pelvis"]
    pgx, pgy = (pel[0] - X0) * F * VOXEL_M, (pel[1] - Y0) * F * VOXEL_M

    def toD(p):   # p in original voxel coords -> D metres, y above ground
        gx, gy, gz = (p[0] - X0) * F * VOXEL_M, (p[1] - Y0) * F * VOXEL_M, (p[2] - Z0) * F * VOXEL_M
        return (-(gx - pgx), gz, gy - pgy)

    palette = new_pal
    os.makedirs(os.path.join(MOD, "vox"), exist_ok=True)
    os.makedirs(os.path.join(MOD, "script"), exist_ok=True)
    layout = {}
    for p in ORDER:
        cs = out_parts[p]
        mn = [min(c[i] for c in cs) for i in range(3)]
        mx = [max(c[i] for c in cs) for i in range(3)]
        shifted = {(c[0] - mn[0], c[1] - mn[1], c[2] - mn[2]): cell_color[c] for c in cs}
        dims = write_vox(os.path.join(MOD, "vox", p + ".vox"), p, shifted, palette)
        # bounding box in output metres (magica axes)
        bx0, by0, bz0 = mn[0] * VOXEL_M, mn[1] * VOXEL_M, mn[2] * VOXEL_M
        bx1, by1, bz1 = (mx[0] + 1) * VOXEL_M, (mx[1] + 1) * VOXEL_M, (mx[2] + 1) * VOXEL_M
        cx, cy = (bx0 + bx1) / 2, (by0 + by1) / 2
        # bottom-centre of the shape in D coordinates (absolute, ground = 0)
        bottom = (-(cx - pgx), bz0, cy - pgy)
        center = (-(cx - pgx), (bz0 + bz1) / 2, cy - pgy)
        piv = toD(PIVOTS[p])
        layout[p] = {
            "name": p, "parent": PARENT.get(p),
            "pivot": [round(v, 4) for v in piv],
            "bottom": [round(v, 4) for v in bottom],
            "center": [round(v, 4) for v in center],
            "size": [mx[0] - mn[0] + 1, mx[2] - mn[2] + 1, mx[1] - mn[1] + 1],   # engine order: x, up, depth
            "voxels": len(cs),
        }
        print("  wrote %-10s %s voxels %d" % (p, dims, len(cs)))

    def sub(a, b):
        return [round(a[i] - b[i], 4) for i in range(3)]

    for p, L in layout.items():
        L["rel_parent"] = sub(L["pivot"], layout[L["parent"]]["pivot"]) if L["parent"] else [0, 0, 0]
        L["shape_pos"] = sub(L["bottom"], L["pivot"])      # vox pos relative to the body origin
        L["center_off"] = sub(L["center"], L["pivot"])     # bbox centre relative to the body origin

    def length(v):
        return math.sqrt(sum(c * c for c in v))

    summary = {
        "kong_height_m": KONG_HEIGHT_M, "multiple": TARGET_KONG_MULTIPLE, "resample": F,
        "height_m": round(max(zs) - Z0 + 1, 1) * F * VOXEL_M,
        "pelvis_height": layout["pelvis"]["pivot"][1],
        "thigh_len": round(length(layout["shinR"]["rel_parent"]), 3),
        "shin_len": round(length(layout["footR"]["rel_parent"]), 3),
        "ankle_height": layout["footR"]["pivot"][1],
        "voxels": sum(L["voxels"] for L in layout.values()),
        "strength": STRENGTH, "density": DENSITY,
    }
    print("summary", json.dumps(summary))

    write_xml(layout)
    write_script(layout, summary)
    write_manifest(summary)
    print("done ->", MOD)


def f(v):
    return "%.4f" % v


def write_xml(layout):
    lines = ['<prefab version="1.7.0">', '\t<group name="Muscular Titan" pos="0.0 0.0 0.0" rot="0.0 0.0 0.0">',
             '\t\t<script pos="0.0 0.0 0.0" file="MOD/script/titan.lua" param0="speed=9" param1="turnspeed=0.2">']
    lines.append('\t\t\t<group name="titan">')
    for p in ORDER:
        L = layout[p]
        tags = "titanpart"
        if p == "pelvis":
            tags += " body"
        if p.startswith("foot"):
            tags += " foot"
        lines.append('\t\t\t\t<body name="%s" tags="%s" pos="%s %s %s" rot="0.0 0.0 0.0" dynamic="true">' %
                     (p, tags, f(L["pivot"][0]), f(L["pivot"][1]), f(L["pivot"][2])))
        sp = L["shape_pos"]
        lines.append('\t\t\t\t\t<vox tags="nocull" pos="%s %s %s" rot="0.0 180.0 0.0" density="%d" strength="%d" file="MOD/vox/%s.vox"/>' %
                     (f(sp[0]), f(sp[1]), f(sp[2]), DENSITY, STRENGTH, p))
        lines.append('\t\t\t\t</body>')
    lines += ['\t\t\t</group>', '\t\t</script>', '\t</group>', '</prefab>', '']
    with open(os.path.join(MOD, "titan.xml"), "w") as fh:
        fh.write("\n".join(lines))


def lua_val(v, indent=0):
    pad = "\t" * indent
    if isinstance(v, dict):
        items = []
        for k, x in v.items():
            items.append("%s\t%s = %s," % (pad, k, lua_val(x, indent + 1)))
        return "{\n" + "\n".join(items) + "\n" + pad + "}"
    if isinstance(v, (list, tuple)):
        return "{" + ", ".join(lua_val(x) for x in v) + "}"
    if isinstance(v, str):
        return '"%s"' % v
    if v is None:
        return "nil"
    if isinstance(v, bool):
        return "true" if v else "false"
    return repr(v)


def write_script(layout, summary):
    tpl = open(os.path.join(HERE, "titan.lua.tpl")).read()
    parts = {}
    for p in ORDER:
        L = dict(layout[p])
        if L["parent"] is None:
            L["parent"] = None
        parts[p] = {k: v for k, v in L.items() if v is not None}
    block = "LAYOUT = {\n"
    block += "\tparts = " + lua_val(parts, 1) + ",\n"
    block += "\torder = {" + ", ".join('"%s"' % p for p in ORDER) + "},\n"
    block += "\tkongHeight = %s, multiple = %s, resample = %s,\n" % (summary["kong_height_m"], summary["multiple"], summary["resample"])
    block += "\tpelvisHeight = %s, thighLen = %s, shinLen = %s, ankleHeight = %s,\n" % (
        summary["pelvis_height"], summary["thigh_len"], summary["shin_len"], summary["ankle_height"])
    block += "}\n"
    assert "--@LAYOUT@--" in tpl
    with open(os.path.join(MOD, "script", "titan.lua"), "w") as fh:
        fh.write(tpl.replace("--@LAYOUT@--", block))


def write_manifest(summary):
    with open(os.path.join(MOD, "info.txt"), "w") as fh:
        fh.write("name = Muscular Titan\nauthor = Kiro\n"
                 "description = A %.0f m tall muscular titan (about 3x King Kong) that walks after you. "
                 "HOW TO USE: enable the mod, start a sandbox level, press Esc and open Spawn, then pick Titans > Muscular Titan. "
                 "Settings are at the top of script/titan.lua.\ntags =\n" % summary["height_m"])
    with open(os.path.join(MOD, "spawn.txt"), "w") as fh:
        fh.write("#List spawnable instances here in this format (path is relative to the mod folder):\n"
                 "#path/to/prefab.xml : Category/Name\n"
                 "titan.xml : Titans/Muscular Titan\n")


if __name__ == "__main__":
    main()
