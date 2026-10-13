#!/usr/bin/env python3
"""Static validation of the generated mod folder: XML, vox files, references, materials, size."""
import collections, os, re, struct, sys
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.dirname(__file__))
import build_titan as B

mod = sys.argv[1] if len(sys.argv) > 1 else B.MOD
bad = 0
def ok(c, m):
    global bad
    print(("  ok   " if c else "  FAIL ") + m)
    if not c: bad += 1

root = ET.parse(os.path.join(mod, "titan.xml")).getroot()
ok(root.tag == "prefab", "titan.xml parses, root <prefab>")
bodies = root.findall(".//body")
ok(len(bodies) == 13, "13 bodies (%d)" % len(bodies))
names = [b.get("name") for b in bodies]
ok(len(set(names)) == 13, "unique body names")
scripts = root.findall(".//script")
ok(len(scripts) == 1 and scripts[0].get("file") == "MOD/script/titan.lua", "one script -> MOD/script/titan.lua")
ok(os.path.exists(os.path.join(mod, "script", "titan.lua")), "script file exists")
ok(all(b.get("dynamic") == "true" for b in bodies), "all bodies dynamic")
for v in root.findall(".//vox"):
    f = v.get("file")
    ok(f.startswith("MOD/") and os.path.exists(os.path.join(mod, f[4:])), "vox reference exists: " + f)
    ok(int(v.get("strength")) > 1 and int(v.get("strength")) < 10 ** 6, "finite strength %s (not unbreakable)" % v.get("strength"))

spawn = open(os.path.join(mod, "spawn.txt")).read().splitlines()
entries = [l for l in spawn if l.strip() and not l.startswith("#")]
ok(entries == ["titan.xml : Titans/Muscular Titan"], "spawn.txt entry: %s" % entries)
ok(os.path.exists(os.path.join(mod, entries[0].split(" : ")[0])), "spawn target exists")
info = open(os.path.join(mod, "info.txt")).read()
ok("name = Muscular Titan" in info, "info.txt name")

# vox files
tot = 0
mats = collections.Counter()
pal_ok = True
for n in names:
    path = os.path.join(mod, "vox", n + ".vox")
    d = open(path, "rb").read()
    ok(d[:4] == b"VOX " and struct.unpack("<i", d[4:8])[0] == 150, n + ".vox header")
    off = 8; n_, m_ = struct.unpack("<ii", d[off + 4:off + 12]); off += 12 + n_; end = off + m_
    ok(end == len(d), n + ".vox chunk sizes consistent")
    size = None; vox = []; pal = None
    while off < end:
        cid = d[off:off + 4]; a, b = struct.unpack("<ii", d[off + 4:off + 12]); c = off + 12
        if cid == b"SIZE": size = struct.unpack("<iii", d[c:c + 12])
        if cid == b"XYZI":
            k = struct.unpack("<i", d[c:c + 4])[0]
            vox = [(d[c + 4 + 4 * i], d[c + 5 + 4 * i], d[c + 6 + 4 * i], d[c + 7 + 4 * i]) for i in range(k)]
        if cid == b"RGBA": pal = [tuple(d[c + 4 * i:c + 4 * i + 4]) for i in range(256)]
        off = c + a + b
    ok(max(size) <= 256, "%s size %s <= 256" % (n, size))
    ok(all(v[0] < size[0] and v[1] < size[1] and v[2] < size[2] and v[3] != 0 for v in vox), n + " voxels inside bounds, index 0 unused")
    tot += len(vox)
    for v in vox: mats[v[3]] += 1
slots = set(B.TOUGH_SLOTS)
outside = sum(c for i, c in mats.items() if i not in slots)
ok(outside == 0, "every voxel uses a tough material slot (outside: %d)" % outside)
def kind(i):
    for nm, (a, b) in (("hardmetal", (169, 176)), ("hardmasonry", (177, 184)), ("heavymetal", (137, 152)), ("metal", (121, 136)), ("rock", (41, 56)), ("masonry", (73, 104))):
        if a <= i <= b: return nm
by = collections.Counter()
for i, c in mats.items(): by[kind(i)] += c
print("  voxels per material:", {k: "%.1f%%" % (100.0 * v / tot) for k, v in by.most_common()})
print("  total voxels:", tot)
lua = open(os.path.join(mod, "script", "titan.lua")).read()
ok("--@LAYOUT@--" not in lua, "layout injected")
ok(lua.count("function ") > 20, "script has functions")
print("FAILURES:", bad)
sys.exit(1 if bad else 0)
