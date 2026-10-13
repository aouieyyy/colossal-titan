#!/usr/bin/env python3
"""Assemble the generated mod (vox files + titan.xml, as Teardown would) and render PNG previews."""
import os, struct, sys, zlib
import xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__))
MOD = os.path.join(os.path.dirname(HERE), "Muscular Titan")

def read_part(path):
    d = open(path, "rb").read()
    off = 8; n, m = struct.unpack("<ii", d[off + 4:off + 12]); off += 12 + n; end = off + m
    size, vox, pal = None, [], None
    while off < end:
        cid = d[off:off + 4]; a, b = struct.unpack("<ii", d[off + 4:off + 12]); c = off + 12
        if cid == b"SIZE": size = struct.unpack("<iii", d[c:c + 12])
        if cid == b"XYZI":
            k = struct.unpack("<i", d[c:c + 4])[0]
            vox = [(d[c + 4 + 4 * i], d[c + 5 + 4 * i], d[c + 6 + 4 * i], d[c + 7 + 4 * i]) for i in range(k)]
        if cid == b"RGBA": pal = [tuple(d[c + 4 * i:c + 4 * i + 3]) for i in range(256)]
        off = c + a + b
    return size, vox, pal

def png(path, w, h, rows):
    raw = bytearray()
    for r in rows: raw.append(0); raw += r
    def ch(t, dd): c = struct.pack(">I", len(dd)) + t + dd; return c + struct.pack(">I", zlib.crc32(t + dd) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + ch(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) + ch(b"IDAT", zlib.compress(bytes(raw), 6)) + ch(b"IEND", b""))

def assemble(pose=None):
    """returns list of (x,y,z,(r,g,b)) in titan-local metres (front = -z, up = +y, x = titan's right)."""
    root = ET.parse(os.path.join(MOD, "titan.xml")).getroot()
    pts = []
    for body in root.iter("body"):
        bp = [float(v) for v in body.get("pos").split()]
        vx = body.find("vox")
        sp = [float(v) for v in vx.get("pos").split()]
        size, vox, pal = read_part(os.path.join(MOD, vx.get("file")[4:]))
        sx, sy, sz = size
        base = (bp[0] + sp[0], bp[1] + sp[1], bp[2] + sp[2])
        for x, y, z, i in vox:
            # engine-local (x, up, depth) with vox rot "0 180 0" about the bottom-centre
            D = (-(x + 0.5 - sx / 2) * 0.1, (z + 0.5) * 0.1, (y + 0.5 - sy / 2) * 0.1)
            pts.append((base[0] + D[0], base[1] + D[1], base[2] + D[2], pal[i - 1], body.get("name")))
    return pts

def render(pts, au, av, depth, sign, path, scale=3):
    us = [p[au] for p in pts]; vs = [p[av] for p in pts]
    u0, u1, v0, v1 = min(us), max(us), min(vs), max(vs)
    W, H = int((u1 - u0) * scale * 10) + 1, int((v1 - v0) * scale * 10) + 1
    buf = {}
    for p in pts:
        key = (int((p[au] - u0) * scale * 10), int((p[av] - v0) * scale * 10))
        d = p[depth] * sign
        if key not in buf or d > buf[key][0]: buf[key] = (d, p[3])
    dmin = min(b[0] for b in buf.values()); dmax = max(b[0] for b in buf.values())
    rows = []
    for v in range(H - 1, -1, -1):
        r = bytearray()
        for u in range(W):
            b = buf.get((u, v))
            if b is None: r += bytes((36, 36, 44))
            else:
                sh = 0.6 + 0.4 * (b[0] - dmin) / max(1e-6, dmax - dmin)
                r += bytes(min(255, int(c * sh)) for c in b[1])
        rows.append(r)
    png(path, W, H, rows)
    return (u0, u1, v0, v1, W, H)

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    pts = assemble()
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]; zs = [p[2] for p in pts]
    print("assembled voxels", len(pts), " x %.1f..%.1f  y %.1f..%.1f  z %.1f..%.1f" % (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
    print("height %.2f m = %.2f x King Kong" % (max(ys) - min(ys), (max(ys) - min(ys)) / 19.7))
    # camera in FRONT of the titan (front = -z): the nearest voxels have the smallest z
    print("front view", render(pts, 0, 1, 2, -1, os.path.join(out, "preview_front.png"), scale=0.5))
    print("side view ", render(pts, 2, 1, 0, 1, os.path.join(out, "preview_side.png"), scale=0.5))
