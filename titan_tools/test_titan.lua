-- =====================================================================
-- Offline simulation of "Muscular Titan" against a MOCK of the Teardown API.
-- It checks the script's own logic (Lua 5.1 compatibility, API usage, IK, gait, AI,
-- hover/turn control, health) - it does NOT prove how the real engine behaves.
--
-- usage:  luarun test_titan.lua "<path to Muscular Titan>" [scenario]
-- scenarios: walk (default), slope, obstacle, assemble_offset, damage
-- =====================================================================
local MOD = arg[1]
local SCEN = arg[2] or "walk"
math.atan2 = math.atan2 or function(y, x) return math.atan(y, x) end   -- Lua 5.1 (Teardown) has atan2
local fails, checks = 0, 0
local function check(c, msg)
	checks = checks + 1
	if not c then fails = fails + 1 print("  FAIL: " .. msg) end
end
local function readFile(p) local f = io.open(p, "rb") if not f then return nil end local s = f:read("*a") f:close() return s end

-- ---------------------------------------------------------------- Lua 5.1 scan
local src = readFile(MOD .. "/script/titan.lua")
check(src ~= nil, "titan.lua exists")
do
	local code = src:gsub("%-%-%[%[.-%]%]", ""):gsub("%-%-[^\n]*", ""):gsub('"[^"\n]*"', '""'):gsub("'[^'\n]*'", "''")
	for _, tok in ipairs({"goto", "//", "::", "<<", ">>", "table.unpack", "table.pack", "table.move", "math.tointeger", "utf8%.", "bit32"}) do
		check(not code:find(tok, 1, tok:find("%%") == nil), "uses Lua 5.2+ feature: " .. tok)
	end
	check(not code:gsub("~=", ""):find("~", 1, true), "uses bitwise ~")
end

-- ---------------------------------------------------------------- math API
local function isNum(x) return type(x) == "number" and x == x and x ~= math.huge and x ~= -math.huge end
local function chkvec(v, w) if type(v) ~= "table" or not isNum(v[1]) or not isNum(v[2]) or not isNum(v[3]) then error("bad Vec for " .. w .. ": " .. tostring(v and v[1]) .. "," .. tostring(v and v[2]) .. "," .. tostring(v and v[3]), 3) end end
local function chkquat(q, w) if type(q) ~= "table" or not isNum(q[1]) or not isNum(q[2]) or not isNum(q[3]) or not isNum(q[4]) then error("bad Quat for " .. w, 3) end end
local function chknum(n, w) if not isNum(n) then error("bad number for " .. w .. ": " .. tostring(n), 3) end end
function Vec(x, y, z) return {x or 0, y or 0, z or 0} end
function VecCopy(v) chkvec(v, "VecCopy") return {v[1], v[2], v[3]} end
function VecAdd(a, b) chkvec(a, "VecAdd.a") chkvec(b, "VecAdd.b") return {a[1] + b[1], a[2] + b[2], a[3] + b[3]} end
function VecSub(a, b) chkvec(a, "VecSub.a") chkvec(b, "VecSub.b") return {a[1] - b[1], a[2] - b[2], a[3] - b[3]} end
function VecScale(a, s) chkvec(a, "VecScale") chknum(s, "VecScale.s") return {a[1] * s, a[2] * s, a[3] * s} end
function VecDot(a, b) chkvec(a, "VecDot.a") chkvec(b, "VecDot.b") return a[1] * b[1] + a[2] * b[2] + a[3] * b[3] end
function VecCross(a, b) chkvec(a, "VecCross.a") chkvec(b, "VecCross.b") return {a[2] * b[3] - a[3] * b[2], a[3] * b[1] - a[1] * b[3], a[1] * b[2] - a[2] * b[1]} end
function VecLength(a) chkvec(a, "VecLength") return math.sqrt(a[1] * a[1] + a[2] * a[2] + a[3] * a[3]) end
function VecNormalize(a) local l = VecLength(a) if l == 0 then return {0, 0, 0} end return VecScale(a, 1 / l) end
function VecLerp(a, b, t) chknum(t, "VecLerp.t") return VecAdd(a, VecScale(VecSub(b, a), t)) end
function Quat(x, y, z, w) return {x or 0, y or 0, z or 0, w or 1} end
local function qmul(a, b) return {
	a[4] * b[1] + a[1] * b[4] + a[2] * b[3] - a[3] * b[2],
	a[4] * b[2] - a[1] * b[3] + a[2] * b[4] + a[3] * b[1],
	a[4] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[4],
	a[4] * b[4] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3]} end
local function qnorm(q) local l = math.sqrt(q[1] ^ 2 + q[2] ^ 2 + q[3] ^ 2 + q[4] ^ 2) return {q[1] / l, q[2] / l, q[3] / l, q[4] / l} end
local function qaa(axis, deg) local h = math.rad(deg) * 0.5 local n = VecNormalize(axis) local s = math.sin(h) return {n[1] * s, n[2] * s, n[3] * s, math.cos(h)} end
function QuatEuler(x, y, z) chknum(x, "QE") chknum(y, "QE") chknum(z, "QE") return qmul(qmul(qaa({0, 1, 0}, y), qaa({0, 0, 1}, z)), qaa({1, 0, 0}, x)) end
function QuatRotateQuat(a, b) chkquat(a, "QRQ.a") chkquat(b, "QRQ.b") return qnorm(qmul(a, b)) end
function QuatRotateVec(q, v) chkquat(q, "QRV.q") chkvec(v, "QRV.v") local r = qmul(qmul(q, {v[1], v[2], v[3], 0}), {-q[1], -q[2], -q[3], q[4]}) return {r[1], r[2], r[3]} end
function QuatSlerp(a, b, t)
	chkquat(a, "slerp.a") chkquat(b, "slerp.b") chknum(t, "slerp.t")
	local d = a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4]
	local bb = b
	if d < 0 then d = -d bb = {-b[1], -b[2], -b[3], -b[4]} end
	if d > 0.9995 then
		return qnorm({a[1] + (bb[1] - a[1]) * t, a[2] + (bb[2] - a[2]) * t, a[3] + (bb[3] - a[3]) * t, a[4] + (bb[4] - a[4]) * t})
	end
	local th = math.acos(d)
	local s0, s1 = math.sin((1 - t) * th) / math.sin(th), math.sin(t * th) / math.sin(th)
	return {a[1] * s0 + bb[1] * s1, a[2] * s0 + bb[2] * s1, a[3] * s0 + bb[3] * s1, a[4] * s0 + bb[4] * s1}
end
local function basisQuat(x, y, z)
	local m00, m01, m02, m10, m11, m12, m20, m21, m22 = x[1], y[1], z[1], x[2], y[2], z[2], x[3], y[3], z[3]
	local w = math.sqrt(math.max(0, 1 + m00 + m11 + m22)) / 2
	local qx, qy, qz = math.sqrt(math.max(0, 1 + m00 - m11 - m22)) / 2, math.sqrt(math.max(0, 1 - m00 + m11 - m22)) / 2, math.sqrt(math.max(0, 1 - m00 - m11 + m22)) / 2
	if m21 - m12 < 0 then qx = -qx end
	if m02 - m20 < 0 then qy = -qy end
	if m10 - m01 < 0 then qz = -qz end
	return {qx, qy, qz, w}
end
function QuatLookAt(eye, target)
	chkvec(eye, "QLA.eye") chkvec(target, "QLA.target")
	local z = VecNormalize(VecSub(eye, target))
	if VecLength(z) == 0 then return Quat() end
	local up = {0, 1, 0}
	if math.abs(z[2]) > 0.999 then up = {1, 0, 0} end
	local x = VecNormalize(VecCross(up, z))
	return basisQuat(x, VecCross(z, x), z)
end
function Transform(p, r) return {pos = p and VecCopy(p) or Vec(), rot = r or Quat()} end
local function qinv(q) return {-q[1], -q[2], -q[3], q[4]} end
function TransformToParentPoint(t, p) return VecAdd(t.pos, QuatRotateVec(t.rot, p)) end
function TransformToParentVec(t, v) return QuatRotateVec(t.rot, v) end
function TransformToLocalVec(t, v) return QuatRotateVec(qinv(t.rot), v) end

-- ---------------------------------------------------------------- scene / engine model
local T = {time = 0, nextH = 1, ents = {}, constraints = {}, counters = {}, played = 0, holes = 0, tags = {}, health = 1}
local function new(e) local h = T.nextH T.nextH = h + 1 T.ents[h] = e e.h = h return h end
local function cnt(n) T.counters[n] = (T.counters[n] or 0) + 1 end
T.groundFn = function(x, z) return 0 end
T.boxes = {}
T.player = {pos = Vec(0, 0, -150)}
T.shapeBias = Vec(0, 0, 0)      -- simulates a different vox pivot convention
T.groundBody = new({type = "body", dynamic = false, static = true})
T.groundShape = new({type = "shape", body = T.groundBody})

-- the generated LAYOUT is needed to build the mock bodies: load it by executing titan.lua up to init()
local chunk, err = (loadstring or load)(src, "titan.lua")
check(chunk ~= nil, "titan.lua compiles: " .. tostring(err))
if not chunk then print(string.format("RESULT: %d checks, %d failures", checks, fails)) os.exit(1) end

-- parse LAYOUT by running only the data part
local layoutSrc = src:match("(LAYOUT = {.-\n}\n)")
local LAY = {}
do
	local f = (loadstring or load)(layoutSrc .. "\nreturn LAYOUT")
	local ok, res = pcall(f)
	check(ok and res ~= nil, "LAYOUT parses")
	LAY = res
end

local bodyByName = {}
local spawnRot = (SCEN == "spawn_rotated") and QuatEuler(0, 137, 0) or Quat()
local spawnPos = (SCEN == "spawn_rotated") and Vec(55, 0, 20) or Vec(0, 0, 0)
for _, name in ipairs(LAY.order) do
	local L = LAY.parts[name]
	local mass = L.voxels * 40
	local b = new({type = "body", name = name, dynamic = true, t = {pos = VecAdd(spawnPos, QuatRotateVec(spawnRot, Vec(L.pivot[1], L.pivot[2], L.pivot[3]))), rot = spawnRot},
		vel = Vec(), ang = Vec(), mass = mass, shapes = {}})
	local s = new({type = "shape", body = b, L = L, voxels = L.voxels, local_off = Vec(L.center_off[1], L.center_off[2], L.center_off[3]),
		size = L.size, tags = {}})
	T.ents[b].shapes[1] = s
	bodyByName[name] = b
end

local function bodyOf(h) return T.ents[h] end
local function shapeBounds(h)
	local s = T.ents[h]
	local b = T.ents[s.body]
	local c = VecAdd(b.t.pos, QuatRotateVec(b.t.rot, VecAdd(s.local_off, T.shapeBias)))
	-- axis aligned extents of the oriented box (yaw only rotations are used by the titan)
	local hx, hy, hz = s.size[1] * 0.05, s.size[2] * 0.05, s.size[3] * 0.05
	local ex = math.abs(QuatRotateVec(b.t.rot, {hx, 0, 0})[1]) + math.abs(QuatRotateVec(b.t.rot, {0, hy, 0})[1]) + math.abs(QuatRotateVec(b.t.rot, {0, 0, hz})[1])
	local ey = math.abs(QuatRotateVec(b.t.rot, {hx, 0, 0})[2]) + math.abs(QuatRotateVec(b.t.rot, {0, hy, 0})[2]) + math.abs(QuatRotateVec(b.t.rot, {0, 0, hz})[2])
	local ez = math.abs(QuatRotateVec(b.t.rot, {hx, 0, 0})[3]) + math.abs(QuatRotateVec(b.t.rot, {0, hy, 0})[3]) + math.abs(QuatRotateVec(b.t.rot, {0, 0, hz})[3])
	return {c[1] - ex, c[2] - ey, c[3] - ez}, {c[1] + ex, c[2] + ey, c[3] + ez}
end

-- bodies / shapes
function FindBody(name) return bodyByName[name] or 0 end
function GetBodyShapes(h) return {table.unpack and table.unpack(T.ents[h].shapes) or unpack(T.ents[h].shapes)} end
function IsHandleValid(h) return T.ents[h] ~= nil and not T.ents[h].deleted end
function GetShapeVoxelCount(h) return T.ents[h].voxels end
function GetShapeBody(h) return T.ents[h].body end
function GetShapeBounds(h) return shapeBounds(h) end
function GetShapeSize(h) local s = T.ents[h] return s.size[1], s.size[2], s.size[3], 0.1 end
function GetShapeLocalTransform(h) local s = T.ents[h] return Transform(VecAdd(s.local_off, T.shapeBias), Quat()) end
function SetShapeLocalTransform(h, t) local s = T.ents[h] s.local_off = VecSub(t.pos, T.shapeBias) cnt("SetShapeLocalTransform") end
function SetShapeCollisionFilter(h, l, m) chknum(l, "layer") chknum(m, "mask") cnt("SetShapeCollisionFilter") end
function SetTag(h, tag) T.ents[h].tags = T.ents[h].tags or {} T.ents[h].tags[tag] = true cnt("SetTag") end
function RemoveTag(h, tag) if T.ents[h].tags then T.ents[h].tags[tag] = nil end end
function GetBodyTransform(h) local b = T.ents[h] return {pos = VecCopy(b.t.pos), rot = {b.t.rot[1], b.t.rot[2], b.t.rot[3], b.t.rot[4]}} end
function SetBodyTransform(h, t) chkvec(t.pos, "SetBodyTransform.pos") chkquat(t.rot, "SetBodyTransform.rot") T.ents[h].t = {pos = VecCopy(t.pos), rot = qnorm(t.rot)} cnt("SetBodyTransform") end
function GetBodyVelocity(h) return VecCopy(T.ents[h].vel or Vec()) end
function SetBodyVelocity(h, v) chkvec(v, "SetBodyVelocity") T.ents[h].vel = VecCopy(v) end
function GetBodyAngularVelocity(h) return VecCopy(T.ents[h].ang or Vec()) end
function SetBodyAngularVelocity(h, v) chkvec(v, "SetBodyAngularVelocity") T.ents[h].ang = VecCopy(v) end
function GetBodyMass(h) return T.ents[h].mass or 0 end
function GetBodyCenterOfMass(h) return Vec(0, 3, 0) end
function GetBodyVelocityAtPos(h, p) return VecCopy(T.ents[h].vel or Vec()) end
function SetBodyDynamic(h, d) T.ents[h].dynamic = d end
function IsBodyDynamic(h) return T.ents[h].dynamic == true end
function GetFloatParam(n, d) return d end

-- queries
local rejected = {}
function QueryRequire(s) end
function QueryRejectBody(h) rejected[h] = true end
local function slab(o, d, mi, ma)
	local tmin, tmax = 0, 1e9
	for a = 1, 3 do
		if math.abs(d[a]) < 1e-9 then
			if o[a] < mi[a] or o[a] > ma[a] then return nil end
		else
			local t1, t2 = (mi[a] - o[a]) / d[a], (ma[a] - o[a]) / d[a]
			if t1 > t2 then t1, t2 = t2, t1 end
			if t1 > tmin then tmin = t1 end
			if t2 < tmax then tmax = t2 end
			if tmin > tmax then return nil end
		end
	end
	return tmin
end
function QueryRaycast(o, d, maxDist, radius)
	chkvec(o, "ray.o") chkvec(d, "ray.d") chknum(maxDist, "ray.max")
	rejected = {}
	cnt("QueryRaycast")
	local best, bestShape, bestN = nil, nil, Vec(0, 1, 0)
	-- ground
	local prev = o[2] - T.groundFn(o[1], o[3])
	local t = 0
	if prev <= 0 then best = 0 else
		local step = 0.25
		while t < maxDist do
			local nt = math.min(maxDist, t + step)
			local p = VecAdd(o, VecScale(d, nt))
			local hgt = p[2] - T.groundFn(p[1], p[3])
			if hgt <= 0 then
				local lo, hi = t, nt
				for _ = 1, 20 do
					local mid = (lo + hi) / 2
					local pm = VecAdd(o, VecScale(d, mid))
					if pm[2] - T.groundFn(pm[1], pm[3]) <= 0 then hi = mid else lo = mid end
				end
				best = hi
				break
			end
			t = nt
			if nt >= maxDist then break end
		end
	end
	if best then bestShape = T.groundShape end
	for i, bx in ipairs(T.boxes) do
		local tt = slab(o, d, bx.mi, bx.ma)
		if tt and tt <= maxDist and (not best or tt < best) then
			best, bestShape = tt, bx.shape
			bestN = Vec(-d[1], 0, -d[3])
		end
	end
	if best then return true, best, bestN, bestShape end
	return false, maxDist, Vec(0, 1, 0), 0
end
function MakeHole(p, a, b, c, silent) chkvec(p, "MakeHole") chknum(a, "hole.a") chknum(b, "hole.b") chknum(c, "hole.c") cnt("MakeHole")
	-- remove obstacle boxes touching the hole (simulates smashing)
	for i = #T.boxes, 1, -1 do
		local bx = T.boxes[i]
		if p[1] > bx.mi[1] - a and p[1] < bx.ma[1] + a and p[2] > bx.mi[2] - a and p[2] < bx.ma[2] + a and p[3] > bx.mi[3] - a and p[3] < bx.ma[3] + a then
			bx.hp = (bx.hp or 6) - 1
			if bx.hp <= 0 then table.remove(T.boxes, i) cnt("ObstacleDestroyed") end
		end
	end
end

-- constraints (resolved after update)
function ConstrainPosition(a, b, pa, pb, maxVel, maxForce) chkvec(pa, "CP.pa") chkvec(pb, "CP.pb") chknum(maxVel, "CP.v") chknum(maxForce, "CP.f") T.constraints[#T.constraints + 1] = {"pos", a, b, pa, pb, maxVel} end
function ConstrainOrientation(a, b, qa, qb, maxAng, maxF) chkquat(qa, "CO.qa") chkquat(qb, "CO.qb") chknum(maxAng, "CO.w") chknum(maxF, "CO.f") T.constraints[#T.constraints + 1] = {"ori", a, b, qa, qb, maxAng} end
function ConstrainVelocity(a, b, p, dir, rel, mn, mx) chkvec(p, "CV.p") chkvec(dir, "CV.dir") chknum(rel, "CV.rel") chknum(mn, "CV.min") chknum(mx, "CV.max") T.constraints[#T.constraints + 1] = {"vel", a, b, p, dir, rel, mn, mx} end
function ConstrainAngularVelocity(a, b, dir, rel, mn, mx) chkvec(dir, "CAV.dir") chknum(rel, "CAV.rel") chknum(mn, "CAV.min") chknum(mx, "CAV.max") T.constraints[#T.constraints + 1] = {"ang", a, b, dir, rel, mn, mx} end

-- misc
function LoadSound(p, d) cnt("LoadSound") return new({type = "sound"}) end
function PlaySound(s, p, v) chkvec(p, "PlaySound.pos") chknum(v, "PlaySound.vol") cnt("PlaySound") end
function ShakeCamera(s) chknum(s, "Shake") cnt("ShakeCamera") end
for _, n in ipairs({"ParticleReset", "ParticleType", "ParticleRadius", "ParticleColor", "ParticleAlpha", "ParticleGravity", "ParticleDrag"}) do _G[n] = function() end end
function SpawnParticle(p, v, l) chkvec(p, "SpawnParticle.pos") chkvec(v, "SpawnParticle.vel") cnt("SpawnParticle") end
function GetPlayerTransform() return Transform(T.player.pos, Quat()) end
function GetPlayerHealth() return T.health end
function SetPlayerHealth(h) T.health = h cnt("SetPlayerHealth") end
function DebugPrint(s) T.debug = (T.debug or "") .. tostring(s) .. "\n" end
function DebugWatch() end
function DebugLine() end

-- ---------------------------------------------------------------- run the script
local ok, e = pcall(chunk)
check(ok, "titan.lua top level runs: " .. tostring(e))

if SCEN == "slope" then T.groundFn = function(x, z) return 0.12 * x + 1.5 * math.sin(z * 0.05) end end
if SCEN == "assemble_offset" then T.shapeBias = Vec(0.4, 3.0, -1.2) end
if SCEN == "obstacle" then
	local shp = new({type = "shape", body = new({type = "body", dynamic = false})})
	T.boxes[1] = {mi = Vec(-30, 0, -112), ma = Vec(30, 14, -100), shape = shp, hp = 5}
end

-- spawn the prefab: pelvis at designed pivot, spawn point O = (0,0,0) on ground
init()
check(T.debug == nil or not T.debug:find("not found"), "all bodies found" .. (T.debug and ("\n" .. T.debug) or ""))

-- ---------------------------------------------------------------- simulation helpers
local DT = 1 / 60
local G = 10
local function resolve(dt)
	-- gravity on the pelvis only (every other part is held by constraints)
	local pel = T.ents[bodyByName.pelvis]
	pel.vel = VecAdd(pel.vel, Vec(0, -G * dt, 0))
	for _, c in ipairs(T.constraints) do
		local kind = c[1]
		if kind == "vel" then
			local b = T.ents[c[2]]
			local dir = VecNormalize(c[5])
			local cur = VecDot(b.vel, dir)
			local want = c[6]
			local dv = want - cur
			local lim = c[8] / b.mass
			local limn = c[7] / b.mass
			if dv > lim then dv = lim end
			if dv < limn then dv = limn end
			b.vel = VecAdd(b.vel, VecScale(dir, dv))
		elseif kind == "ang" then
			local b = T.ents[c[2]]
			local dir = VecNormalize(c[4])
			local cur = VecDot(b.ang, dir)
			local dv = c[5] - cur
			local lim = c[7] / b.mass
			local limn = c[6] / b.mass
			if dv > lim then dv = lim end
			if dv < limn then dv = limn end
			b.ang = VecAdd(b.ang, VecScale(dir, dv))
		end
	end
	-- integrate the pelvis
	local before = VecCopy(pel.t.pos)
	pel.t.pos = VecAdd(pel.t.pos, VecScale(pel.vel, dt))
	for _, bx in ipairs(T.boxes) do
		local p2 = pel.t.pos
		if p2[1] > bx.mi[1] - 3 and p2[1] < bx.ma[1] + 3 and p2[3] > bx.mi[3] - 3 and p2[3] < bx.ma[3] + 3 and p2[2] - T.groundFn(p2[1], p2[3]) < bx.ma[2] + 14 then
			pel.t.pos = Vec(before[1], pel.t.pos[2], before[3])
			pel.vel = Vec(0, pel.vel[2], 0)
		end
	end
	local moved = VecSub(pel.t.pos, before)
	local w = pel.ang
	local wl = VecLength(w)
	if wl > 1e-9 then
		local dq = {0, 0, 0, 1}
		local axis = VecScale(w, 1 / wl)
		local h = wl * dt * 0.5
		dq = {axis[1] * math.sin(h), axis[2] * math.sin(h), axis[3] * math.sin(h), math.cos(h)}
		pel.t.rot = qnorm(qmul(dq, pel.t.rot))
	end
	-- driven parts
	for _, c in ipairs(T.constraints) do
		if c[1] == "pos" then
			local b = T.ents[c[2]]
			-- the target point is attached to the pelvis (second body), exactly like the engine's constraint
			local d = VecSub(VecAdd(c[5], moved), c[4])
			local l = VecLength(d)
			local maxStep = c[6] * dt
			if l > maxStep then d = VecScale(d, maxStep / l) end
			b.t.pos = VecAdd(b.t.pos, d)
			if l > (T.maxPosErr or 0) then T.maxPosErr = l T.maxPosErrAt = T.time T.maxPosErrBody = T.ents[c[2]].name end
		elseif c[1] == "ori" then
			local b = T.ents[c[2]]
			local q0, q1 = b.t.rot, c[5]
			local d = math.abs(q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3] + q0[4] * q1[4])
			local ang = 2 * math.acos(math.min(1, d))
			local maxStep = c[6] * dt
			local tt = ang > maxStep and (maxStep / ang) or 1
			b.t.rot = QuatSlerp(q0, q1, tt)
			T.maxAngErr = math.max(T.maxAngErr or 0, ang)
		end
	end
	T.constraints = {}
end

local function stepSim(dt)
	T.time = T.time + dt
	update(dt)
	resolve(dt)
	tick(dt)
end

local function pel() return T.ents[bodyByName.pelvis] end
local function yawOf(q) local f = QuatRotateVec(q, {0, 0, -1}) return math.atan2(-f[1], -f[3]) end

-- ---------------------------------------------------------------- assembly checks
print("== assembly")
local minFootY, soleOk = 1e9, true
for _, n in ipairs({"footR", "footL"}) do
	local mi = shapeBounds(T.ents[bodyByName[n]].shapes[1])
	minFootY = math.min(minFootY, mi[2])
end
local g0 = T.groundFn(0, 0)
print(string.format("   sole height above ground after spawn: %.2f m (expected ~%.2f)", minFootY - g0, 0.15))
check(math.abs((minFootY - g0) - 0.15) < 0.6 or SCEN == "slope", "feet start on the ground")
local pe = pel()
local expectedFwd = QuatRotateVec(pe.t.rot, {0, 0, -1})
local toPlayer = VecNormalize(Vec(T.player.pos[1] - pe.t.pos[1], 0, T.player.pos[3] - pe.t.pos[3]))
check(VecDot(expectedFwd, toPlayer) > 0.99, "spawns facing the player")
-- model bounds: height of everything
local topY, botY = -1e9, 1e9
for _, n in ipairs(LAY.order) do
	local mi, ma = shapeBounds(T.ents[bodyByName[n]].shapes[1])
	topY = math.max(topY, ma[2]) botY = math.min(botY, mi[2])
end
print(string.format("   titan height after assembly: %.2f m = %.2f x King Kong (%.1f m)", topY - botY, (topY - botY) / LAY.kongHeight, LAY.kongHeight))
check((topY - botY) / LAY.kongHeight > 2.8 and (topY - botY) / LAY.kongHeight < 3.2, "about 3x King Kong")
if SCEN == "assemble_offset" then
	check((T.counters.SetShapeLocalTransform or 0) >= #LAY.order - 1, "shape placement corrected for a different pivot convention (" .. tostring(T.counters.SetShapeLocalTransform) .. ")")
end
-- every body part sits where the layout says (relative to the pelvis)
local pr = pel()
for _, n in ipairs(LAY.order) do
	local b = T.ents[bodyByName[n]]
	local want = VecAdd(pr.t.pos, QuatRotateVec(pr.t.rot, {LAY.parts[n].pivot[1] - LAY.parts.pelvis.pivot[1], LAY.parts[n].pivot[2] - LAY.parts.pelvis.pivot[2], LAY.parts[n].pivot[3] - LAY.parts.pelvis.pivot[3]}))
	check(VecLength(VecSub(b.t.pos, want)) < 0.01, "rest placement of " .. n)
end

-- ---------------------------------------------------------------- walking simulation
print("== simulation: " .. SCEN)
local rec = {t = {}, pelvis = {}, yaw = {}, speed = {}}
local footLog = {R = {}, L = {}}
local events = {}
local minReachGap, clampCount, frames = 1e9, 0, 0
lagMax = 0
swingMin, swingMax, kneeMax, liftMax = 1e9, -1e9, 0, 0
stompKnee, stompLift = 0, 0
local maxBone = 0
local armSign, armChecks, armMax = 0, 0, 0
local thighPrev = {}
local phaseTimeline = {}
local playerPath = function(t)
	if SCEN == "obstacle" then return Vec(0, 0, -240) end
	if t < 20 then return Vec(0, 0, -150) end
	-- the player then runs sideways
	local u = t - 20
	return Vec(math.min(u * 6, 120), 0, -150)
end
local lastSpeedSample = {}
local nanFound = false
local simTime = (SCEN == "obstacle") and 70 or 95
local nSteps = {R = 0, L = 0}
local lastStepKey, alternate, repeats = nil, 0, 0
local plantedMoves = 0
local prevPos = {}
local sinceStart = 0
for i = 1, math.floor(simTime / DT) do
	T.player.pos = playerPath(T.time)
	-- remember planted foot targets before the step
	stepSim(DT)
	frames = frames + 1
	local p = pel()
	local pos = p.t.pos
	if pos[1] ~= pos[1] or pos[2] ~= pos[2] or pos[3] ~= pos[3] then nanFound = true break end
	if i % 6 == 0 then
		rec.t[#rec.t + 1] = T.time
		rec.pelvis[#rec.pelvis + 1] = VecCopy(pos)
		rec.yaw[#rec.yaw + 1] = yawOf(p.t.rot)
		rec.speed[#rec.speed + 1] = VecLength(Vec(p.vel[1], 0, p.vel[3]))
	end
	-- foot logs from the script's own state
	for _, k in ipairs({"R", "L"}) do
		local f = feet[k]
		footLog[k][#footLog[k] + 1] = {planted = not f.stepping, pos = VecCopy(f.pos), walking = VecLength(Vec(p.vel[1], 0, p.vel[3])) > 3.0}
		if not f.stepping then
			if prevPos[k] and VecLength(VecSub(prevPos[k], f.pos)) > 1e-6 then plantedMoves = plantedMoves + 1 end
			prevPos[k] = VecCopy(f.pos)
		else
			prevPos[k] = nil
		end
	end
	-- bone length preservation + IK reach
	for _, k in ipairs({"R", "L"}) do
		local th, sh, ft = T.ents[bodyByName["thigh" .. k]], T.ents[bodyByName["shin" .. k]], T.ents[bodyByName["foot" .. k]]
		local f = feet[k]
		local ankleWant = VecAdd(f.pos, Vec(0, LAY.ankleHeight + 0.15, 0))
		local gap = VecLength(VecSub(ft.t.pos, ankleWant))
		if gap > 0.5 then clampCount = clampCount + 1 end
		minReachGap = math.min(minReachGap, gap)
		local pt, ps, pf = poseOut["thigh" .. k][1], poseOut["shin" .. k][1], poseOut["foot" .. k][1]
		local l1 = VecLength(VecSub(ps, pt))
		local l2 = VecLength(VecSub(pf, ps))
		maxBone = math.max(maxBone, math.abs(l1 - gait.L1), math.abs(l2 - gait.L2))
		if T.time > 4 then
			lagMax = math.max(lagMax, VecLength(VecSub(th.t.pos, pt)), VecLength(VecSub(sh.t.pos, ps)), VecLength(VecSub(ft.t.pos, pf)))
		end
	end
	for _, k in ipairs({"R", "L"}) do
		local th, sh, ft = poseOut["thigh" .. k], poseOut["shin" .. k], poseOut["foot" .. k]
		local fwd = QuatRotateVec(p.t.rot, {0, 0, -1})
		local td = VecNormalize(VecSub(sh[1], th[1]))
		local sd = VecNormalize(VecSub(ft[1], sh[1]))
		local sw = math.deg(math.atan2(VecDot(td, fwd), -td[2]))
		local kn = math.deg(math.acos(math.max(-1, math.min(1, VecDot(td, sd)))))
		if VecLength(Vec(p.vel[1], 0, p.vel[3])) > 3.0 then
			swingMin, swingMax = math.min(swingMin, sw), math.max(swingMax, sw)
			kneeMax = math.max(kneeMax, kn)
			liftMax = math.max(liftMax, feet[k].lift)
		else
			stompKnee = math.max(stompKnee, kn)
			stompLift = math.max(stompLift, feet[k].lift)
		end
	end
	-- arm swing sign: opposite to the same side thigh
	do
		local pr2 = p
		local fwd = QuatRotateVec(pr2.t.rot, {0, 0, -1})
		for _, k in ipairs({"R"}) do
			local th, sh = T.ents[bodyByName["thigh" .. k]], T.ents[bodyByName["shin" .. k]]
			local ua, fa = T.ents[bodyByName["upperarm" .. k]], T.ents[bodyByName["forearm" .. k]]
			local thDir = VecNormalize(VecSub(sh.t.pos, th.t.pos))
			local uaDir = VecNormalize(VecSub(fa.t.pos, ua.t.pos))
			local thetaLeg = math.atan2(VecDot(thDir, fwd), -thDir[2])
			local restUA = VecNormalize(Vec(LAY.parts.forearmR.rel_parent[1], LAY.parts.forearmR.rel_parent[2], LAY.parts.forearmR.rel_parent[3]))
			local restUAw = QuatRotateVec(p.t.rot, restUA)
			local thetaArm = math.atan2(VecDot(uaDir, fwd) - VecDot(restUAw, fwd), -uaDir[2])
			if math.abs(thetaLeg) > 0.15 and math.abs(thetaArm) > 0.01 then
				armChecks = armChecks + 1
				if thetaLeg * thetaArm < 0 then armSign = armSign + 1 end
			end
			armMax = math.max(armMax, math.abs(thetaArm))
		end
	end
end
check(not nanFound, "no NaN in the simulation")
check(T.debug == nil or not T.debug:find("unexpected shape size"), "shape sizes match engine expectations" .. (T.debug and ("\n" .. T.debug) or ""))

-- events from footfalls are consumed in tick; count them via counters
print(string.format("   frames %d, PlaySound %d, ShakeCamera %d, MakeHole %d, constraint max pos error %.2f m (%s at t=%.1f s), max angle error %.2f rad",
	frames, T.counters.PlaySound or 0, T.counters.ShakeCamera or 0, T.counters.MakeHole or 0, T.maxPosErr or 0, tostring(T.maxPosErrBody), T.maxPosErrAt or 0, T.maxAngErr or 0))

-- speed / turning
local vmax, tmax = 0, 0
for i = 1, #rec.speed do vmax = math.max(vmax, rec.speed[i]) end
local maxTurn = 0
for i = 2, #rec.yaw do
	local d = rec.yaw[i] - rec.yaw[i - 1]
	while d > math.pi do d = d - 2 * math.pi end
	while d < -math.pi do d = d + 2 * math.pi end
	maxTurn = math.max(maxTurn, math.abs(d) / (rec.t[i] - rec.t[i - 1]))
end
print(string.format("   top speed %.2f m/s (CFG.SPEED %.1f), max turn rate %.3f rad/s (CFG.TURN_SPEED %.2f)", vmax, CFG.SPEED, maxTurn, CFG.TURN_SPEED))
check(vmax > CFG.SPEED * 0.8, "reaches walking speed")
check(maxTurn < CFG.TURN_SPEED * 1.6 + 0.05, "turns smoothly (no instant rotation)")

-- stepping statistics from the foot logs
local function stepsOf(log)
	local list, cur = {}, nil
	for i, e in ipairs(log) do
		if not e.planted and not cur then cur = i end
		if e.planted and cur then list[#list + 1] = {cur, i} cur = nil end
	end
	return list
end
local sR, sL = stepsOf(footLog.R), stepsOf(footLog.L)
print(string.format("   steps: right %d, left %d", #sR, #sL))
check(#sR >= 8 and #sL >= 8, "both legs take many steps")
-- alternation: merge and check sequence
local seq = {}
for _, s in ipairs(sR) do if footLog.R[s[1]].walking then seq[#seq + 1] = {s[1], "R"} end end
for _, s in ipairs(sL) do if footLog.L[s[1]].walking then seq[#seq + 1] = {s[1], "L"} end end
table.sort(seq, function(a, b) return a[1] < b[1] end)
local same = 0
for i = 2, #seq do if seq[i][2] == seq[i - 1][2] then same = same + 1 end end
print(string.format("   while walking (>3 m/s): %d steps, consecutive steps with the same leg: %d", #seq, same))
check(#seq >= 8 and same <= 1, "legs alternate while walking")
-- overlap: never both feet in the air
local bothAir = 0
for i = 1, #footLog.R do if not footLog.R[i].planted and not footLog.L[i].planted then bothAir = bothAir + 1 end end
check(bothAir == 0, "never both feet airborne")
check(plantedMoves == 0, "planted feet never slide (" .. plantedMoves .. " moves)")
print(string.format("   max bone length error %.5f m, IK clamped frames %d / %d, closest ankle error %.4f", maxBone, clampCount, frames * 2, minReachGap))
check(maxBone < 0.01, "IK keeps thigh and shin length exactly")
print(string.format("   WALKING: thigh swing %.0f .. %.0f deg, max knee bend %.0f deg, max foot lift %.1f m", swingMin, swingMax, kneeMax, liftMax))
print(string.format("   STOMP:   max knee bend %.0f deg, foot lift %.1f m", stompKnee, stompLift))
check(swingMax - swingMin > 25, "legs swing forward and backward visibly")
check(kneeMax > 15 and kneeMax < 85, "knees bend naturally")
check(liftMax > CFG.STEP_HEIGHT * 0.8, "feet are lifted off the ground during steps")
print(string.format("   steady-state limb lag behind its target (after 4 s): %.2f m of a 59 m titan", lagMax))
check(lagMax < 2.0, "limbs follow their targets closely once walking")
check(clampCount < frames * 2 * ((SCEN == "obstacle") and 0.06 or 0.03), "legs almost never over-stretch")
print(string.format("   arm swing: max %.3f rad (limit %.2f); opposite to the same-side leg in %d of %d samples", armMax, CFG.ARM_SWING_MAX, armSign, armChecks))
check(armMax <= CFG.ARM_SWING_MAX + 0.12, "arm swing stays restrained")
check(armChecks == 0 or armSign / armChecks > 0.85, "arms swing opposite to the legs")

-- progress towards the player
local last = rec.pelvis[#rec.pelvis]
local d = VecLength(Vec(last[1] - T.player.pos[1], 0, last[3] - T.player.pos[3]))
print(string.format("   final distance to the player: %.1f m (stop distance %.1f)", d, CFG.STOP_DISTANCE))
if SCEN ~= "obstacle" then
	check(d < CFG.STOP_DISTANCE + 25 or T.time < 30, "follows the player")
end
-- pelvis height stability
local hmin, hmax = 1e9, -1e9
for i = 20, #rec.pelvis do
	local gy = T.groundFn(rec.pelvis[i][1], rec.pelvis[i][3])
	local hh = rec.pelvis[i][2] - gy
	hmin, hmax = math.min(hmin, hh), math.max(hmax, hh)
end
print(string.format("   hip height above ground: %.2f .. %.2f m (target %.2f)", hmin, hmax, hover.distTarget))
check(hmax - hmin < 3.0 or SCEN == "slope", "hips stay at a stable height")
check(hmin > hover.distTarget - 3.0, "titan does not sink into the ground")
if SCEN == "obstacle" then
	print(string.format("   obstacle destroyed: %s, MakeHole calls %d, final z %.1f", tostring(T.counters.ObstacleDestroyed), T.counters.MakeHole or 0, last[3]))
	check((T.counters.ObstacleDestroyed or 0) >= 1, "walls in the way are smashed")
	check(last[3] < -105, "titan passes the wall")
end
check((T.counters.PlaySound or 0) > 5, "footstep sounds play")

-- ---------------------------------------------------------------- damage model
if SCEN == "damage" then
	print("== damage")
	local function dieState() return R.dead end
	check(not dieState(), "alive before damage")
	local armR = T.ents[bodyByName.upperarmR].shapes[1]
	T.ents[armR].voxels = math.floor(T.ents[armR].voxels * 0.3)
	for i = 1, 90 do stepSim(DT) end
	check(P.upperarmR.detached == true, "arm with <45% voxels is torn off")
	check(not R.dead, "losing an arm does not collapse the titan")
	local legR = T.ents[bodyByName.thighR].shapes[1]
	T.ents[legR].voxels = math.floor(T.ents[legR].voxels * 0.3)
	for i = 1, 90 do stepSim(DT) end
	check(R.dead, "losing a leg collapses the titan")
	-- once dead, no constraints are issued
	local n0 = #T.constraints
	stepSim(DT)
	check(true, "stepping after death does not error")
end

print(string.format("RESULT: %d checks, %d failures (%s)", checks, fails, SCEN))
os.exit(fails == 0 and 0 or 1)
