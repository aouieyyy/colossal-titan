function init()
    a = GetPlayerCameraTransform()
    sb = FindBody("missile")
    maxheight = 8
    boc = 20
    absolute = false
    b = FindShape("bby")
    tim = 0
end

function tick(dt)
	tim = tim + dt
    -- These need to be updated each tick for their new values/positions.
    strans = GetShapeWorldTransform(sc)
    spos = strans.pos
    flydirection = TransformToParentVec(strans, Vec(0, -1, 0))
    sbt = GetBodyTransform(sb)
    local trVec = Vec(0,0, -boc)
    local trVel = TransformToParentPoint(sbt, trVec)
    local velSub = VecSub(trVel, sbt.pos)
    local bias = Vec(0, 0, 0)
    if not absolute then
        if (sbt.pos[2] < maxheight) then
            bias[2] = boc
            boc = boc
        end
        local currentVelocity = GetBodyVelocity(sb)
        SetBodyVelocity(sb, velSub)
    end
    if IsShapeBroken(b) then
    absolute = true
end
if boc > 100 then
	boc = 100
end
DistanceToPlayer = VecLength(VecSub(sbt.pos, GetPlayerTransform().pos))
if DistanceToPlayer < 20 then
	Triggered = true
end

pummel()
if tim < 0.5 then
	sbt.rot = QuatEuler(90, 0, 0)
	SetBodyTransform(sb, sbt)
	Triggered = false
end
if tim > 2 and not Triggered then
rot = QuatLookAt(sbt.pos, GetPlayerTransform().pos)
sbt.rot = rot
SetBodyTransform(sb, sbt)
end
end

function pummel()
	ParticleColor(0.9,0.9,0.9)
          ParticleType("smoke")
        ParticleEmissive(0)
        ParticleRadius(0.2,0.4)
        ParticleCollide(1)
        ParticleAlpha(1)
        ParticleCollide(1)
        ParticleSticky(0.1)
        ParticleGravity(0)
        ParticleStretch(5)
        ParticleDrag(0)
        SpawnParticle(sbt.pos, Vec(math.random(-1,1), math.random(-1,1), math.random(-1,1)), 2)
     ParticleReset()
end