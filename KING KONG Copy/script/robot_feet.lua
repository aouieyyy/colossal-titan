--Made of 2 part legs: foot is bottom, leg is top
--Foot and leg usually does not collide to allow smooth movement, but collosion switched on when stunned for better ragdolls
--Waiting time between steps is the last 20% of the step, there's no waiting time before (actually unsure about what this 20% is)
--Step time / animation depends on the current speed and predicted location of the bot at the end of the step
--(note, the above looks good for walking, but legs lag behind with running fast like that)
--Pretty generalised as the rest of the code
--works regardless the number of feet
--Works at any speed of the robot
--Since movement is based on distance of ideal standing position from current, the stepping works automatically both for walking and turning in place]
--Probably min step distance should be lower

--overall animation idea
--first calculate new step position from predicting where the torso would end up at 
--average of current and desired speed. (and adjust a bit forward so stepping forward)
--if both feet are on the ground, step with the one that has further to go
--the non animated foot is held in place (no running)
--the rotation of the animated foot is arranged by transitioning to natural from current
--(could look better transitioning to a forward position)

feet = {}

function feetInit()
	local f = FindBodies("foot")
	for i=1, #f do
		local foot = {}
		foot.body = f[i]
		local t = GetBodyTransform(foot.body)
		local rayOrigin = TransformToParentPoint(t, Vec(0, 0.9, 0))
		local rayDir = TransformToParentVec(t, Vec(0, -1, 0))

		foot.lastTransform = TransformCopy(t)
		foot.targetTransform = TransformCopy(t)
		foot.candidateTransform = TransformCopy(t)
		foot.worldTransform = TransformCopy(t)
		foot.stepAge = 1 --A timer for the current step. (goes up with dt during the step)
		foot.stepLifeTime = 1 --Chosen time for the next step to take
		foot.localRestTransform = TransformToLocalTransform(robot.transform, t)
		foot.localTransform = TransformCopy(foot.localRestTransform)
		foot.rayOrigin = TransformToLocalPoint(robot.transform, rayOrigin)
		foot.rayDir = TransformToLocalVec(robot.transform, rayDir)
		foot.rayDist = hover.distTarget + hover.distPadding
		foot.contact = true
		local mass = GetBodyMass(foot.body)
		foot.linForce = 20 * mass
		foot.angForce = 1 * mass
		local linScale, angScale = getTagParameter2(foot.body, "force", 1.0)
		foot.linForce = foot.linForce * linScale
		foot.angForce = foot.angForce * angScale
		feet[i] = foot
	end
end

--Foot and leg usually does not collide to allow smooth movement, but collosion switched on when stunned for better ragdolls
--Still ragdolls don't look great
function feetCollideLegs(enabled)
	local mask = 0
	if enabled then
		mask = 253
	end
	local feet = FindBodies("foot")
	for i=1, #feet do
		local shapes = GetBodyShapes(feet[i])
		for j=1, #shapes do
			SetShapeCollisionFilter(shapes[j], 2, mask)
		end
	end
	local legs = FindBodies("leg")
	for i=1, #legs do
		local shapes = GetBodyShapes(legs[i])
		for j=1, #shapes do
			SetShapeCollisionFilter(shapes[j], 2, mask)
		end
	end
	for i=1, #wheels.bodies do
		local shapes = GetBodyShapes(wheels.bodies[i])
		for j=1, #shapes do
			SetShapeCollisionFilter(shapes[j], 2, mask)
		end
	end
end

-----------------------------------------------------------------------------------------------------------------------------------------------------------------
--!!! This is the the main part to change to adjust step characteristics
--The general shape of the step is adjusted in animation (less important)
FOOT_SPEED_FACTOR = 2.5 --The feet must move faster than the body for steps to work. This is how much faster they move
LEG_DIST_FACTOR = 5 --a factor between step size and movement speed, affects general length of steps
MIN_STEP_DISTANCE = 0.35 --Ideal step position has to be at least this far to initiate step, otherwise there is no reason to take a step
MAX_STEP_DISTANCE = 1.8
STEP_HEIGHT_FACTOR = 0.4 --the maximum height of the step depends on the length of the step, and it is much less (steps are pretty flat usually)
MAX_STEP_TIME = 1.2 --max time a step can take, note there's no min time, the steps can theoretically keep up with any velocity
STEP_OVERLAP_FRACTION = 0.2 --at the end of one step, before fully finishing the step, it can already start a step with the other leg
--when looking where to step, the prediction starts by calculating where the torso will likely end up, and compared to that where the foot should be
--However, that way at the end of the step the foot would end up exactly below the torso (does not look good)
--This factor increases the step distance so that it is forward from the torso at the end of the step
--if it's too low, the legs tend to lag behind, if too high, they end up being too forward. Ideally this should increase with speed, but that didn't work in practice
STEP_FORWARD_FACTOR = 2.5
UPWARD_OFFSET_FACTOR = 0.2 --when stepping the torso moves upward not just the leg, it moves up this fraction compared to what fraction the leg moves up
-----------------------------------------------------------------------------------------------------------------------------------------------------------------


--takes a linear function as the input and the output is a smoothened function, ramps up slowly and slows down toward the end
--while still looking quite close to the linear original
function Smoothen(q)
	return q * q * (3.0 - 2.0 * q) 
end

function feetUpdate(dt)
	if robot.stunned > 0 then
		feetCollideLegs(true)
		return
	else
		feetCollideLegs(false)
	end

	UpwardOffset = 0 --will recalculate height offset based on stepping

	--Main part of the code where important step characteristics are calculated
	local vel = GetBodyVelocity(robot.body)
	local MovementSpeed = VecLength(vel)  --magnitude of velocity affects step size, how many meters in one second for body
	--actual distance of step depends on velocity, within reasonable values, foot velocity is higher than body velocity
	--Will not step unless found a candidate step this far
	local stepLength = clamp(MovementSpeed*LEG_DIST_FACTOR, MIN_STEP_DISTANCE, MAX_STEP_DISTANCE) 
	local stepTime = math.min(stepLength / MovementSpeed /FOOT_SPEED_FACTOR, MAX_STEP_TIME)  --how long the step will take. There's a max time for a step, but usually depends on distance and stepping velocity
	local stepHeight = stepLength * STEP_HEIGHT_FACTOR --The height of the step is proportional to distance, small steps mean also small vertical movement

	--StepFraction is the fraction of the current step, how much time passed compared to maximum
	--Animation goes from stepfraction = 0 to 1
	--There is some overlap between steps. (20% in vanilla) when one leg is 80% finished stepping, the other leg can start stepping
	--The only time stepping can start, is when both steps are (almost) on the ground (ie inStep == false)
	local inStep = false
	for i=1, #feet do
		local StepFraction = feet[i].stepAge/feet[i].stepLifeTime
		if feet[i].stepLifeTime > stepTime then
			feet[i].stepLifeTime = stepTime
		end
		if StepFraction < 1-STEP_OVERLAP_FRACTION then
			inStep = true
		end
	end
	
	--For both feet do the animation (if appropriate) and check where it could be stepping next if currently standing
	for i=1, #feet do
		local foot = feet[i]
		
		if not inStep then
			--Find candidate footstep

			local tPredict = TransformCopy(robot.basetransform) --starting from current transform of robot, guess where robot will be
			local vPredict = VecScale(robot.dir, robot.speed) --predicted velocity starts at desired velocity
			vPredict = VecLerp(vPredict, vel, 0.5) --predicted velocity is the average of desired and current velocity (bold assumption)
			--predicted velocity is scaled by stepTime, to get predicted distance (v = s/t)
			--it is also scaled by 1.5 (STEP_FORWARD_FACTOR) so the step is a bit to the front when it finishes, not exactly below the robot
			--Prediction does not take angular velocity into account (maybe it's fine: turning only happens when desired position is at an angle anyway)
			local sPredict = VecScale(vPredict, stepTime*STEP_FORWARD_FACTOR)
			tPredict.pos = VecAdd(tPredict.pos, sPredict)

			--take into account turning (max degree in a step)
			local d = FaceDiff --face diff is +-1 at back, 0.5 at 90 deg
			local maxdiff = 0.2
			if d<-maxdiff then d = -maxdiff end
			if d>maxdiff then d = maxdiff end
			tPredict.rot = QuatRotateQuat(QuatEuler(0,d*180,0),tPredict.rot) --assuming rotation is to right with negative

			local rayOrigin = TransformToParentPoint(tPredict, foot.rayOrigin)
			local rayDir = TransformToParentVec(tPredict, foot.rayDir)
			QueryRequire("physical large")
			rejectAllBodies(robot.allBodies)
			local hit, dist, normal, shape = QueryRaycast(rayOrigin, rayDir, foot.rayDist)
			local targetTransform = TransformToParentTransform(robot.basetransform, foot.localRestTransform)
			if hit then
				targetTransform.pos = VecAdd(rayOrigin, VecScale(rayDir, dist))
			end
			foot.candidateTransform = targetTransform
		end

		--Animate foot
		if hover.contact > 0.7 then
			if foot.stepAge < foot.stepLifeTime then
				foot.stepAge = math.min(foot.stepAge + dt, foot.stepLifeTime)
				local StepFraction = foot.stepAge / foot.stepLifeTime
				-- smoothstep: before this step, StepFraction is the linear step fraction, and the leg position will depend on StepFraction
				--However using a linear function looks unnatural (starts at full speed and suddenly stops)
				StepFraction = Smoothen(StepFraction) 
				local p = VecLerp(foot.lastTransform.pos, foot.targetTransform.pos, StepFraction)
				--For the stepheight, the sin function is used, (goes 0 to 0 while StepFraction goes 0-1) it is an even upward curve
				local h = math.sin(math.pi * StepFraction)*stepHeight
				p[2] = p[2] + h
				if h> UpwardOffset then UpwardOffset = h end
				local r = QuatSlerp(foot.lastTransform.rot, foot.targetTransform.rot, StepFraction)
				foot.worldTransform = Transform(p, r)
				foot.localTransform = TransformToLocalTransform(robot.transform, foot.worldTransform) --not used
				if foot.stepAge == foot.stepLifeTime then
					PlaySound(stepSound, p, 0.5, false)
				end
			end
			ConstrainPosition(foot.body, robot.body, GetBodyTransform(foot.body).pos, foot.worldTransform.pos, 8, foot.linForce)
			ConstrainOrientation(foot.body, robot.body, GetBodyTransform(foot.body).rot, foot.worldTransform.rot, 16, foot.angForce)
		end

	end

	UpwardOffset = UpwardOffset * UPWARD_OFFSET_FACTOR

	--From each of the legs, choose the best one to step next (the best one is the one that is a bigger step)
	if not inStep then
		--Find best step candidate
		local bestFoot = 0
		local bestDist = 0
		for i=1, #feet do
			local foot = feet[i]
			local dist = VecLength(VecSub(foot.targetTransform.pos, foot.candidateTransform.pos))
			if dist > stepLength and dist > bestDist then
				bestDist = dist
				bestFoot = i
			end
		end
		--Initiate best footstep
		if bestFoot ~= 0 then
			local foot = feet[bestFoot]
			foot.lastTransform = TransformCopy(GetBodyTransform(foot.body))
			foot.targetTransform = TransformCopy(foot.candidateTransform)
			foot.stepAge = 0
			foot.stepLifeTime = stepTime
		end
	end
end
