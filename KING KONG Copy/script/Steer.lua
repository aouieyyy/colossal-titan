function init()
	local shape = FindShape('steer')
	steerJoint = GetShapeJoints(shape)[1]
	scale = GetTagValue(shape,'steer')
	v = GetBodyVehicle(GetShapeBody(shape))
end
	
function tick()
	local steer = 0
	if v==GetPlayerVehicle()then
		steer = steer - InputValue("left")
		steer = steer + InputValue("right")
	end
	SetJointMotorTarget(steerJoint,35*scale*steer,math.pi/6/0.7*scale)
end