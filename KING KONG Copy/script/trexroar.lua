
function init()
	sniffer = FindJoint("sniffer")
	radar = FindJoint("radar")
	SetJointMotor(radar, 0.01)
	body = FindBody("target")

	board = FindShape("board")
	
	spring = FindJoint("spring")
	
	body = FindBody("springboard")

end

Timer = 0

function tick(dt)

	--local timer = GetFloat("springboard.timer")

	Timer = Timer + 1
	if Timer <= 90 then
	--SetFloat("springboard.timer", 2)
	SetJointMotorTarget(sniffer, -60, 1)
	elseif Timer == 130 then
	--SetFloat("springboard.timer", GetFloat("springboard.timer") - dt)
	SetJointMotorTarget(sniffer, 60, 0.3)
	--Delete(sniffer)
	elseif Timer == 130 then
	Timer = 0
	end
end