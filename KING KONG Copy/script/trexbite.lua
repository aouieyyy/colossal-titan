
function init()

    playerTransform = Transform()
    playerPos = Vec()

	biteSound = LoadSound("MOD/snd/bite.ogg", 9.0)
	biteSound2 = LoadSound("MOD/snd/bite2.ogg", 9.0)
end


function tick(dt)

    playerTransform = GetPlayerTransform()
    playerPos = VecAdd(playerTransform.pos, Vec(0, 1, 0))
    cameraTransform = GetPlayerCameraTransform()
    cameraDir = TransformToParentVec(cameraTransform, Vec(0, 0, -1))

    local t = GetCameraTransform()
	local pos = t.pos

	if GetPlayerHealth() <= 0 then
		if not playing then
			PlaySound(biteSound, playerPos, 0.5, false)
			PlaySound(biteSound2, playerPos, 15, false)
			ShakeCamera(1)
			--PlaySound(swing)
			playing = true
		end
	elseif GetPlayerHealth() >= 0 then
		if playing then
			playing = false
		end
	end	
end	