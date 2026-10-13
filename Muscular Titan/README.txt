MUSCULAR TITAN  -  Teardown content mod
=======================================

A 59 m tall muscular titan (3.0 x King Kong, who is 19.7 m) that walks after you.

INSTALL
  1. Copy the whole folder "Muscular Titan" into
        Documents\Teardown\mods\
     (the folder must contain info.txt, spawn.txt, titan.xml, script\ and vox\ directly).
  2. Start Teardown -> Play -> Mods -> Local files -> select "Muscular Titan" -> Enable.

SPAWN
  Start any sandbox level, press Esc, open "Spawn" at the bottom, and choose
        Titans  >  Muscular Titan
  (the same Spawn menu King Kong uses). Spawn it on open ground; it appears facing you and
  starts walking towards you immediately. It can be spawned any number of times.

WHAT IT DOES
  * walks after the player (alternating steps, planted feet, restrained arm swing,
    chest twist / weight shift, stable head); stops near you and stomps when you are in reach
  * cuts through walls that block it, steps with 60 dB footsteps, shakes the camera, leaves
    small footprints and hurts the player when a stomping foot lands next to them
  * made of very tough materials (hard metal / heavy metal / metal / hard masonry / rock /
    masonry) with xml strength 100 and density 5,000,000 - but NOT unbreakable:
      - a limb that lost more than 55 % of its voxels is torn off
      - if a leg, the pelvis, chest or head is destroyed, or the titan lost 35 % of all voxels,
        it collapses

SETTINGS
  script\titan.lua, table CFG at the top:  SPEED, TURN_SPEED, STOP_DISTANCE, DETECT_DISTANCE,
  step timing / height, arm swing, COLLAPSE_FRACTION, LIMB_BREAK_FRACTION, ATTACK_RANGE,
  STOMP_DAMAGE, FOOTPRINT, FOOTSTEP_SHAKE ...   (param0 / param1 in titan.xml also set speed and turn speed)
  titan.xml: strength= / density= on every <vox> (1 = normal material strength).

REBUILD / RESIZE
  titan_tools\build_titan.py regenerates everything from muscle_titan.vox
  (TARGET_KONG_MULTIPLE, STRENGTH, DENSITY, OVERLAP at the top of that file).

KNOWN LIMITS
  * Damage resistance cannot be configured separately per damage type (explosion / impact /
    bullets) in Teardown; hardness is set by material + strength, the rest by CFG thresholds.
  * The model is a hollow 1-2 voxel thick skin (as supplied); rotating limbs can show thin
    slivers at the joints. Joints overlap by ~0.4 m to keep them closed.
  * This mod has been verified by code/asset inspection and an offline simulation, not in Teardown.
