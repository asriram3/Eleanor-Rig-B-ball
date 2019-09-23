function sysCall_init() 
    -- First time we execute this script. 

    -- Make sure we have version 2.4.12 or above (the omni-wheels are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20412) then
        sim.displayDialog('Warning','The YouBot model is only fully supported from V-REP version 2.4.12 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObjectHandle('rollingJoint_fl')
    wheelJoints[2]=sim.getObjectHandle('rollingJoint_rl')
    wheelJoints[3]=sim.getObjectHandle('rollingJoint_rr')
    wheelJoints[4]=sim.getObjectHandle('rollingJoint_fr')
    p_sensor = sim.getObjectHandle('Proximity_sensor')
    youBot=sim.getObjectHandle('youBot')
    youBotRef=sim.getObjectHandle('youBot_ref')
    tip=sim.getObjectHandle('youBot_positionTip')
    target=sim.getObjectHandle('youBot_positionTarget')
    armJoints={-1,-1,-1,-1,-1}
    for i=0,4,1 do
        armJoints[i+1]=sim.getObjectHandle('youBotArmJoint'..i)
    end
    ui=simGetUIHandle('youBot_UI')
    simSetUIButtonLabel(ui,0,sim.getObjectName(youBot)..' user interface') -- Set the UI title (with the name of the current robot)
    ik1=sim.getIkGroupHandle('youBotUndamped_group')
    ik2=sim.getIkGroupHandle('youBotDamped_group')
    ikFailedReportHandle=-1
    forwBackVelRange={-240*math.pi/180,240*math.pi/180}  -- min and max wheel rotation vel. for backward/forward movement
    leftRightVelRange={-240*math.pi/180,240*math.pi/180} -- min and max wheel rotation vel. for left/right movement
    rotVelRange={-240*math.pi/180,240*math.pi/180}       -- min and max wheel rotation vel. for left/right rotation movement

    forwBackVel=0
    leftRightVel=0
    rotVel=0
    initSizeFactor=sim.getObjectSizeFactor(youBot) -- only needed if we scale the robot up/down
    
    -- desired joint positions, and desired cartesian positions:
    desiredJ={0,30.91*math.pi/180,52.42*math.pi/180,72.68*math.pi/180,0} -- when in FK mode
    for i=1,5,1 do
        sim.setJointPosition(armJoints[i],desiredJ[i])
    end
    desiredPos={0,0,0} -- when in IK mode
    currentPos={0,0,0} -- when in IK mode
    ikMinPos={-0.5*initSizeFactor,-0.2*initSizeFactor,-0.3*initSizeFactor}
    ikRange={1*initSizeFactor,1*initSizeFactor,0.9*initSizeFactor}

    -- We compute the initial position and orientation of the tip RELATIVE to the robot base (because the base is moving)
    initialTipPosRelative=sim.getObjectPosition(tip,youBotRef)--youBot)
    ikMode=false -- We start in FK mode
    maxJointVelocity=40*math.pi/180 
    maxPosVelocity=0.1*initSizeFactor
    previousS=initSizeFactor
    
    gripperCommunicationTube=sim.tubeOpen(0,'youBotGripperState'..sim.getNameSuffix(nil),1)




end
-- This example script is non-threaded (executed at each simulation pass)
-- The functionality of this script (or parts of it) could be implemented
-- in an extension module (plugin) and be hidden. The extension module could
-- also allow connecting to and controlling the real robot.


function sysCall_cleanup() 
 
end 

startTime = sim.getSystemTime()
lastTime = startTime
totalTime = 0
state = 0

function sysCall_actuation() 
    -- Code for moving in a square

    --[[
    startTime = sim.getSystemTime()
    totalTime = totalTime + (startTime - lastTime)
    if (totalTime > 3) then
        totalTime = 0
        if (state == 3) then
            state = 0
        else
            state = state + 1
        end
    end

    if (state == 0) then
        forwBackVel = 1
        leftRightVel = 0
    elseif (state == 1) then
        forwBackVel = 0
        leftRightVel = 1
    elseif (state == 2) then        
        forwBackVel = -1
        leftRightVel = 0
    elseif (state == 3) then
        forwBackVel = 0
        leftRightVel = -1
    end
    --]]


    -- code for moving towards the wall

    result, distance, point, handle, norm = sim.readProximitySensor(p_sensor)
    print(distance)
    forwBackVel = -1
    if (distance ~= nil) then
        forwBackVel = -distance*2
        if (distance < 0.15) then
            forwBackVel = 0
        end
    end
    
    -- Now apply the desired wheel velocities:
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)

    lastTime = startTime
end 
