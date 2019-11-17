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


    ball=sim.getObjectHandle('ball')
    youbot = sim.getObjectHandle('youBot')
    back_right = sim.getObjectHandle('wheel_respondable_rr')
    front_right = sim.getObjectHandle('wheel_respondable_fr')



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
prev_ball_pos = sim.getObjectPosition(sim.getObjectHandle('ball'), 0)

-- State 1, rotate towards ball

function sysCall_actuation() 

    ball_pos = sim.getObjectPosition(ball, 0)

    my_pos = sim.getObjectPosition(youbot, 0)
    

    --my_rot = sim.getObjectRotation(youbot, 0)
    --print(my_rot)

    dx = my_pos[1] - ball_pos[1]
    dy = my_pos[2] - ball_pos[2]
    dz = my_pos[3] - ball_pos[3]


    front_right_pos = sim.getObjectPosition(front_right, 0)
    back_right_pos = sim.getObjectPosition(back_right, 0)

    dwx = back_right_pos[1] - front_right_pos[1]
    dwy = back_right_pos[2] - front_right_pos[2]
    
    print(state)
    if(math.abs(prev_ball_pos[1] - ball_pos[1]) > 0.1) then
        state = 0
    end

    if(state == 0) then
        
        target_angle = math.atan2(dy, dx)
        my_angle = math.atan2(dwy, dwx)
        ang_dif = target_angle - my_angle
        rotVel = 5*(ang_dif);
        --print(my_rot[2] * 180 / 3.1415)
        --print(target_angle * 180 / 3.1415)
            --print('--')   
        dist = math.sqrt(dy * dy + dx * dx)
        forwBackVel = 5*(dist * dist)
        print(dist)
        
        if(math.abs(ang_dif) > 1.7) then
            rotVel = 5;
            forwBackVel = 0;
        end

        if(math.abs(ang_dif) < 0.01 and dist < 4) then
            state = 1
            rotVel = 0
            forwBackVel = 0
        end
    else
        rotVel = 0
        forwBackVel = 0

         -- We are in IK mode
        maxVariationAllowed=maxPosVelocity*sim.getSimulationTimeStep()
        deltaX={0,0,0}
        -- position:
        desiredPos = {dx, dy, dz}
        for i=1,3,1 do
            delta=desiredPos[i]-currentPos[i]
            if (math.abs(delta)>maxVariationAllowed) then
                delta=maxVariationAllowed*delta/math.abs(delta) -- we limit the variation to the maximum allowed
            end
            deltaX[i]=delta
        end
    
        currentPos={currentPos[1]+deltaX[1],currentPos[2]+deltaX[2],currentPos[3]+deltaX[3]}
    
        pos={initialTipPosRelative[1]+currentPos[1],initialTipPosRelative[2]+currentPos[2],initialTipPosRelative[3]+currentPos[3]}
        -- We set the desired position and orientation
        sim.setObjectPosition(target,youbot,ball_pos)--youBot,pos)
    
        if (sim.handleIkGroup(ik1)==sim.ikresult_fail) then
            -- the position could not be reached.
            sim.handleIkGroup(ik2) -- Apply a damped resolution method
            if (ikFailedReportHandle==-1) then -- We dispblay a IK failure (in pos) report message
                ikFailedReportHandle=sim.displayDialog("IK failure report","IK solver failed.",sim.dlgstyle_message,false,"",nil,{1,0.7,0,0,0,0})
            end
        else
            if (ikFailedReportHandle>=0) then
                sim.endDialog(ikFailedReportHandle) -- We close any report message about IK failure in orientaion
                ikFailedReportHandle=-1
            end
        end
        -- Now update the desiredJ in case we switch back to FK mode:
        for i=1,5,1 do
            desiredJ[i]=sim.getJointPosition(armJoints[i])
        end 



    end

    prev_ball_pos = ball_pos

    -- code for moving towards the wall
--
--    result, distance, point, handle, norm = sim.readProximitySensor(p_sensor)
--    --print(distance)
--    forwBackVel = -1
--    if (distance ~= nil) then
--        forwBackVel = -distance*2
--        if (distance < 0.15) then
--            forwBackVel = 0
--        end
--    end

    
    -- Now apply the desired wheel velocities:
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)

    lastTime = startTime



end 
