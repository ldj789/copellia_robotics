"""
function sysCall_init()

    bubbleRobBase=sim.getObjectAssociatedWithScript(sim.handle_self)
    leftMotor=sim.getObjectHandle("bubbleRob_leftMotor")
    rightMotor=sim.getObjectHandle("bubbleRob_rightMotor")
    noseSensor=sim.getObjectHandle("bubbleRob_sensingNose")
    minMaxSpeed={50*math.pi/180,300*math.pi/180}
    backUntilTime=-1 -- Tells whether bubbleRob is in forward or backward mode
    floorSensorHandles={-1,-1,-1}
    floorSensorHandles[1]=sim.getObjectHandle("bubbleRob_leftSensor")
    floorSensorHandles[2]=sim.getObjectHandle("bubbleRob_middleSensor")
    floorSensorHandles[3]=sim.getObjectHandle("bubbleRob_rightSensor")
    -- Create the custom UI:
    xml = '<ui title="'..sim.getObjectName(bubbleRobBase)..' speed" closeable="false" resizeable="false" activate="false">'..[[
                <hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>
            <label text="" style="* {margin-left: 300px;}"/>
        </ui>
        ]]
    ui=simUI.create(xml)
    speed=(minMaxSpeed[1]+minMaxSpeed[2])*0.5
    simUI.setSliderValue(ui,1,100*(speed-minMaxSpeed[1])/(minMaxSpeed[2]-minMaxSpeed[1]))

end
function speedChange_callback(ui,id,newVal)
    speed=minMaxSpeed[1]+(minMaxSpeed[2]-minMaxSpeed[1])*newVal/100
end



function sysCall_actuation()
    result=sim.readProximitySensor(noseSensor)
    if (result>0) then backUntilTime=sim.getSimulationTime()+4 end

    -- read the line detection sensors:
    sensorReading={false,false,false}
    for i=1,3,1 do
        result,data=sim.readVisionSensor(floorSensorHandles[i])
        print(result,data)
        if (result>=0) then
            sensorReading[i]=(data[11]<0.3) -- data[11] is the average of intensity of the image
            --print(data[11])
        end
    end

    -- compute left and right velocities to follow the detected line:
    rightV=speed
    leftV=speed
    if sensorReading[1] then
        leftV=0.03*speed
    end
    if sensorReading[3] then
        rightV=0.03*speed
    end
    if sensorReading[1] and sensorReading[3] then
        backUntilTime=sim.getSimulationTime()+2
    end

    if (backUntilTime<sim.getSimulationTime()) then
        -- When in forward mode, we simply move forward at the desired speed
        sim.setJointTargetVelocity(leftMotor,leftV)
        sim.setJointTargetVelocity(rightMotor,rightV)
    else
        -- When in backward mode, we simply backup in a curve at reduced speed
        sim.setJointTargetVelocity(leftMotor,-speed/2)
        sim.setJointTargetVelocity(rightMotor,-speed/8)
    end
end

function sysCall_cleanup()
    simUI.destroy(ui)
end
"""

import sys
import time  # used to keep track of time
import numpy as np  # array library

import sim
from sensors.odometery import Odometer

# Pre-Allocation
PI = np.pi  # constant
op_mode = sim.simx_opmode_oneshot_wait
sim.simxFinish(-1)  # just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:  # check if client connection successful
    print('Connected to remote API server')

else:
    print('Connection not successful')
    sys.exit('Could not connect')

odometer = Odometer(clientID, .075)

# retrieve motor  handles
_, left_motor_handle = sim.simxGetObjectHandle(clientID, 'bubbleRob_leftMotor', op_mode)
_, right_motor_handle = sim.simxGetObjectHandle(clientID, 'bubbleRob_rightMotor', op_mode)
_, noseSensor = sim.simxGetObjectHandle(clientID, 'bubbleRob_sensingNose', op_mode)

floorSensorHandles = [-1, -1, -1]
_, floorSensorHandles[0] = sim.simxGetObjectHandle(clientID, 'bubbleRob_leftSensor', op_mode)
_, floorSensorHandles[1] = sim.simxGetObjectHandle(clientID, 'bubbleRob_middleSensor', op_mode)
_, floorSensorHandles[2] = sim.simxGetObjectHandle(clientID, 'bubbleRob_rightSensor', op_mode)

minMaxSpeed = [50*PI/180, 300*PI/180]
speed = minMaxSpeed[0] + (minMaxSpeed[1] - minMaxSpeed[0]) * 0.40
backUntilTime = -1  # Forward or back mode

# start time
t = time.time()

while (time.time() - t) < 45:
    # Update odometers
    # odometer.update_motors()
    # print(f"{odometer}")

    # Loop Execution
    resCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector \
        = sim.simxReadProximitySensor(clientID, noseSensor, op_mode)
    result = np.linalg.norm(detectedPoint)
    if result > 0:
        backUntilTime = sim.simxGetLastCmdTime(clientID) + 4

    # read the line detection sensors:
    sensorReading = [0, 0, 0]
    sensorBool = [False, False, False]
    for i, vision_handle in enumerate(floorSensorHandles):
        result, detection_state, aux_packets = sim.simxReadVisionSensor(clientID, vision_handle, op_mode)
        print(f"result: {result}, ds: {detection_state}, packets: {aux_packets}")
        if result >= 0:
            sensorReading[i] = aux_packets[0][10]
            sensorBool[i] = aux_packets[0][10] < 0.3  # data[11] is the average of intensity of the image
            # print(data[11])

    # compute left and right velocities to follow the detected line:
    rightV = speed
    leftV = speed

    print(sensorReading, '\n', sensorBool)

    if sensorBool[0]:
        leftV = 0.03 * speed

    if sensorBool[2]:
        rightV = 0.03 * speed

    if sensorBool[0] and sensorBool[2]:
        backUntilTime = sim.simxGetLastCmdTime(clientID) + 3

    # Forward Mode, because backUntilTime is less than now i.e. before
    if backUntilTime < sim.simxGetLastCmdTime(clientID):
        # When in forward mode, we simply move forward at the desired speed
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle, leftV, op_mode)
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle, rightV, op_mode)
    else:
        # When in backward mode, we simply backup in a curve at reduced speed
        sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -speed/2, op_mode)
        sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -speed/8, op_mode)

# STOP
_ = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
_ = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)
