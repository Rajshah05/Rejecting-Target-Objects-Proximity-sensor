vrep=remApi('remoteApi');
disp('program started');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19999,true,false,10000,5);
if(clientID>-1)
    disp('connected to remote API server');
end

[res1,joint]=vrep.simxGetObjectHandle(clientID, 'Prismatic_joint',vrep.simx_opmode_oneshot_wait);
[res2,sensor]=vrep.simxGetObjectHandle(clientID, 'Proximity_sensor0',vrep.simx_opmode_oneshot_wait);
[res3,sensorJoint]=vrep.simxGetObjectHandle(clientID, 'Proximity_sensor',vrep.simx_opmode_oneshot_wait);
[res4,cylinderOuter]=vrep.simxGetObjectHandle(clientID, 'Cylinder2',vrep.simx_opmode_oneshot_wait);
[res5,cylinderInner]=vrep.simxGetObjectHandle(clientID, 'Cylinder3',vrep.simx_opmode_oneshot_wait);

sd=1;
store_sensor=[];
while 1
    [returnCode1,detectionState_sensor,detectedPoint_sensor,detectedObjectHandle_sensor,detectedSurfaceNormalVector_sensor]=vrep.simxReadProximitySensor(clientID,sensor,vrep.simx_opmode_oneshot_wait);
    [returnCode2,detectionState_sensorJoint,detectedPoint_sensorJoint,detectedObjectHandle_sensorJoint,detectedSurfaceNormalVector_sensorJoint]=vrep.simxReadProximitySensor(clientID,sensorJoint,vrep.simx_opmode_oneshot_wait);
    if detectionState_sensor==0
        continue
    end
    while detectionState_sensor==1
        [returnCode3,detectionState_sensor,detectedPoint_sensor,detectedObjectHandle_sensor,detectedSurfaceNormalVector_sensor]=vrep.simxReadProximitySensor(clientID,sensor,vrep.simx_opmode_oneshot_wait);
        [returnCode4,detectionState_sensorJoint,detectedPoint_sensorJoint,detectedObjectHandle_sensorJoint,detectedSurfaceNormalVector_sensorJoint]=vrep.simxReadProximitySensor(clientID,sensorJoint,vrep.simx_opmode_oneshot_wait);
        store_sensor=[store_sensor;detectedPoint_sensor(1,3)];
    end
    sd=std(store_sensor(1:5));
    while detectionState_sensor==0
        [returnCode5,detectionState_sensor,detectedPoint_sensor,detectedObjectHandle_sensor,detectedSurfaceNormalVector_sensor]=vrep.simxReadProximitySensor(clientID,sensor,vrep.simx_opmode_oneshot_wait);
        [returnCode6,detectionState_sensorJoint,detectedPoint_sensorJoint,detectedObjectHandle_sensorJoint,detectedSurfaceNormalVector_sensorJoint]=vrep.simxReadProximitySensor(clientID,sensorJoint,vrep.simx_opmode_oneshot_wait);
        if sd<0.003
            if detectionState_sensorJoint==1
                [returnCode7]=vrep.simxSetJointPosition(clientID,joint,1,vrep.simx_opmode_oneshot_wait);
                [returnCode8]=vrep.simxSetJointPosition(clientID,joint,0,vrep.simx_opmode_oneshot_wait);
            end
        end
    end
    sd=0;
    store_sensor=[];
end
