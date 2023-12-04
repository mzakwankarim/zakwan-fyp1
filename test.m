clear all
close all
clc
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    %create some joints pos
    joint_pos1=[-2*pi/3,0,0,0,pi/4,0];
    joint_pos2=[0,-pi/4,pi/2,0,0,0];
    joint_pos3=[0,0,0,0,0,0];
    
    %joints handles
    h=[0,0,0,0,0,0]
      [r,h(1)]=sim.simxGetObjectHandle(clientID,'rotary_head',sim.simx_opmode_blocking);
      [r,h(2)]=sim.simxGetObjectHandle(clientID,'lower_arm',sim.simx_opmode_blocking);
      [r,h(3)]=sim.simxGetObjectHandle(clientID,'upper_arm',sim.simx_opmode_blocking);
      [r,h(4)]=sim.simxGetObjectHandle(clientID,'forearm_twisting',sim.simx_opmode_blocking);
      [r,h(5)]=sim.simxGetObjectHandle(clientID,'wrist',sim.simx_opmode_blocking);
      [r,h(6)]=sim.simxGetObjectHandle(clientID,'axis6',sim.simx_opmode_blocking);
      
      while true
          %for i=1:6
          %sim.simxSetJointTargetPosition(clientID,h(i),joint_pos1(i),sim.simx_opmode_streaming);
          %end
          %pause(10);
          
          %for i=1:6
          %sim.simxSetJointTargetPosition(clientID,h(i),joint_pos2(i),sim.simx_opmode_streaming);
          %end
          %pause(10);
          
          for i=1:6
          sim.simxSetJointTargetPosition(clientID,h(i),joint_pos3(i),sim.simx_opmode_streaming);
          end
          pause(10);
      end
    
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!
    
disp('Program ended');