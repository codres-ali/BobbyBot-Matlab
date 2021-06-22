restoredefaultpath
addpath(genpath(pwd))

clear all;

% mode: RobotSimulation - simulate robot using dynamical mathematical model
%       RealRobot - communicate signals to puzzlebot
my_robot = Robot('mode','RealRobot');
my_robot.SetSamplingTime(0.01);

% voltage inputs to motors
uR = 7;
uL = 7;

wR = 0;

N = 200;

in = containers.Map();

tic
for k=1:N
    
    % Set input signals to be sent to robot
    in('VelocitySetR') = uR;
    in('VelocitySetL') = uL;
    
    out = my_robot.Run(in);
    
    % out contains all signals received from the robot
    if out.isKey('VelocityEncR')
        wR = out('VelocityEncR').data;
    end
    if out.isKey('VelocityEncL')
        wL = out('VelocityEncL').data;
    end
        
    wR_all(k) = wR;
    
end
toc
% stop simulation or communication
my_robot.Stop();

figure(1);
plot(wR_all);
hold on;
plot(uR);
