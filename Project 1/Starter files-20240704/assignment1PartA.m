% assignment1PartA.m
% MTRN4230 Assignment 1 24T2
% Name: Vinoshan Parathan
% Zid:  z5358838

clear; clc;

%host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
rtde = rtde(host, port);

disp("Enter the pickup position")
pickupJointConfiguration = readConfiguration();

clc;
disp("Move robot to dropoff position")
dropoffJointConfiguration = readConfiguration();

clc;
disp("Calculated pickup pose: ")
pickupPose = convertJointToPose(pickupJointConfiguration)
disp("Calculated dropoff pose: ")
dropoffPose = convertJointToPose(dropoffJointConfiguration)

disp("Set robot to remote control mode then click enter")
input('');


% RTDE says it's taking in [x,y,z,r,p,y] but 
% its actually taking in [x,y,z,(rotation vector)]
% 
% The below four lines converts students rpy pose, into one with a rotation
% vector 
Tp = rpy2tr(pickupPose(4:6))
pickupPose = [pickupPose(1:3), rotmat2vec3d(Tp(1:3, 1:3))]'

Td = rpy2tr(dropoffPose(4:6))
dropoffPose = [dropoffPose(1:3), rotmat2vec3d(Td(1:3, 1:3))]'


rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(pickupPose', 'pose')
rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')

rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(dropoffPose', 'pose')
rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')

% Function to convert user input to array
function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j3,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% You must implement the following function
function outputPose = convertJointToPose(jointConfiguration)
    alpha=[pi/2 0 0 pi/2 -pi/2 0]; % in radian
    d = [162.5 0 0 133.3 99.7 99.6];
    a = [0 -425 -392.2 0 0 0]; 
    theta = deg2rad(jointConfiguration);
    Final_T_matrix = eye(4);

    for i=1:6
        T_matrix = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)), sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
                    sin(theta(i)), cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
                    0, sin(alpha(i)), cos(alpha(i)), d(i);
                    0, 0, 0, 1];
        Final_T_matrix = Final_T_matrix*T_matrix;
    end
%T0to1 = [cos]

    position = Final_T_matrix(13:15);
    orientation = tr2rpy(Final_T_matrix(1:3,1:3));
% Replace this with your implementation    
    outputPose = [position, orientation];
    
    % You must not use RTDE at all in this implementation (it
    % must be done from first principles)
end