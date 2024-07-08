% assignment1PartC.m
% MTRN4230 Assignment 1 24T2
% Name: 
% Zid:  zXXXXXXX

%% Initialisation
clear all;
clc; close all;

host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
% host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;

%% Use the code here to test your function implementation
% you may change it as much as required, this will not be submitted
rtde = rtde(host, port);

v = 0.25;
a = 1.2;

% Creating individual waypoints
path_pose1 = [[-588.53, -133.30, 200, 2.2214, -2.2214, 0.00], a, v, 0, 0.0];
path_pose2 = [[-688.53, -133.30, 200, 2.2214, -2.2214, 0.00], a, v, 0, 0.0];
path_pose3 = [[-688.53, -233.30, 200, 2.2214, -2.2214, 0.00], a, v, 0, 0.0];
path_pose4 = [[-588.53, -233.30, 200, 2.2214, -2.2214, 0.00], a, v, 0, 0.0];

% Creating the path
path = [path_pose1; path_pose2; path_pose3; path_pose4; path_pose1];

% Performing motion
pause(1); % Small pause to stop rtde/Matlab skipping recording values
[poses,joints,jointVelocities,jointAccelerations,torques] = rtde.movej(path,'pose');

% Function call for implementation
v_max = calculateMaxLinearVelocity(joints, jointVelocities);

disp("Max linear velocity achieved: ");
disp(v_max);

rtde.close();