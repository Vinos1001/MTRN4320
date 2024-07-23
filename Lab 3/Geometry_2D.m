% Author: Raghav Hariharan
% Based of video content from the Robot Academy
% It is recommended to run this program one section at a time
startup_rvc;

%% Describing Rotation in 2D
% Source: https://robotacademy.net.au/masterclass/2d-geometry/?lesson=75
% Create a 2D rotation matrix 
% rotation matrix = [[cos(theta), sin(theta)];
%                    [sin(theta), cos(theta]];

A = rot2(0) % with 0 radians

B = rot2(0.2) % rotation angle with 0.2 radians

C = rot2(30,'deg') % rotation angle with 30 degrees

rot2(deg2rad(30)) % rotation angle with 30 degrees

% Plot the rotation axis using trplot2
% trplot2 : Plots a 2D coordinate frame

hold on;
trplot2(A, 'color','r'); 
trplot2(B, 'color','g');
trplot2(C, 'color','b');

%% Describing Rotation and Translation in 2D
% Source: https://robotacademy.net.au/masterclass/2d-geometry/?lesson=76

% transl2: Represents a homogeneous transformation matrix which represents
% a pure translation. So no rotation at all

D = transl2(1,2) % x = 1, y = 2

% Question: What is the difference between rot2 and trot2?
% What is the size of matrix D. What is the size of the rotation matrix
% returned by rot2 and trot2?
rot2(30, 'deg')

trot2(30, 'deg')

% Lets create the transformation matrix
transformation = transl2(1,2)*trot2(30, 'deg')

% The toolbox provides an easier way to create a 2D transformation matrix
% Using the "SE2" function
T = SE2(1,2,30,'deg')

%% Pose Transformation Example

% Set up the coordinate axis parameters
axis([0 7 0 7])
axis square
hold on

% Lets create some coordinate frames
% Frame 1
T1 = SE2(1,2,30,'deg');
trplot2(T1, 'frame', '1', 'color', 'b');

% Frame 2
T2 = SE2(2.5,1, 57, 'deg');
trplot2(T2, 'frame', '2', 'color', 'r');

% Frame 3
T3 = T1 * T2;
trplot2(T3, 'frame', '3', 'color', 'g');

% Lets create a point P
P = [3 2]';

plot_point(P, '*');

% How do we determine the position of point P with respect to frame 1?

% We are using SE2, NOT se2 (like in video), SE2 gives representations of Rigid-body motion
% and the .T converts it to a homogenous transformation matrix 
% (See help SE2 command) 
P1 = inv(T1.T) * [P ; 1]

% with respect to frame 2?

P2 = inv(T2.T) * [P ; 1]

% with respect to frame 3?

P3 = inv(T3.T) * [P ; 1]
