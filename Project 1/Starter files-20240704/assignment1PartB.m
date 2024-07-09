% assignment1PartB.
% MTRN4230 Assignment 1 24T2
% Name: Vinoshan Parathan
% Zid:  z5358838

%This is Part 2 of Part B 
%DO I NEED TO SUBMITE 
startup_rvc;

home_joint = [0.00, -75.00, 90.00, -105.00, -90.00, 0.00];
theta = deg2rad(home_joint);
%ASK ABOUT IF WE NEED TO DO IT IN MM OR M
alpha=[pi/2 0 0 pi/2 -pi/2 0]; % in radian
d = [162.5 0 0 133.3 99.7 99.6]/1000; %
a = [0 -425 -392.2 0 0 0]/1000; 

L(1) = Link('revolute', 'd', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', 0); 
L(2) = Link('revolute', 'd', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', 0); 
L(3) = Link('revolute', 'd', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', 0); 
L(4) = Link('revolute', 'd', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', 0); 
L(5) = Link('revolute', 'd', d(5), 'a', a(5), 'alpha', alpha(5), 'offset', 0); 
L(6) = Link('revolute', 'd', d(6), 'a', a(6), 'alpha', alpha(6), 'offset', 0); 


robot = SerialLink(L,'name','UR5e')

T_matrix= robot.fkine(theta)

outputPose = [(T_matrix*[0;0;0])',tr2rpy(T_matrix)]
