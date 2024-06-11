%Robot-2
clear all;
startup_rvc;
load hershey;

%definitions

totaltraj= [];
scale = 0.04;
x_offset = 0;
y_offset = 0;
zPos_plane  = 98.3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%A, B

phrase = 'abcd4';
%C
phrase = '5*4=';
xPos_plane = -588.53;
yPos_plane = -133.30;
zRot_plane = 0;
numbers = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%C
stringcells = {phrase};
operators =  {'+', '-', '*'};
%DO PAD function 
for i = 1:length(operators)
    if contains(phrase, operators(i))
        phrase = phrase(1:end-1);
        numbers = strsplit(phrase, operators);
        number1  = str2double(numbers(1));
        number2 = str2double(numbers(2));
        used_operator = cell2mat(operators(i));

        if ~isnan(number1) && ~isnan(number2)
     switch used_operator
         case '+'
             result = number1 + number2;
         case '-'
             result = number1 - number2;
         case '*'
             result = number1 * number2;
     end
    stringcells = {num2str(number1),strcat(num2str(number2),used_operator),num2str(result)};
 end
    end
end
 

for j = 1:length(stringcells)
char = stringcells{j};
for i = 1:length(char)
    character = hershey{char(i)};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))]; % create the path 

% Where ever there is an nan it indicates that we need to lift up.
k = find(isnan(path(1,:)));

% At these positions add in a z hight
path(:,k) = path(:,k-1);
path(3,k) = 0.2*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
path(2,:) = path(2,:) + 0.05*scale;
path(:,end+1) = [path(1,end);path(2,end);0.2*scale];


traj = [(path'*1000)]; % convert to the mm units so that we can use the rtde toolbox
% Generate a plot of what we are expecting
%scatter3(traj(:,1), traj(:,2), traj(:,3));
%plot3(traj(:,1), traj(:,2), traj(:,3));
traj(:,1) = traj(:,1) + x_offset;
traj(:,2) = traj(:,2) - y_offset;
x_offset = x_offset + max(traj(:,1))-min(traj(:,1))+(0.2*scale*1000);


R_matrix = rot2(zRot_plane, 'deg');
traj(:,1:2) = (R_matrix * traj(:,1:2)')';
totaltraj = [totaltraj; traj];
end
if length(stringcells) > 1
y_offset = y_offset+ 40; %max(traj(:,2))-min(traj(:,2))+(0.2*0.1*1000);

    if j==1
        if(length(stringcells{1})==length(stringcells{2}))
            x_offset = max(traj(:,1))-min(traj(:,1))-(0.2*scale*1000);
        else
            x_offset = 0;
        end
    end

    if(j==2)
        if(length(stringcells{1}) > length(stringcells{3}))
             x_offset = (length(stringcells{1})- length(stringcells{3}))*(max(traj(:,1))-min(traj(:,1)));         
        elseif(length(stringcells{1}) < length(stringcells{3}))
            x_offset = (length(stringcells{1})- length(stringcells{3}))*(max(traj(:,1))-min(traj(:,1))+(0.2*scale*1000));
        else 
        x_offset = 0;
        
        end
    end
end
end

figure(1);
clf;
plot3(totaltraj(:,1), totaltraj(:,2), totaltraj(:,3));


%% NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!


% % TCP Host and Port settings
%host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
 host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
% 

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0.00];
poses = rtde.movej(home);

% Creating a path array
path = [];

% setting move parameters
v = 0.5;
a =1.2;
blend = 0.001;

% Populate the path array
for i = 1:length(totaltraj)
    %disp(i);
   % disp(totaltraj(i,1:3) + [xPos_plane, yPos_plane, 100]);
    point = [[(totaltraj(i,1:3) + [xPos_plane, yPos_plane, zPos_plane]),(home(4:6))],a,v,0,blend];
    if isempty(path)
        path = point;
    else
        path = cat(1,path,point);
    end
end

% Execute the movement!
poses = rtde.movej(path);

rtde.drawPath(poses);
%rtde.movej(home);
rtde.close;