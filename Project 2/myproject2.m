
%Transform first so that it is only the table showing then go scann all the
%cucodes.
% Part A 
close all;
clc;
clear all;
startup_rvc;
cam = webcam(1);
pause(3);
real_img = snapshot(cam);


figure(3)
subplot(2,2,1)
imshow(real_img)

[board_coords, table_coords] = findArucoMarkers(real_img);

            
model = [50,50;340,50; 50,540; 340,540];
newform = fitgeotrans(board_coords,model ,'projective');
new_img_board = transformPointsForward(newform, board_coords);

new_img = imwarp(real_img, newform,OutputView=imref2d([590 390]));
subplot(2,2,2)
imshow(new_img);
% 1.2
%1.3

% 1.4
%boarders
orangemask = (new_img(:,:,1) <= 255)&(new_img(:,:,1) >=150)&...
(new_img(:,:,2) <=230)&(new_img(:,:,2) >= 145)&(new_img(:,:,3) <=80);
se = strel('disk',100);
orangemask = imclose(orangemask,se);
%players 
redmask = (new_img(:,:,1) >=130)&(new_img(:,:,2) <=80)&(new_img(:,:,3) <=80);
redmask = maskFix(redmask);

bluemask = (new_img(:,:,3) >=120)&(new_img(:,:,1) <=80)&(new_img(:,:,2) <=80);
bluemask = maskFix(bluemask);

greenmask = (new_img(:,:,2) >=100)&(new_img(:,:,1) <=80)&(new_img(:,:,3) <=80);
greenmask = maskFix(greenmask);


%% Find centers of each game square 

redCentroids = getCenters(redmask);
greenCentroids = getCenters(greenmask);
blueCentroids = getCenters(bluemask);
orangeCentroids = getCenters(orangemask);
orangeCentroids = getcorners(board_coords);

figure(3);

hold on;
viscircles(blueCentroids, 20, 'Color','g');

viscircles(redCentroids, 20, 'Color','k');
viscircles(greenCentroids, 20, 'Color','c');
plot(new_img_board(:,1),new_img_board(:,2), 'm*');
plot(blueCentroids(:,1), blueCentroids(:,2), "g*");
plot(redCentroids(:,1), redCentroids(:,2), "k*");
plot(greenCentroids(:,1), greenCentroids(:,2), "c*");

%%
% PART B: Grid Novices (CREDIT)
grid = [ 
        1 1 1 1 1 1 1 1 1 1
        1 1 0 0 0 0 0 0 1 1
        1 0 0 0 1 0 0 0 0 1
        1 0 0 1 0 0 1 0 0 1
        1 4 2 0 0 2 2 0 3 1
        1 1 0 0 0 0 0 0 1 1
        1 1 1 1 1 1 1 1 1 1 ];
% 0 - empty
% 1 - wall or red
% 2 - blue 
% 3 - player
% 4 destination
start_postition = []; %player piece
destination = [];
red_obstacle = [];
blue_obstacle = [];
walls = [];
goal_postition = [];

for i = 1:size(grid,1)
    for j = 1:size(grid,2)
        if (i == 1 || i == 7 || j == 1 || j == 10)
        
        else
            if (grid(i,j) == 1) 
                if (i ==2 && j == 2 || i == 2 && j == 9 || i ==6 && j == 2 || i ==6 && j ==9)
                else
                red_obstacle = cat(1,red_obstacle,[i j]);
                end
                walls = cat(1,walls,[i j]);
            elseif (grid(i,j) == 2)
                blue_obstacle = cat(1,blue_obstacle,[i j]);
                walls = cat(1,walls,[i j]);
            elseif (grid(i,j) == 3)
                start_postition = cat(1,start_postition,[i j]);
                start_postition(1) =  start_postition(1);
                grid(i,j) = 0;
            elseif (grid(i,j) == 4)
                goal_postition = cat(1,goal_postition,[i j]);
                goal_postition(1) = goal_postition(1);
                grid(i,j) = 0;
            end
        
        end
    end  
end

start = [start_postition(2),7 - start_postition(1)+1]
goal = [goal_postition(2), 7 - goal_postition(1)+1];
flipedGrid = flipud(grid); 
bug = myBug2(flipedGrid);
origin = [0;0];
p = bug.query(start, goal);
figure(4)
bug.plot
hold on;
plot(p(:,1),p(:,2),'-');

path = []; 
for i = 1:size(p,1)
    row = p(i,2);
    col = p(i,1);
    path = cat(1, path, [row,col]);
end
path

%perspective change grid to image 
%player 
img_coords = [];
grid_coords = [];
if size(red_obstacle,1) >= 4
    grid_coords = red_obstacle;
    image_coords = object_order(redCentroids);
else
    grid_coords = [red_obstacle; start_postition];
    img_coords = [object_order(redCentroids); greenCentroids];
end
grid_coords
img_coords
    %this is grid to model image 
    modelform = fitgeotrans(grid_coords,img_coords,'projective');
model_path = [];
for i=1:size(path,1)
   
    imageform = transformPointsForward(modelform, path(i,:));
    model_path = cat(1,model_path, imageform);
end
model_path

%model to real image
    % real_image_playerform = fitgeotrans(new_img_board,board_coords,'projective');
 
 
 real_img_path = [];
 for i=1:size(path,1)
 
     imageform = transformPointsInverse(newform, model_path(i,:));
     real_img_path = cat(1,real_img_path, imageform);
 end

world = [-0.230,0.60;-0230,-0.520;-0.990,0.60;-0.990,-0.520];
%world = [0.60, -0.230; -0.520, -0.230;0.60, -0.990; -0.990, -0.520 ]
real_tform =  fitgeotrans(table_coords,world,'projective');
H_matrix = real_tform.T;
true_board_coords = transformPointsForward(real_tform, board_coords);

%perspective change image to robot
robot_path = [];
for i = 1:size(model_path,1)
 
    robot_trans = transformPointsForward(real_tform, model_path(i,:));
    robot_path = cat(1,robot_path, robot_trans);
end
robot_path
%%
%robot
%host = '127.0.0.1'; % THIS IP ADDRESS BE USED FOR THE VIRTUAL BOX VM 
 host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR TEH REAL ROBOT
port = 30003;
vacuumport = 63352;
rtde = rtde(host,port);
 vacuum = vacuum(host,vacuumport);
home = [-588.53, -133.30,227.00, 2.221,2.221,0];
rtde.movej(home);
maxHeight = 0.1;
minHeight = 0.5;
a = 0.5;
v = 0.5;
blend = 0.005;

%Move player 
for i = 1:size(robot_path, 1)
    path_point = [robot_path(i,1),robot_path(i,2), maxHeight, 2.221, 2.221, 0, a, v, 0, blend];
    poses = rtde.movej(path_point);

    if (i == 1)
        path_point=[robot_path(i,:), minHeight, 2.221, 2.221, 0, a, v, 0, blend];
        poses = rtde.movej(path_point);
        vacuum.grip();
        pause(5);
        path_point = [robot_path(i,:), maxHeight, 2.221, 2.221, 0, a, v, 0, blend];
        poses = rtde.movej(path_point);

    elseif (i==size(robot_path,1))
        path_point = [robot_path(i,:), minHeight, 2.221, 2.221, 0, a, v, 0, blend];
        poses = rtde.movej(path_point);
        vacuum.release()
        pause(5);
        path_point = [robot_path(i,:), maxHeight, 2.221, 2.221, 0, a, v, 0, blend];
        poses = rtde.movej(path_point);

    end
end
    home = [-588.53, -133.30,227.00, 2.221,2.221,0];
    poses = rtde.movej(home);

%%

%%
% hold on;
% plot(p(:,1),p(:,2),'-')
% prev_path = [0,0]
% counter = 1;
% while counter > 0
%     for i = 1:size(p,1)
% 
%         % conversion from p -> grid 
%         grid_row = -(p(i,2)-(7+1));
%         grid_col = p(i,1);
%     p(i,1) - 1
%     p(i,2) + 1 
%         % D1 - Move left -> right downhill
%         if((p(i,1) - 1 == prev_path(1) && p(i,2) + 1 == prev_path(2)))
% 
%             % Check if there was 2 obstacles between @ current
%             if ((grid(grid_row-1,grid_col) == 1 || grid(grid_row-1,grid_col) == 2) && (grid(grid_row,grid_col-1) == 1 || grid(grid_row,grid_col-1) == 2))
%                 % Go back 1 space and add an obstacle
%                 grid(grid_row-1,grid_col-1) = 1;
%                 fprintf("ADDED BLOCK D1\n");
%                 counter = counter + 1;
%             end
% 
%         % D2 - Move right -> left uphill
%         elseif ((p(i,1) + 1 == prev_path(1) && p(i,2) - 1 == prev_path(2)))
%             % Check if there was 2 obstacles between       
%             if ((grid(grid_row+1,grid_col) == 1 || grid(grid_row+1,grid_col) == 2) && (grid(grid_row,grid_col+1) == 1 || grid(grid_row,grid_col+1) == 2))
%                 grid(grid_row+1,grid_col+1) = 1;
%                 fprintf("ADDED BLOCK D2\n");
%                 counter = counter + 1;
%             end
% 
%         % D3 - Move left -> right uphill
%         elseif ((p(i,1) - 1 == prev_path(1) && p(i,2) - 1 == prev_path(2)))
%             % Check if there was 2 obstacles between         
%             if ((grid(grid_row+1,grid_col) == 1 || grid(grid_row+1,grid_col) == 2) && (grid(grid_row,grid_col-1) == 1 || grid(grid_row,grid_col-1) == 2))
%                 grid(grid_row+1,grid_col-1) = 1;
%                 fprintf("ADDED BLOCK D3\n");  
%                 counter = counter + 1;
% 
%             end
% 
%         % D4 - Move right -> left downhill
%         elseif ((p(i,1) + 1 == prev_path(1) && p(i,2) + 1 == prev_path(2)))
% 
%             % Check if there was 2 obstacles between         
%             if ((grid(grid_row-1,grid_col) == 1 || grid(grid_row-1,grid_col) == 2) && (grid(grid_row,grid_col+1) == 1 || grid(grid_row,grid_col+1) == 2))
%                 grid(grid_row-1,grid_col+1) = 1;
%                 fprintf("ADDED BLOCK D4\n");   
%                 counter = counter + 1;
% 
%             end
%         end     
%         prev_path = p(i,:);   
%     end 
%     % Ensure there is no diagonal crossing between obstacles
%     counter = counter - 1;
%     flipedGrid = flipud(grid); 
% bug = myBug2(flipedGrid);
% figure(2)
% p = bug.query(start_postition, goal_postition, 'animate');
% hold on;
% plot(p(:,1),p(:,2),'-')
% 
% end

%%
% helper functions 
%%
function [board_coords, table_coords] = findArucoMarkers(img)
    [ids,locs,detectedFamily] = readArucoMarker(img, "DICT_4X4_250");
    table_coords = [];
    board_coords = [];
    numMarkers = length(ids);
    for i = 1:numMarkers
        loc = locs(:, :, find(ids ==i));
        center = mean(loc);
        if i >= 5
            board_coords = [board_coords; center];
        elseif i < 5
            table_coords = [table_coords; center];
        end
    end
end
%%
function newMask = maskFix(mask)
se = strel('square', 5);
mask = imclose(mask, se);

newMask = bwareaopen(mask, 100);
end
%%
function centroids = getCenters(mask)
props = regionprops(mask,"Centroid");
centroids = cat(1,props.Centroid);
end
%%
function corners = getcorners(centers)

 new_centers = [];    
 counter = 0;
   
    for i = 1:length(centers)
        for j = 1:length(centers)
            if (centers(j,1) < 476 && centers(j,2) < 201 && counter == 0)
                centers(j,:) = [centers(j,1)-15, centers(j,2)-15];
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;

            elseif(centers(j,1) > 476 && centers(j,2) < 201 && counter == 1) 
                centers(j,:) = [centers(j,1)+15, centers(j,2)-15];
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;   
            elseif(centers(j,1) < 476 && centers(j,2) > 201 && counter == 2) 
                centers(j,:) = [centers(j,1)-15, centers(j,2)+15];
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;   
            elseif(centers(j,1) > 400 && centers(j,2) > 201 && counter == 3) 
                centers(j,:) = [centers(j,1)+15, centers(j,2)+15];
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;  
            end
        end 
    end
        corners = new_centers;

end
function corners = fixgetcorners(centers)
    new_centers = [];    
    counter = 0;
    centers
    for i = 1:length(centers)
        for j = 1:length(centers)

            if (centers(j,1) < 476 && centers(j,2) < 201 && counter == 0)
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;
            elseif(centers(j,1) > 476 && centers(j,2) < 201 && counter == 1) 
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;   
            elseif(centers(j,1) < 476 && centers(j,2) > 201 && counter == 2) 
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;   
            elseif(centers(j,1) > 410 && centers(j,2) > 201 && counter == 3) 
                new_centers = cat(1,new_centers,centers(j,:));
                counter = counter + 1;   
            end
        end 
    
    end 
    corners = new_centers;
end
%%
function organized_points = object_order(centers)
   
organized_points = sortrows(centers, [1,2]);
% error = 10;
    % [~, idx] = sort(centers(:,1), 'descend');
    % %sorted_centers = centers(idx,:);
    % 
    % organized_points = [];
    % i = 1;
    % while i <= size(sorted_centers, 1)
    %     group = sorted_centers(i,:);
    %     j = i + 1;
    %     while j <= size(sorted_centers, 1) && abs(sorted_centers(j,1) - group(1,1)) <= error
    %         group = [group; sorted_centers(j,:)];
    %         j = j + 1;
    %     end
    %     group = sortrows(group, 2);
    %     organized_points = [organized_points; group];
    %     i = j;
    % end
end
%%