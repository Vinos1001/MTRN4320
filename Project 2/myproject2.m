%Ask if it has to continuously repeat or does it just go and ask me to go again and ask again to go again.

%GRID AND BUG2
close all;
clc;
clear all;
startup_rvc;
%cam = webcam(1);
        %pause(3);
        %real_img = snapshot(cam);
        real_img =imread("3.jpg");
        [new_img_board,newform,table_coords, board_coords, blueCentroids] = understandImage(real_img);


starter_grid = [ 
        1 1 1 1 1 1 1 1 1 1
        1 1 0 0 0 0 0 0 1 1
        1 0 0 0 1 0 0 0 0 1
        1 0 0 1 0 0 0 0 0 1
        1 4 2 0 0 2 2 0 3 1
        1 1 0 0 0 0 0 0 1 1
        1 1 1 1 1 1 1 1 1 1 ];
grid = scaleGrid(starter_grid);
% 0 - empty
% 1 - wall or red
% 2 - blue 
% 3 - player
% 4 destination

[start_postition,grid,blue_obstacle, walls, goal_postition]= analyseGrid(grid);
grid_blue_obstacle = idk(starter_grid);
[path] = bug2stuff(start_postition, goal_postition, grid);

%GETTING IMAGE AND STUFF AND ASKING ABOUT BLUE

    keepAsking = true;
while keepAsking
    resp = input('continue yes/no:', 's');
    
    if strcmpi(resp, 'yes')
        keepAsking = false;
    elseif strcmpi(resp, 'no')
        disp('Repeating the loop...');
        close all;
        clc;
            keepAsking = true;
        %cam = webcam(1);
        %pause(3);
        %real_img = snapshot(cam);
        real_img =imread("3.jpg");
        [new_img_board,newform,table_coords, board_coords, blueCentroids] = understandImage(real_img);
        [blue_current,blue_end] = checkblue(grid_blue_obstacle, new_img_board, blueCentroids, newform);
        

    else
        disp('Invalid response- "yeah" or "no".');
    end
end
disp('You chose to move on.');



%all the transformations in the world 
robot_path = fullPerspectiveTransform(path,new_img_board, newform, table_coords, board_coords)



%%
%robot
host = '127.0.0.1'; % THIS IP ADDRESS BE USED FOR THE VIRTUAL BOX VM 
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR TEH REAL ROBOT
%startup_rvc;
port = 30003;
vacuumport = 63352;
%rtde = rtde(host,port);
vacuum = vacuum(host,vacuumport);
home = [-588.53, -133.30,227.00, 2.221,2.221,0];
rtde.movej(home);
maxHeight = 100;
minHeight = 6;
a = 1.2;
v = 0.5;
blend = 0.001;
%Move player 
path_point = [];
for i = 1:size(robot_path, 1)
    robot_path(i,:)
    path_point = [robot_path(i,1),robot_path(i,2), maxHeight, 2.221, 2.221, 0]
    poses = rtde.movej(path_point);

    if (i == 1)
        path_point = [robot_path(i,:), minHeight, 2.221, 2.221, 0];
        poses = rtde.movej(path_point);
        vacuum.grip();
        pause(5);
        path_point = [robot_path(i,:), maxHeight, 2.221, 2.221, 0];
        poses = rtde.movej(path_point);

    elseif (i==size(robot_path,1))
        path_point = [robot_path(i,:), minHeight, 2.221, 2.221, 0];
        poses = rtde.movej(path_point);
        vacuum.release()
        pause(5);
        path_point = [robot_path(i,:), maxHeight, 2.221, 2.221, 0];
        poses = rtde.movej(path_point);

    end
end
    home = [-588.53, -133.30,227.00, 2.221,2.221,0];
    poses = rtde.movej(home);
    rtde.close;



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
   error = 20;
rounded_x = round(centers(:,1) / error) * error;
rounded_y = round(centers(:,2) / error) * error;
new_centers = [rounded_x, rounded_y];
organized_points = sortrows(new_centers, [1,2]);
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
function new_grid = scaleGrid(grid)
scaleFactor = 10;
[rows, cols] = size(grid);
new_grid = zeros(rows * scaleFactor, cols * scaleFactor);

    for i = 1:rows
        for j = 1:cols
        new_grid((i-1)*scaleFactor + (1:scaleFactor), (j-1)*scaleFactor + (1:scaleFactor)) = grid(i,j);
        end
    end
end 
%%
function [start_postition,grid,blue_obstacle, walls, goal_postition] = analyseGrid(grid)
    start_postition = []; %player piece
    red_obstacle = [];
    blue_obstacle = [];
    walls = [];
    goal_postition = [];

    for i = 1:size(grid,1)
        for j = 1:size(grid,2)
            if (i <= 10 || i >= 60 || j <= 10 || j >= 100)
        
            else
                if (grid(i,j) == 1) 
                    if (i <= 20 && j <= 20 || i <= 20 && j >= 90 || i >=60 && j <= 20 || i <=60 && j >=90)
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

    for k = 1:size(walls,1)-1
    i = walls(k,1);
    j = walls(k,2);
    
    grid(i+1,j+1) = 1;
    grid(i+1,j) = 1;
    grid(i,j+1) = 1;
    
    end
end
%% 
function [new_img_board,newform, table_coords, board_coords, blueCentroids]=understandImage(real_img)
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


% Find centers of each game square 

    redCentroids = getCenters(redmask);
    greenCentroids = getCenters(greenmask);
    blueCentroids = getCenters(bluemask);

    figure(3);

    hold on;
    viscircles(blueCentroids, 20, 'Color','g');

    viscircles(redCentroids, 20, 'Color','k');
    viscircles(greenCentroids, 20, 'Color','c');
    plot(new_img_board(:,1),new_img_board(:,2), 'm*');
    plot(blueCentroids(:,1), blueCentroids(:,2), "g*");
    plot(redCentroids(:,1), redCentroids(:,2), "k*");
    plot(greenCentroids(:,1), greenCentroids(:,2), "c*");
end

%%
function [path]=bug2stuff(start_postition, goal_postition, grid)
start = [round(mean(start_postition(:,2))),70 - round(mean(start_postition(:,1)))];
goal = [round(mean(goal_postition(:,2))), 70 - round(mean(goal_postition(:,1)))];
flipedGrid = flipud(grid); 
bug = myBug2(flipedGrid);

origin = [0;0];
p = bug.query(start, goal);
figure(4);
bug.plot;
hold on;

path = []; 
% for i = 1:size(p,1)
%     row = -p(i,2)+7+1;
%     col = p(i,1);
%     path = cat(1, path, [row,col]);
% end
for k = 1:size(p,1)-1
    x1 = p(k,1);
    y1 = p(k, 2);
    x2 = p(k+1,1);
    y2 = p(k+1,2);

    center_x1 = (ceil(x1/10)-1) * 10 + 10/2+1;
    center_y1 = (ceil(y1/10)-1) * 10 + 10/2-1;
    center_x2 = (ceil(x2/10)-1) * 10 + 10/2+1;
    center_y2 = (ceil(y2/10)-1) * 10 + 10/2-1;



    if isempty(path) || ~isequal(path(end,:), [center_x1, center_y1])
        path = [path; center_x1,center_y1];
    end
    if center_x1 ~= center_x2 && center_y1 ~= center_y2
        % Determine the correct order of adding intermediate points based on direction
        if x2 > x1
             path = [path; center_x1+10, center_y1];
        elseif x2 < x1
            path = [path; center_x1-10, center_y1]; 
        end
    end
end
plot(path(:,1),path(:,2),'g-');

end
%%
function [robot_path] = fullPerspectiveTransform(path, new_img_board, newform, table_coords, board_coords)
grid_coords = [86 54; 86 14; 16 54; 16, 14];
img_coords = new_img_board;
    %this is grid to model image 
    modelform = fitgeotrans(grid_coords,img_coords,'projective');
model_path = [];
for i=1:size(path,1)
   
    imageform = transformPointsForward(modelform, path(i,:));
    model_path = cat(1,model_path, imageform);
end


%model to real image
    % real_image_playerform = fitgeotrans(new_img_board,board_coords,'projective');
 
 
 real_img_path = [];
 for i=1:size(path,1)
 
     imageform = transformPointsInverse(newform, model_path(i,:));
     real_img_path = cat(1,real_img_path, imageform);
 end

world = [-230,60;-230,-520;-990,60;-990,-520];
%world = [0.60, -0.230; -0.520, -0.230;0.60, -0.990; -0.990, -0.520 ]
real_tform =  fitgeotrans(table_coords,world,'projective');
H_matrix = real_tform.T;
true_board_coords = transformPointsForward(real_tform, board_coords);

%perspective change image to robot
robot_path = [];
for i = 1:size(model_path,1)
 
    robot_trans = transformPointsForward(real_tform, real_img_path(i,:));
    robot_path = cat(1,robot_path, robot_trans);
end
end
%%
function blue = idk(sgrid)

%for i = 1:(size(blue_obstacle,1))/100
 %       start_object = (i - 1) * 100 + 1;
  %      end_object = i*100;
   %    object_points =  blue_obstacle(start_object:end_object, :);
    %   mean_pos = mean(object_points, 1);
     %  mean_positions(i, :) = round(mean_pos);

%end
%mean_positions = [mean_positions(:,2), mean_positions(:,1)];
blue = [];
for i = 1:size(sgrid,1)
    for j = 1: size(sgrid,2)
        if (sgrid(i,j)==2)
            blue = cat(1, blue,[i*10+6-10,j*10+6-10]);
            
        end
    end
end

end

function [real_current_pos,real_end_pos] = checkblue(grid_blue_obstacle, new_img_board, blueCentroids, newform)
    %for i= 1:size(grid_blue_obstacle, 1)
      
    grid_blue_obstacle =  [grid_blue_obstacle(:,2),70 - grid_blue_obstacle(:,1)];
    %end
    grid_coords = [86 54; 86 14; 16 54; 16, 14];
    img_coords = new_img_board;
    %this model image to grid
    modelform = fitgeotrans(grid_coords,img_coords,'projective');
    correct_places = [];
    for i=1:size(grid_blue_obstacle,1)
   
        imageform = transformPointsForward(modelform, grid_blue_obstacle(i,:));
        correct_places = cat(1,correct_places, imageform); 
    end

    img_index = [];
    grid_index = [];
    incorrect_now_empty = [];
    counterA = 0;
    for i = 1:size(correct_places,1)
        for j = 1:size(correct_places,1)
            if (abs(correct_places(i,:)-blueCentroids(j,:)) <=10)
                img_index = cat(1, img_index, j);
                counterA = counterA+1;
            end
        end
        if(counterA ==0)
            fprintf("INCORRECT PLACEMENT");
            grid_index = cat(1,grid_index,i);
            incorrect_now_empty = cat(1, incorrect_now_empty, correct_places(i,:));
        end
        counterA = 0;
    end

    counterB = 0;
    current = [];
    for i = 1:size(grid_blue_obstacle, 1)
        for j = 1:size(img_index)
            if i == img_index(j)
                counterB = counterB+1;
            end
        end
        if(counterB ==1)
            fprintf("%d is correct", i);
        else 
            fprintf('%d needs to be moved',i);
            current = cat(current,blueCentroids(i,:));
        end
        counterB = 0;
    end
%%
%current %this is the incorrect 

%% 
    real_current_pos = [];
    real_end_pos = [];
    if isempty(current)
        real_current_pos = [0 0];
        real_end_pos = [0 0];
    else
        img_current_pos = [];
        for i=1:size(path,1)
 
            imageform = transformPointsInverse(newform, current(i,:));
            img_current_pos = cat(1,img_current_pos, imageform);
        end

        world = [-230,60;-230,-520;-990,60;-990,-520];
        %world = [0.60, -0.230; -0.520, -0.230;0.60, -0.990; -0.990, -0.520 ]
        real_tform =  fitgeotrans(table_coords,world,'projective');

        for i = 1:size(model_path,1)
 
            robot_trans = transformPointsForward(real_tform, img_current_pos(i,:));
            real_current_pos = cat(1,real_current_pos, robot_trans);
        end
    
   %%
%incorrect_now_empty %this is the correct one
        img_end_pos = [];
        for i=1:size(path,1)
 
            imageform = transformPointsInverse(newform, incorrect_now_empty(i,:));
            img_end_pos = cat(1,img_end_pos, imageform);
        end

        world = [-230,60;-230,-520;-990,60;-990,-520];
        %world = [0.60, -0.230; -0.520, -0.230;0.60, -0.990; -0.990, -0.520 ]
        real_tform =  fitgeotrans(table_coords,world,'projective');


        for i = 1:size(model_path,1)
 
            robot_trans = transformPointsForward(real_tform, img_end_pos(i,:));
            real_end_pos = cat(1,real_end_pos, robot_trans);
        end
    
    end
end