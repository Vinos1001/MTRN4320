
%Transform first so that it is only the table showing then go scann all the
%cucodes.
% Part A 
clear all;

img = imread("3.jpg");
figure(3)


imshow(img)

[board_coords, table_coords] = findArucoMarkers(img);

% 1.2
%1.3
world = [-230,60;-230,-520;-990,60;-990,-520];
tform =  fitgeotrans(table_coords,world,'projective');
H_matrix = tform.T;
true_board_coords = transformPointsForward(tform, board_coords);

% 1.4
%boarders
orangemask = (img(:,:,1) <= 255)&(img(:,:,1) >=150)&...
(img(:,:,2) <=230)&(img(:,:,2) >= 145)&(img(:,:,3) <=80);
se = strel('disk',100);
orangemask = imclose(orangemask,se);

%players 
redmask = (img(:,:,1) <= 255)&(img(:,:,1) >=150)&...
(img(:,:,2) <=80)&(img(:,:,3) <=80);
redmask = maskFix(redmask);

bluemask = (img(:,:,3) <= 255)&(img(:,:,3) >=150)&...
(img(:,:,1) <=80)&(img(:,:,2) <=80);
bluemask = maskFix(bluemask);

greenmask = (img(:,:,2) <= 255)&(img(:,:,2) >=150)&...
(img(:,:,1) <=80)&(img(:,:,3) <=80);
greenmask = maskFix(greenmask);


%% Find centers of each game square 

redCentroids = getCenters(redmask);
greenCentroids = getCenters(greenmask);
blueCentroids = getCenters(bluemask);
orangeCentroids = getCenters(orangemask);
orangeCentroids = getcorners(orangeCentroids);

figure(3);

hold on;
viscircles(blueCentroids, 20, 'Color','g');
viscircles(orangeCentroids, 5, 'Color','g');

viscircles(redCentroids, 20, 'Color','k');
viscircles(greenCentroids, 20, 'Color','c');
plot(table_coords(:,1),table_coords(:,2), 'm*');
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
        if (i ~= 1 || i ~= 7 || j ~= 1 || j ~= 10)
            if (grid(i,j) == 1) 
                red_obstacle = cat(1,red_obstacle,[i j]);
                walls = cat(1,walls,[i j]);
            elseif (grid(i,j) == 2)
                blue_obstacle = cat(1,blue_obstacle,[i j]);
                walls = cat(1,walls,[i j]);
            elseif (grid(i,j) == 3)
                start_postition = cat(1,start_postition,[i j]);
                start_postition(1) =  7 - start_postition(1)+1
                grid(i,j) = 0;
            elseif (grid(i,j) == 4)
                goal_postition = cat(1,goal_postition,[i j]);
                goal_postition(1) = 7 - goal_postition(1)+1
                grid(i,j) = 0;
            end
        else
        end
    end  
end

start_postition = [start_postition(2),start_postition(1)]
goal_postition = [goal_postition(2), goal_postition(1)];
flipedGrid = flipud(grid); 
bug = myBug2(flipedGrid);
figure(2)
p = bug.query(start_postition, goal_postition, 'animate');



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
