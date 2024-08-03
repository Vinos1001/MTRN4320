% Part A 
clear all;
img = imread("3.jpg");
[ids,locs,detectedFamily] = readArucoMarker(img, "DICT_4X4_250");
table_coords = [];
board_coords = [];
for i = 1:8
    center = mean(locs(:, :, find(ids ==i)));
    if i >= 5
        board_coords = [board_coords; center];
    elseif i < 5
        table_coords = [table_coords; center];
    end

end
% topLeft_Ind = find(ids ==1);
% topLeft_pt = mean(locs(:, :, topLeft_Ind));
% 
% topRight_Ind = find(ids ==2);
% topRight_pt = mean(locs(:, :, topRight_Ind));
% 
% bottomLeft_Ind = find(ids ==3);
% bottomLeft_pt = mean(locs(:, :, bottomLeft_Ind));
% 
% bottomRight_Ind = find(ids ==4);
% bottomRight_pt = mean(locs(:, :, bottomRight_Ind));
% 
% board_coordinates = [topLeft_pt;topRight_pt;bottomLeft_pt;bottomRight_pt];


% 1.1
% view = [50, 50; 340 , 50; 50, 540; 340, 540];
% viewform = fitgeotrans(board_coords,view,'projective');
% gameview = imwarp(img,viewform,OutputView=imref2d([590, 390]));
figure(3);
imshow(img)

% 1.2
%1.3
world = [-230,60;-230,-520;-990,60;-990,-520];
tform =  fitgeotrans(table_coords,world,'projective');
H_matrix = tform.T;
board_coords_robot = transformPointsForward(tform, board_coords);

% 1.4
%red 
redmask = (img(:,:,1) <= 255)&(img(:,:,1) >=150)&...
(img(:,:,2) <=80)&(img(:,:,3) <=80);



bluemask = (img(:,:,3) <= 255)&(img(:,:,3) >=150)&...
(img(:,:,1) <=80)&(img(:,:,2) <=80);


greenmask = (img(:,:,2) <= 255)&(img(:,:,2) >=150)&...
(img(:,:,1) <=80)&(img(:,:,3) <=80);

%% Find centers of each game square 
blueProps = regionprops(bluemask, "Centroid");
blueCentroids = cat(1,blueProps.Centroid);

redProps = regionprops(redmask, "Centroid");
redCentroids = cat(1,redProps.Centroid);

greenProps = regionprops(greenmask, "Centroid");
greenCentroids = cat(1,greenProps.Centroid);

figure(3);
hold on;
viscircles(blueCentroids, 20, 'Color','g')
viscircles(redCentroids, 20, 'Color','k')
viscircles(greenCentroids, 20, 'Color','c')
plot(blueCentroids(:,1), blueCentroids(:,2), "g*")
plot(redCentroids(:,1), redCentroids(:,2), "k*")
plot(greenCentroids(:,1), greenCentroids(:,2), "c*")


