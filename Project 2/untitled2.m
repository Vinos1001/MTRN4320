clear all;

I = imread("3.jpg"); % Does not like PNGs
imshow(I)

transformImg = I;

%% Get and display AruCo Marker location
[ids,locs,detectedFamily] = readArucoMarker(I, "DICT_4X4_250");

numMarkers = length(ids);
all_centers = [];
for i = 1:numMarkers
    loc = locs(:,:,i);
   
    % Display the marker ID and family
    disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))  
 
    % Insert marker edges
    I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
 
    % Insert marker corners
    markerRadius = 6;
    numCorners = size(loc,1);
    markerPosition = [loc,repmat(markerRadius,numCorners,1)];
    I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
   
    % Insert marker IDs
    center = mean(loc);
    all_centers = [all_centers; center];
    I = insertText(I,center,ids(i),FontSize=30,BoxOpacity=1);
end

figure(2);
imshow(I)

%% Check the number of points
if size(all_centers, 1) ~= size(world, 1)
    error('The number of detected marker centers does not match the number of world points');
end

%% Get points of AruCo board markers in image
% Find aruco markers in image with an id of 5
topLeft_Ind = find(ids == 5);

% Find the centre of the marker
topLeft_pt = mean(locs(:, :, topLeft_Ind));

% Define your X and Y points from the image here
% all_centers already contains the image points

%% Do transforms
% Find the transformation matrix
tform = fitgeotrans(all_centers, world, 'projective');

% Create the transformed image (birds-eye view of the board)
gameview = imwarp(I, tform, 'OutputView', imref2d(size(I)));

figure(3);
imshow(gameview)