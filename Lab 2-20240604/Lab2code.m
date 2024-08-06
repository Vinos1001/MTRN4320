%Lab2 code 
clear all;
cam = webcam(2);
pause(5);
img1 = snapshot(cam);
I = snapshot(cam);
%img = imread("3.jpg");
figure(3)


imshow(I)
;
[ids,locs,detectedFamily] = readArucoMarker(I)
numMarkers = length(ids);
for i = 1:numMarkers
  loc = locs(:,:,i);
   
  % Display the marker ID and family
 % disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i)); 
 
  % Insert marker edges
  I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
 
  % Insert marker corners
  markerRadius = 6;
  numCorners = size(loc,1);
  markerPosition = [loc,repmat(markerRadius,numCorners,1)];
  markercenter = [(markerPosition(1,1)+markerPosition(3,1))/2, (markerPosition(1,2)+markerPosition(3,2))/2, 6];
  
  I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
   
  % Insert marker IDs
  center = mean(loc);
  I = insertText(I,center,ids(i),FontSize=30,BoxOpacity=1);

  I = insertShape(I, "filled-circle", markercenter, ShapeColor="cyan", Opacity=1);
end

tform = fitgeotrans(locs(:,:,8), locs(:,:,4),'projective');

figure(1);
imshow(I)