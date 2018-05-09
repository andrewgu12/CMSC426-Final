%% CMSC 426: Project 5 Helper Code
% Written by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
% PhD in CS Student at University of Maryland, College Park
% Acknowledgements: Bhoram Lee of University of Pennsylvania for help with depthToCloud function

clc
clear all
close all

%% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 1;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/image_',FrameNum,'_depth.png']);

%% Extract 3D Point cloud
% Inputs:
% ID - the depth image
% I - the RGB image
% calib_file - calibration data path (.mat) 
%              ex) './param/calib_xtion.mat'
%              
% Outputs:
% pcx, pcy, pcz    - point cloud (valid points only, in RGB camera coordinate frame)
% r,g,b            - color of {pcx, pcy, pcz}
% D_               - registered z image (NaN for invalid pixel) 
% X,Y              - registered x and y image (NaN for invalid pixel)
% validInd	   - indices of pixels that are not NaN or zero
% NOTE:
% - pcz equals to D_(validInd)
% - pcx equals to X(validInd)
% - pcy equals to Y(validInd)

[pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
Pts = [pcx pcy pcz];

%% Display Images and 3D Points
% Note this needs the computer vision toolbox.
figure,
subplot 121
imshow(I);
title('RGB Input Image');
subplot 122
imagesc(ID);
title('Depth Input Image');

figure,
pcshow(Pts,[r g b]/255);
drawnow;
title('3D Point Cloud');
close all;

%% Crop the scene
ro = 500;
xo = 0;
yo = 0;
zo = 0;

% Initialize final size of inlier
max = [];
maxRGB = [];
maxIndex = [];
% 
pts = ((Pts(:,1)-yo).^2 + (Pts(:,2)-xo).^2 + (Pts(:,2)-zo).^2) < ro.^2;

shrunkPts = find(pts>0);
subsetPcx = pcx(shrunkPts,:);
subsetPcy = pcy(shrunkPts,:);
subsetPcz = pcz(shrunkPts,:);
subsetR = r(shrunkPts,:);
subsetG = g(shrunkPts,:);
subsetB = b(shrunkPts,:);
Pts = [subsetPcx subsetPcy subsetPcz];
RGB = [subsetR subsetG subsetB];

figure,
pcshow(Pts,RGB/255);
drawnow;
title('3D Point Cloud');

%% Remove the first plane

k = 100;
for i = 1:k        
     % Set up inlier
     index = 1;
     inliers = zeros(length(Pts),3);
     inliersInd = zeros(length(Pts),1);
     inlierRGB = zeros(length(Pts),3);
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     syms x y z;
     Pt = [x,y,z];
     %planefunction = dot(normal, Pt-Pt1); 
     
     normalUnit = normal/norm(normal);
     
     for j = 1:length(Pts)
         dist = dot(Pt1-Pts(j,:), normalUnit);
         % find the points within a certain threshold
         if abs(dist) < 7
             inliersInd(index,1) = j;
             inliers(index,1) = Pts(j,1);
             inliers(index,2) = Pts(j,2);
             inliers(index,3) = Pts(j,3);
             inlierRGB(index,1) = RGB(j,1);
             inlierRGB(index,2) = RGB(j,2);
             inlierRGB(index,3) = RGB(j,3);
             index = index+1;
         end
     end

     inliers = inliers(1:index-1,:);
     inlierRGB = RGB(1:index-1,:);
     inliersInd = inliersInd(1:index-1,:);
     
     if (length(inliers) > length(max))
         max = inliers;
         maxRGB = inlierRGB;
         maxIndex = inliersInd;
     end

end

%% Remove all the unwanted points

Pts(maxIndex,:) = [];
RGB(maxIndex,:) = [];

%% Remove the second plane

% Initialize final size of inlier
max = [];
maxRGB = [];
for i = 1:k        
     % Set up inlier
     index = 1;
     inliers = zeros(length(Pts),3);
     inliersInd = zeros(length(Pts),1);
     inlierRGB = zeros(length(Pts),3);
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     syms x y z;
     Pt = [x,y,z];
     %planefunction = dot(normal, Pt-Pt1); 
     
     normalUnit = normal/norm(normal);
     
     for j = 1:length(Pts)
         %dist2Pts = (Pt1-Pts(j,:))./norm(Pt1-Pts(j,:));
         dist = dot(Pt1-Pts(j,:), normalUnit);
         % find the points within a certain threshold
         if abs(dist) < 7
             inliersInd(index,1) = j;
             inliers(index,1) = Pts(j,1);
             inliers(index,2) = Pts(j,2);
             inliers(index,3) = Pts(j,3);
             inlierRGB(index,1) = RGB(j,1);
             inlierRGB(index,2) = RGB(j,2);
             inlierRGB(index,3) = RGB(j,3);
             index = index+1;
         end
     end

     inliers = inliers(1:index-1,:);
     inlierRGB = RGB(1:index-1,:);
     inliersInd = inliersInd(1:index-1,:);
     
     if (length(inliers) > length(max))
         max = inliers;
         maxRGB = inlierRGB;
         maxIndex = inliersInd;
     end

end

Pts(maxIndex,:) = [];
RGB(maxIndex,:) = [];

%%
figure,
pcshow(Pts,RGB/255);
drawnow;
title('3D Point Cloud');

%%
meanPt = mean(Pts);

distToMean = zeros(length(Pts),1);

for i = 1:length(Pts)
    distToMean(i,1) = norm(Pts(i,:)-meanPt);
end

Index = distToMean < 2*std(distToMean);

Pts = Pts(Index,:);
RGB = RGB(Index,:);

%%

figure,
pcshow(Pts,RGB/255);
drawnow;
title('3D Point Cloud');

%%
