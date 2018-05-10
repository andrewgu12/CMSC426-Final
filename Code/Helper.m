%% CMSC 426: Project 5 Helper Code
% Written by: Nitin J. Sanket (nitinsan@terpmail.umd.edu)
% PhD in CS Student at University of Maryland, College Park
% Acknowledgements: Bhoram Lee of University of Pennsylvania for help with depthToCloud function

clc
%clear all
close all

%% Setup Paths and Read RGB and Depth Images
Path = '../Data/SingleObject/'; 
SceneNum = 0;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(1);

I = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_depth.png']);

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
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     
     normalUnit = normal/norm(normal);     
     distancesMatrix = (Pt1 - Pts) * normalUnit';
     indices = find(abs(distancesMatrix) < 7);
     
     if (length(indices) > length(maxIndex))
         maxIndex = indices;
     end
end

%% Remove all the unwanted points

Pts(maxIndex,:) = [];
RGB(maxIndex,:) = [];

%% Remove the second plane

maxIndex = [];
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
     
     normalUnit = normal/norm(normal);     
     distanceMatrix = (Pt1-Pts) * normalUnit';
     indices = find(abs(distanceMatrix) < 7);
     
     if (length(indices) > length(maxIndex))
         maxIndex = indices;
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

Pts1 = Pts;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Second Imag %%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Setup Paths and Read RGB and Depth Images

Path = '../Data/SingleObject/'; 
SceneNum = 0;
SceneName = sprintf('%0.3d', SceneNum);
FrameNum = num2str(10);

I = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_rgb.png']);
ID = imread([Path,'scene_',SceneName,'/frames/frame_',FrameNum,'_depth.png']);

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
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     
     normalUnit = normal/norm(normal);     
     distancesMatrix = (Pt1 - Pts) * normalUnit';
     indices = find(abs(distancesMatrix) < 7);
     
     if (length(indices) > length(maxIndex))
         maxIndex = indices;
     end
end

%% Remove all the unwanted points

Pts(maxIndex,:) = [];
RGB(maxIndex,:) = [];

%% Remove the second plane

maxIndex = [];
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
     
     normalUnit = normal/norm(normal);     
     distanceMatrix = (Pt1-Pts) * normalUnit';
     indices = find(abs(distanceMatrix) < 7);
     
     if (length(indices) > length(maxIndex))
         maxIndex = indices;
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

Pts10 = Pts;

%%
s = icp(Pts10, Pts1, 100);

%%
figure;
pcshow(s);
drawnow;
title('3D Point Cloud');
