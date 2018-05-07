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

%%

%  
% imageSize = size(I);
% [xx,yy] = ndgrid((1:imageSize(1))-yo,(1:imageSize(2))-xo);
% mask = uint8((xx.^2 + yy.^2) < r^2);
% cropped = uint8(zeros(size(I)));
% cropped(:,:,1) = I(:,:,1).*mask;
% cropped(:,:,2) = I(:,:,2).*mask;
% cropped(:,:,3) = I(:,:,3).*mask;
% % I = cropped(yo-r:yo+r,xo-r:xo+r);
% 
% imageSize = size(ID);
% [xx,yy] = ndgrid((1:imageSize(1))-yo,(1:imageSize(2))-xo);
% mask = uint16((xx.^2 + yy.^2) < r^2);
% cropped = ID.*mask;
% % ID = cropped(yo-r:yo+r,xo-r:xo+r);
% 
% imshow(I);

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

%% Use Ransac to detect a plane in the scene
%i = 1;
k = 250;
ro = 150;
xo = 320;
yo = 240;

% Initialize final size of inlier
max = [];
RGB = [r g b];
% 
% pts = ((Pts(:,1)-yo).^2 + (Pts(:,2)-xo).^2) < r.^2;
% 
% shrunkPts = find(pts>0);
% subsetPcx = pcx(shrunkPts,:);
% subsetPcy = pcy(shrunkPts,:);
% subsetPcz = pcz(shrunkPts,:);
% subsetR = r(shrunkPts,:);
% subsetG = g(shrunkPts,:);
% subsetB = b(shrunkPts,:);
% Pts = [subsetPcx subsetPcy subsetPcz];
% RGB = [subsetR subsetG subsetB];

for i = 1:k        
     % Set up inlier
     inliers = [];
     inlierRGB = [];
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     syms x y z;
     Pt = [x,y,z];
     planefunction = dot(normal, Pt-Pt1); 
     % a,b,c,d
     coefficients = double(fliplr(coeffs(planefunction,[x y z])));
     
     % build out and calculate entire distances matrix
     modifiedPts = [Pts ones(size(Pts,1),1)]';
     denominator = sqrt(coefficients(1)^2 + coefficients(2)^2 + coefficients(3)^2);
     distancesMatrix = (coefficients * modifiedPts) ./ denominator;
     
     % find the points within a certain threshold
     inlierIndices = find(abs(distancesMatrix) < 40);
     inliers(:,1) = Pts(inlierIndices,1);
     inliers(:,2) = Pts(inlierIndices,2);
     inliers(:,3) = Pts(inlierIndices,3);
     inlierRGB(:,1) = RGB(inlierIndices,1);
     inlierRGB(:,2) = RGB(inlierIndices,2);
     inlierRGB(:,3) = RGB(inlierIndices,3);
     
     if (length(inliers) > length(max))
         max = inliers;
     end
     %i = i+1;
end


%%

figure,
pcshow(inliers,inlierRGB/255);
drawnow;
title('3D Point Cloud');
     