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
% r = 150;
% xo = 320;
% yo = 240;
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


%% Use Ransac to detect a plane in the scene
%i = 1;
k = 10;

% Initialize final size of inlier
max = 0;

%while i < k
for i = 1:k

     % Set up inlier index
     index = 1;
     
     % Pick three points
     sample_pts = randperm(length(Pts),3);
     
     Pt1 = Pts(sample_pts(1,1),:);
     Pt2 = Pts(sample_pts(1,2),:);
     Pt3 = Pts(sample_pts(1,3),:);

     normal = cross(Pt1-Pt2, Pt1-Pt3);
     syms x y z
     Pt = [x,y,z];
     planefunction = dot(normal, Pt-Pt1); 
     
     for j = 1:length(Pts)
         
         x_curr = Pts(j,1);
         y_curr = Pts(j,2);
         z_curr = Pts(j,3);
         err = subs(planefunction, [x,y,z],[x_curr, y_curr, z_curr]);
         
         if err < .01
             inliers(index,1) = Pts(j,1);
             inliers(index,2) = Pts(j,2);
             inliers(index,3) = Pts(j,3);
             index = index + 1;
         end
         
     end
     
     if (length(inliers) > max)
         max = length(inliers);
     end
     
     Inliers = [];
     %i = i+1;
end
 

%%

figure,
pcshow(inliers,[r g b]/255);
drawnow;
title('3D Point Cloud');
     