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

%% Use Ransac to detect a plane in the scene
%i = 1;
k = 300;
ro = 500;
xo = 0;
yo = 0;
zo = 0;

%dist = sqrt(sum((center - Pts(b)) .^ 2));

% Initialize final size of inlier
max = [];

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

%%
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
         dist2Pts = (Pt1-Pts(j,:))./norm(Pt1-Pts(j,:));
         dist = dot(dist2Pts, normalUnit);
         % find the points within a certain threshold
         if abs(dist) < .07
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
     
%      % a,b,c,d
%      coefficients = double(fliplr(coeffs(planefunction,[x y z])));
%      
%      % build out and calculate entire distances matrix
%      modifiedPts = [Pts ones(size(Pts,1),1)]';
%      denominator = sqrt(coefficients(1)^2 + coefficients(2)^2 + coefficients(3)^2);
%      distancesMatrix = (coefficients * modifiedPts) ./ denominator;
     
     % find the points within a certain threshold
%      inlierIndices = find(abs(distancesMatrix) < 40);
%      inliers(:,1) = Pts(inlierIndices,1);
%      inliers(:,2) = Pts(inlierIndices,2);
%      inliers(:,3) = Pts(inlierIndices,3);
%      inlierRGB(:,1) = RGB(inlierIndices,1);
%      inlierRGB(:,2) = RGB(inlierIndices,2);
%      inlierRGB(:,3) = RGB(inlierIndices,3);

     inliers = inliers(1:index-1,:);
     inlierRGB = RGB(1:index-1,:);
     inliersInd = inliersInd(1:index-1,:);
     
     if (length(inliers) > length(max))
         max = inliers;
     end

end


%%

figure,
pcshow(inliers,inlierRGB/255);
drawnow;
title('3D Point Cloud');

%%

Pts(inliersInd,:) = [];

RGB(inliersInd,:) = [];

%%

figure,
pcshow(Pts,RGB/255);
drawnow;
title('3D Point Cloud');


%%

%meanPts = mean(Pts);
% %SEM = std(Pts)/sqrt(length(Pts));
% 
% ts = tinv([0.025  0.975],length(Pts)-1);
% 
% CI = mean(Pts) + ts'*SEM;

CI = [(mean(Pts) - std(Pts))' (mean(Pts) + std(Pts))'];

CI = CI';

t = find(Pts(:,1) > CI(1,1) & Pts(:,1) < CI(2,1) & Pts(:,2) > CI(1,2) & Pts(:,2) < CI(2,2) & Pts(:,3) < CI(1,3) & Pts(:,3) < CI(2,3));

%temp = find(Pts > CI(1,:) & Pts < CI(2,:));


Pts2 = Pts(t,:);

RGB2 = RGB(t,:);


figure,
pcshow(Pts2,RGB2/255);
drawnow;
title('3D Point Cloud');
%%
%SEM = std(x)/sqrt(length(x)); 
% Standard Error
% ts = tinv([0.025  0.975],length(x)-1);      % T-Score
% CI = mean(x) + ts*SEM;  


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