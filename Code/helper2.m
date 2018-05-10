clc; clear all; close all;
fileDirPath = '../Data/SingleObject/scene_001/frames/';
fileDir = dir(strcat(fileDirPath, '*.png'));
numberOfEntries = size(fileDir,1) / 2;

for imageFileCounter = 1:5:numberOfEntries
    I = imread(strcat(fileDirPath, 'image_', int2str(imageFileCounter),'_rgb.png'));
    ID = imread(strcat(fileDirPath, 'image_', int2str(imageFileCounter), '_depth.png'));

    [pcx, pcy, pcz, r, g, b, D_, X, Y,validInd] = depthToCloud_full_RGB(ID, I, './params/calib_xtion.mat');
    Pts = [pcx pcy pcz];

    %% Crop the scene
    ro = 500;
    xo = 0;
    yo = 0;
    zo = 0;

    % Initialize final size of inlier
    maxIndex = [];
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

%     figure,
%     pcshow(Pts,RGB/255);
%     drawnow;
%     title('3D Point Cloud');

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

    meanPt = mean(Pts);
    distToMean = zeros(length(Pts),1);
    for i = 1:length(Pts)
        distToMean(i,1) = norm(Pts(i,:)-meanPt);
    end
    Index = distToMean < 2*std(distToMean);

    Pts = Pts(Index,:);
    RGB = RGB(Index,:);
    if (imageFileCounter ~= 1)
        s = icp(Pts, s, 100);
        s = unique(s, 'row');
    else
        s = Pts;
    end
    %%
    figure;
    pcshow(s);
    drawnow;
    title('3D Point Cloud');
end
