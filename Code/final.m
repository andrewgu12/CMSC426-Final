clear all;
singleRootDirNames = ["scene_000", "scene_001", "scene_002", "scene_006", "scene_008", "scene_012", "scene_022", "scene_023"];

% for each scene - test every single image for matched features to get idea of which objects are in
sceneImage = rgb2gray(imcrop(imread('../Data/MultipleObjects/scene_025/frames/frame_0_rgb.png'),[0,0,460,630]));
scenePoints = detectSURFFeatures(sceneImage);
[sceneFeatures, scenePoints] = extractFeatures(sceneImage, scenePoints);

singleObjects = single_object_count(singleRootDirNames, sceneImage, scenePoints, sceneFeatures);




function singleObjects = single_object_count(singleRootDirNames, sceneImage, scenePoints, sceneFeatures)

    singleObjects = [];

    for imageCounter = 1:length(singleRootDirNames)
        dirPath = strcat('../Data/SingleObject/', char(singleRootDirNames(imageCounter)),'/frames/');
        if imageCounter == 2
            dirPath = strcat(dirPath, 'image_');
        else
            dirPath = strcat(dirPath, 'frame_');
        end
        rgbImage = rgb2gray(imcrop(imread(strcat(dirPath, '0_rgb.png')), [0,0,450,600]));
        singlePoints = detectSURFFeatures(rgbImage);
        [singleFeatures, singlePoints] = extractFeatures(rgbImage, singlePoints);
        pairs = matchFeatures(singleFeatures, sceneFeatures);
        matchedScenePoints = scenePoints(pairs(:,2),:);
        matchedObjectPoints = singlePoints(pairs(:,1),:);
        figure;
        showMatchedFeatures(rgbImage, sceneImage, matchedObjectPoints, matchedScenePoints, 'montage');
        if length(pairs) ~= 0
            singleObjects(end+1) = imageCounter;
        end
    end

end
