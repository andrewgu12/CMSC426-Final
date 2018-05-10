%%
folderPath = '../Data/SingleObject/scene_000/frames';
fileName = 'frame_40_depth.png';
im = imread(fullfile(folderPath,fileName));
im = im2double(im);
[rows,cols] = size(im);
maxDepth = max(im(:)); minDepth = min(im(im(:)~=0));
im = (im-minDepth)/(maxDepth-minDepth);

close all
figure
imshow(im);


%%
folderPath = '../Data/SingleObject/scene_000/frames';
ratio = .25;
for i = 40:40
    fileName = ['frame_' int2str(i) '_depth.png'];
    im = cleanDepthImage(folderPath,fileName);
    im = cutOuterEdges(im,ratio);
    
    fullWritePath = fullfile(folderPath,'../cleanedDepth',fileName);
    imwrite(im,fullWritePath);
end


%% run ICP
close all
load('combinedCloud.mat')
pcshow(all_pts,all_rgb/255);


%%
Path = '../Data/SingleObject';
SceneNum = 2;
SceneName = sprintf('%0.3d',SceneNum);

%% Convert images to clouds
close all
Path = '../Data/SingleObject';
SceneNum = 0;
SceneName = sprintf('%0.3d',SceneNum);
FrameNum = num2str(i);
I = imread(fullfile(Path,['scene_',SceneName],['frames/frame_',FrameNum,'_rgb.png']));
ID = imread(fullfile(Path,['scene_',SceneName],['frames/frame_',FrameNum,'_depth.png']));
figure, imshow(I)
I = cutOuterEdges(I,.2);
ID = cutOuterEdges(ID,.2);
figure, imshow(I)



