Path = '../Data/SingleObject';


%% Convert images to clouds
SceneNum = 0;
SceneName = sprintf('%0.3d',SceneNum);

tic
for i = 0:0
    FrameNum = num2str(i);
    I = imread(fullfile(Path,['scene_',SceneName],['frames/frame_',FrameNum,'_rgb.png']));
    ID = imread(fullfile(Path,['scene_',SceneName],['frames/frame_',FrameNum,'_depth.png']));
    ID = cutOuterEdges(ID,.2);
    
    [pcX,pcY,pcZ,R,G,B] = cleanPointCloud(pcx,pcy,pcz,r,g,b);
    
    fileName = fullfile(Path,['scene_',SceneName],'cloud',['cloud_',FrameNum,'.mat']);
    save(fileName,'pcX','pcY','pcZ','R','G','B');
end
toc


%% Remove point cloud noise
SceneNum = 0;
SceneName = sprintf('%0.3d',SceneNum);
% C = [-90 25 880]; %scene 000
C = [-75 -50 850]; %scene 002
c0 = [-167 -1 958]; %scene 002

tic
for i = 0:0
    FrameNum = num2str(i);
    fileName = fullfile(Path,['scene_',SceneName],'cloud',['cloud_',FrameNum,'.mat']);
    load(fileName)
    [pcX,pcY,pcZ,R,G,B,c1] = cropPointCloud(pcX,pcY,pcZ,R,G,B,C);
    %C = C+.0001*(C-c1);
    C = C+.5*(c1-c0);
    fileName = fullfile(Path,['scene_',SceneName],'cloud2',['cloud_',FrameNum,'.mat']);
    save(fileName,'pcX','pcY','pcZ','R','G','B','C');
    c0 = c1;
end
toc


%% Display a saved cloud
FrameNum = num2str(0);
SceneNum = 0;
SceneName = sprintf('%0.3d',SceneNum);
fileName = fullfile(Path,['scene_',SceneName],'cloud2',['cloud_',FrameNum,'.mat']);
load(fileName)
Pts = [pcX pcY pcZ];
figure, pcshow(Pts,[R G B]/255);
drawnow;
title('3D Point Cloud');
xlabel('X'); ylabel('Y'); zlabel('Z');


%% 
[all_pts,all_rgb] = match_clouds();
pcshow(all_pts,all_rgb/255);
