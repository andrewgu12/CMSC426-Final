function [all_pts,all_rgb] = match_clouds()
    SceneNum = 0;
    SceneName = sprintf('%0.3d',SceneNum);
    Path = '../Data/SingleObject';
    
    fileName = fullfile(Path,['scene_',SceneName],'cloud',['cloud_','0','.mat']);
    load(fileName);
    
    all_rgb = [R G B]; all_pts = [pcX pcY pcZ]; prev_pts = all_pts;
    
    inc = 10;
    numFrames = 35;
    % rotations and translations hold the respective transformative
    % matrices from frame_k to frame_k-1
    rotations = cell(numFrames,1); translations = cell(numFrames,1);
    for i = inc:inc:inc*numFrames
        FrameNum = num2str(i);
        fileName = fullfile(Path,['scene_',SceneName],'cloud2',['cloud_',FrameNum,'.mat']);
        load(fileName);
        
        pts = [pcX pcY pcZ];
        all_rgb = vertcat(all_rgb, [R G B]);
        [rotations{i/inc},translations{i/inc}] = icp(prev_pts',pts');
        prev_pts = pts;
    end
    
    rotation = eye(3); translation = zeros(3,1);
    for i = inc:inc:inc*numFrames
        FrameNum = num2str(i);
        fileName = fullfile(Path,['scene_',SceneName],'cloud2',['cloud_',FrameNum,'.mat']);
        load(fileName);
        pts = [pcX pcY pcZ];
        numPts = size(pts,1);
        
        translation = translation+rotation*translations{i/inc};
        rotation = rotation*rotations{i/inc};
        pts = (rotation*pts'+repmat(translation,1,numPts))';
        
        all_pts = vertcat(all_pts,pts);
    end
    
    pcX = all_pts(1,:); pcY = all_pts(2,:); pcZ = all_pts(3,:);
    R = all_rgb(1,:); G = all_rgb(2,:); B = all_rgb(3,:);
    fileName = fullfile(Path,['scene_',SceneName],'cloud2',['combinedCloud','.mat']);
    save(fileName,'pcX','pcY','pcZ','R','G','B');
    
    % cleaning function
%     numPts =  
%     for i = 
%         
%     end
end