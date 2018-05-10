function [pcX,pcY,pcZ,R,G,B] = cleanPointCloud(pcx,pcy,pcz,r,g,b)
% cleanPointCloud takes a 3D point cloud pixel coordinates
% as input and returns a cleaned 3D point cloud with large flat surfaces 
% (i.e. floors and walls) removed
%             
% Inputs:
% pcx, pcy, pcz    - point cloud (valid points only, in RGB camera coordinate frame)
% r, g, b          - color of {pcx, pcy, pcz}
%             
% Outputs:
% pcX, pcY, pcZ    - point cloud (valid points only, in RGB camera coordinate frame)
% R, G, B          - color of {pcx, pcy, pcz}

    totPixels = size(pcx,1); numPixels = totPixels;
    removalRatio = .3; currentRatio = 1;
    pcX = pcx; pcY = pcy; pcZ = pcz;
    R = r; G = g; B = b;
    count = 0;

    while currentRatio > removalRatio && count < 100
        count = count+1;
        
        % get random plane using three points
        planeCoeffs = randomPlane(pcX,pcY,pcZ);
        
        % attempt to match plane to image
        threshold = .01;
        numAccepted = getAcceptance(pcX,pcY,pcZ,planeCoeffs,threshold);
        ratioAccepted = numAccepted/numPixels;

        % remove matched points if good match
        if ratioAccepted > .25
            [pcX,pcY,pcZ,R,G,B] = removePixels(pcX,pcY,pcZ,R,G,B,planeCoeffs,threshold*1.1);
        end    
        numPixels = size(pcX,1);
        currentRatio = numPixels/totPixels;
    end
end


%% Create a plane using 3 random coordinates in the point cloud
function planeCoeffs = randomPlane(pcX,pcY,pcZ)
    numPixels = size(pcX,1);
    pts = datasample(1:numPixels,3);
    samplePoints = [pcX(pts(1)) pcY(pts(1)) pcZ(pts(1));
                    pcX(pts(2)) pcY(pts(2)) pcZ(pts(2));
                    pcX(pts(3)) pcY(pts(3)) pcZ(pts(3))];
    planeCoeffs = samplePoints\[1;1;1];
end


%% Get number of pixels that match a given plane within a threshold
function numAccepted = getAcceptance(pcX,pcY,pcZ,planeCoeffs,threshold)
    numPixels = size(pcX,1);
    numAccepted = 0;
    for i = 1:numPixels
       d = dot(planeCoeffs,[pcX(i) pcY(i) pcZ(i)]);
       if abs(d-1) < threshold
          numAccepted = numAccepted+1; 
       end
    end
end


%% Removes all the points that match the given plane within a threshold
function [pcX,pcY,pcZ,R,G,B] = removePixels(pcx,pcy,pcz,r,g,b,planeCoeffs,threshold)
    numPixels = size(pcx,1);
    savedPixels = zeros(numPixels,1);
    for i = 1:numPixels
        d = dot(planeCoeffs,[pcx(i) pcy(i) pcz(i)]);
        if abs(d-1) > threshold
           savedPixels(i) = i; 
        end
    end
    savedPixels = savedPixels(savedPixels~=0);
    pcX = pcx(savedPixels); pcY = pcy(savedPixels); pcZ = pcz(savedPixels);
    R = r(savedPixels); G = g(savedPixels); B = b(savedPixels);
end




