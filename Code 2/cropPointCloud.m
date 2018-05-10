function [pcX,pcY,pcZ,R,G,B,C] = cropPointCloud(pcx,pcy,pcz,r,g,b,c)
% cleanPointCloud takes a 3D point cloud pixel coordinates and estimated
% centroid as input and returns a 3D point cloud with only the pixels
% within a 
% (i.e. floors and walls) removed
%             
% Inputs:
% pcx, pcy, pcz     - point cloud (valid points only, in RGB camera coordinate frame)
% r, g, b           - color of {pcx, pcy, pcz}
% c                 - estimated centroid coordinate of object 
%             
% Outputs:
% pcX, pcY, pcZ     - point cloud (valid points only, in RGB camera coordinate frame)
% R, G, B           - color of {pcx, pcy, pcz}
% C                 - calculated

    objR = 150; % object "radius" 
    totPixels = size(pcx,1);
    validIndices = zeros(totPixels,1);
    dx = zeros(totPixels,1); dy = zeros(totPixels,1); dz = zeros(totPixels,1);
    
    % save all indices that are within bounds
    for i = 1:totPixels
        dx(i) = abs(pcx(i)-c(1));
        dy(i) = abs(pcy(i)-c(2));
        dz(i) = abs(pcz(i)-c(3));
        if dx(i) < objR && dy(i) < objR && dz(i) < objR
           validIndices(i) = i;
        end
    end
    
    % recognize all valid points
    validIndices = validIndices(validIndices~=0);
    pcX = pcx(validIndices); pcY = pcy(validIndices); pcZ = pcz(validIndices);
    R = r(validIndices); G = g(validIndices); B = b(validIndices);
    
    % calculate centroid based on valid points
    totPixels = size(validIndices,1);
    sumX = 0; sumY = 0; sumZ = 0;
    for i = 1:totPixels
        sumX = sumX+pcX(1);
        sumY = sumY+pcY(1);
        sumZ = sumZ+pcZ(1); 
    end
    C = [sumX/totPixels sumY/totPixels sumZ/totPixels];
end