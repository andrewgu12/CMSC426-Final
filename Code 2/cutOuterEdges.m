function imCut = cutOuterEdges(im,ratio)
% this function takes in an image and outputs the same image with a
% portion of the image removed from the outer border
%
% imCut         The modified output image
% im            The original image
% ratio         The ratio the total side length to remove from either side
    
    [rows,cols] = size(im);
    imCut = zeros(rows,cols);
    for i = ceil(rows*ratio):ceil(rows*(1-ratio))
        for j = ceil(cols*ratio):ceil(cols*(1-ratio))
            imCut(i,j) = im(i,j);
        end
    end
    
    for i = 1:rows
        for j = 1:cols
            if imCut(i,j) == 0
                imCut(i,j) = NaN;
            end
        end
    end
end