function [R, t] = rigid_transformation(p1, p2, index)
%     if (size(p1) ~= size(p2))
%         error('p1 and p2 not same size');
%     end
    index=index';
    n = size(p1, 1);
    m = size(p2, 1);
    p2 = p2(index,:);
    centroid_p1 = sum(p1)/n;
    centroid_p2 = sum(p2)/m;
    
    H = transpose(bsxfun(@minus,p1,centroid_p1))*bsxfun(@minus,p2,centroid_p2);
    [U, ~, V] = svd(H);
    
    R=V*transpose(U);
    if (det(R) < 0)
        R(:,3) = R(:,3) * -1;
    end
    
    t = -R*transpose(centroid_p1)+transpose(centroid_p2);
end