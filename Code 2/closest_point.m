function [index, dist]=closest_point(p1, p2)
%square dist of x,y,z
%     p1=p1';
%     p2=p2';
%     m = size(p2,2);
%     n = size(p1,2);    
%     index = zeros(1,m);
%     dist = zeros(1,m);
%     for ki=1:m
%         d=zeros(1,n);
%         for ti=1:3
%             d=d+(p1(ti,:)-p2(ti,ki)).^2;
%         end
%         [dist(ki),index(ki)]=min(d);
%     end
%     dist = sqrt(dist);
    [index, dist] = knnsearch(p2, p1);
end
