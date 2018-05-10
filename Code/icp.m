function [S] = icp(s, d, max_iters)
    % maxiters = 100;
    error = .001;

    R = eye(3)
    t = zeros(1,3);
    n = pcnormals(pointCloud(s));
    iter = 1;  
    dist = 1;

    C = 0;
    B = 0;
    
    while (iter < max_iters && dist > error)
        indS = length(d);
        if length(d) > length(s)
            indS = length(s);
            s = padarray(s,[length(d)-length(s) 0],0,'post');
        end
        
         ind = dsearchn(s,d);
         d = d(ind,:);
         d = d(1:indS,:);
         s = s(1:indS,:);
         
         dist = mean(norm(s-d));

         for i = 1:length(s)
            c = cross(s(i,:),n(i,:));
            
            %C = zeros(6,6);

             test = [c(1,1); c(1,2); c(1,3); n(i,1); n(i,2); n(i,3)];

             % Calculate C
             C = C + [test*c(1,1) test*c(1,2) test*c(1,3) test*n(i,1) test*n(i,2) test*n(i,3)];

             b = zeros(6,1);

             b(1,1) = dot(c(1,1)*(s(i,:)-d(i,:)),n(i,:));
             b(2,1) = dot(c(1,2)*(s(i,:)-d(i,:)),n(i,:));
             b(3,1) = dot(c(1,3)*(s(i,:)-d(i,:)),n(i,:));
             b(4,1) = dot(n(i,1)*(s(i,:)-d(i,:)),n(i,:));
             b(5,1) = dot(n(i,2)*(s(i,:)-d(i,:)),n(i,:));
             b(6,1) = dot(n(i,3)*(s(i,:)-d(i,:)),n(i,:));
             
             % Calculate B
             B = B + b;
         end
         x = C\-B;
         
         Rcurr = x(1:3,1)';
         tcurr = x(4:6,1)';
         
         s = Rcurr.*s;
         s = s + tcurr;
         R = Rcurr.*R;
         t = Rcurr.* tcurr + t;
         
         iter = iter+1;
    end

    S = s;
end

% procedure ICP(s, d, max iters)
% 2: ? = .001
% 3: R, t = (3x3 identity), (0,0,0)
% 4: whilei?maxitersanddist>?do
% 5: mapping = closest point(s, d)
% 6: dist = (mean distance between correspondending points)
% 7: [Ri , ti ] =rigid   transform(s, d,mapping) ? Calculate R,t for iteration i
% 8:
% 9: s=Ri ?s? +ti ?updateR,t,and
% ?niy(si ? di) � ni? niz(si ? di) � ni
%    ? Point clouds s,d (source, destination) ? Some threshold for convergence.
%   2.3.2 Extra Credit Opportunity: Estimating Surface Normals (5 pts)
% Point-to-plane ICP requires that we estimate the surface normal ni at every destination point di. You may use the Matlab function pcnormals, or implement it yourself for 5 points of extra credit.
% Surface normal estimation can be done by fitting a plane through a small region of points and getting the normal of that plane. For each point, find the k closest neighbors. Since these points are highly unlikely to be precisely co-planar, we need to find a best-fit plane using least squares. To do this, form a matrix K out of the k points:
% ?p1x p1y ?p2x p2y
% p1z ? p2z ?
% K = ? .
% pkx pky pkz
% ? ?.?
% 4
% ? Find point correspondence.
%  10: R = Ri ? R
% 11: t=Ri ?ti +t
% 12: increment i;
% 13: end while
% 14: return R, t
% 15: end procedure