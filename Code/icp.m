function [] = icp(s, d, max_iters)
% maxiters = 100;
error = .001;

R = [1 0 0; 0 1 0; 0 0 1];
t = [0,0,0];
    
while iter < max_iters && e < error
     mapping = (s, d)   
end

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
% ?niy(si ? di) · ni? niz(si ? di) · ni
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