function [R, t] = my_icp(p1,p2)
    R=eye(3);
    t=zeros(3,1);
    pre_dist = 0;
    threshold_dist = .0001;
    for i=1:1000
        [index, curr_dist]=closest_point(p2, p1);
        [R1, t1]=rigid_transformation(p2, p1, index);
        p2=R1*p2';
        p2=[p2(1,:)+t1(1) ; p2(2,:)+t1(2); p2(3,:)+t1(3)];
        p2=p2';
        R = R1*R;
        t = R1*t + t1;
        dist = abs(pre_dist - curr_dist);
        pre_dist = dist;
        if (dist < threshold_dist)
            break
        end
    end
end