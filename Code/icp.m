function [S] = icp(s, d, max_iters)
    % maxiters = 100;
    error = .001;

    R = eye(3);
    t = zeros(1,3);
    n = pcnormals(pointCloud(s));
    iter = 1;  
    dist = 1;

    C = 0;
    B = 0;
    
    while (iter < max_iters && abs(dist) > error)
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
         
         Rcurr = R;
         
         %aby
         Rcurr(1,2) = -x(3,1);
         Rcurr(2,1) = x(3,1);
         Rcurr(1,3) = x(2,1);
         Rcurr(3,1) = -x(2,1);
         Rcurr(2,3) = -x(1,1);
         Rcurr(3,2) = x(1,1);
         
         
         tcurr = x(4:6,1);
%          
%          p2=[p2(1,:)+t1(1) ; p2(2,:)+t1(2); p2(3,:)+t1(3)];
%         p2=p2';
%         R = R1*R;
%         t = R1*t + t1;
           
         s = Rcurr*s';
         s = [s(1,:)+tcurr(1,1) ; s(2,:)+tcurr(2,1); s(3,:)+tcurr(3,1)];
         s = s';
         R = Rcurr*R;
         t = Rcurr.*t+ tcurr';
         
         iter = iter+1;
    end

    S = s;
    
end