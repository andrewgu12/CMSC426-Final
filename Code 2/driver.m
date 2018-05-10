R = orth(rand(3,3));

if (det(R) < 0)
    R(:,3) = R(:,3) * -1;
end

t = rand(3,1);

n = 100;
A = rand(n,3);
B = R*A' + repmat(t, 1, n);
B = B';

[ret_R, ret_t] = my_icp(A, B);

A2 = (ret_R*A') + repmat(ret_t, 1, n);
A2 = A2';

err = get_rmse(A2, B);

disp(sprintf('root-mean-square error: %f', err));