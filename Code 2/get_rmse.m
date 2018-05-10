function error = get_rmse(p1,p2)
    sqr_dist = sum(power(p1 - p2, 2));
    error = sqrt(mean(sqr_dist));
end