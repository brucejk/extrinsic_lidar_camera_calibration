function cost = optimizeMultiIntrinsicCost (X, plane, ring, theta_x, theta_y, theta_z, T)
    cost = 0;
    for i = 1: length(X)
        cost = cost + optimizeIntrinsicCost(X{i}(ring), plane{i}, theta_x, theta_y, theta_z, T);
    end
end
