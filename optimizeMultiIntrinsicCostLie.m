function cost = optimizeMultiIntrinsicCostLie (X, plane, ring, theta_x, theta_y, theta_z, T, S)
    cost = 0;
    for i = 1: length(X)
        cost = cost + optimizeIntrinsicCostLie(X{i}(ring), plane{i}, theta_x, theta_y, theta_z, T, S);
    end
end
