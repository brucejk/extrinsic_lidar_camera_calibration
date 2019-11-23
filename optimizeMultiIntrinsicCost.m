function cost = optimizeMultiIntrinsicCost (X, plane, ring, D_corr, theta_corr, phi_corr)
    cost = 0;
    for i = 1: length(X)
        cost = cost + optimizeIntrinsicCost(X{i}(ring), plane{i}, D_corr, theta_corr, phi_corr);
    end
end
