function cost = optimizeMultiIntrinsicCost (X, plane, ring, D_corr, theta_corr, phi_corr)
    cost = 0;
    num_targets = length(X); 
    for i = 1: num_targets
        cost = cost + optimizeIntrinsicCost(X{i}(ring), plane{i}, D_corr, theta_corr, phi_corr);
    end
end
