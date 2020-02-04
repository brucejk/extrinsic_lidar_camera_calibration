function cost = optimizeMultiIntrinsicCostSpherical (X, valid_targets, plane, ring, D_corr, theta_corr, phi_corr)
    cost = 0;
    num_targets = length(X); 
    for i = 1: num_targets
        if valid_targets.valid(1, i)
            cost = cost + optimizeIntrinsicCostSpherical(X{i}(ring), plane{i}, D_corr, theta_corr, phi_corr);
        end
    end
end
