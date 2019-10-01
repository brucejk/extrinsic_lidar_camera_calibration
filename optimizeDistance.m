function delta_D = optimizeDistance(sorted_d, sorted_d_hat)
    delta_D = sum(sorted_d - sorted_d_hat)/size(sorted_d);
end
