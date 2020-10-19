function [X_clean, clean_up, remaining_indices, X_original] = removeLiDARTargetOutliers(X_original, target_len, opt)
    N = 4; % clean up using N std for x axis
    M = 3; % clean up using M std for y, z axis
    opt = optimizeCost(opt, X_original, target_len, 0.001);
    if size(X_original, 2) ~= 4
        X = [X_original(1:3, :); 
             ones(1 ,size(X_original, 2))];
    end
    
    X_ref = opt.H_opt * X;
%     distance = sum(X_ref(1:3,:), 1); % L1
    L_infinity = max(abs(X_ref(1:3,:)));  % L infinity
    K = find(L_infinity < (target_len/2)*1.025);
    X_ref_clean_yz = X_ref(:, K); % clean up y and z axis
    L_infinity = max(X_ref_clean_yz(1:3,:));
    K_center = find(L_infinity < target_len/4);
    X_std = std(X_ref_clean_yz(:, K_center), 1, 2);
    Q = find(abs(X_ref(1,:)) < N*(X_std(1))); % clean up x with 2 std
    remaining_indices = intersect(K, Q);
    X_ref_clean = X_ref(:, remaining_indices);
    X_clean = inv(opt.H_opt) * X_ref_clean;
    clean_up.std = N*(X_std(1));
    clean_up.L_infinity = L_infinity;
    clean_up.L_1 = sum(X_ref(1:3,:), 1);
end