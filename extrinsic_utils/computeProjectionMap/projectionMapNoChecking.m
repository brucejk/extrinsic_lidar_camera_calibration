% clc
% P = rand(3, 4);
% X = rand(4, 8);
% t_projectionMapNoChecking(X, P)
function projected_points_s = projectionMapNoChecking(X, P)
    num_feature = size(X, 2);
    projected_points = P * X;
    projected_points_s = ones(3, num_feature);
    
    for i = 1:num_feature
        projected_points_s(1, i) = projected_points(1, i) / projected_points(3, i);
        projected_points_s(2, i) = projected_points(2, i) / projected_points(3, i);
    end
end