function [convex_hull, area_2D] = makeConvexHull(X)
    X = makeTallMatrix(X);
    if size(X, 2) == 3
        if mean(X(:, 3), 3) == 1
            X_no_ones = X(:,1:2);
        end
    else
         X_no_ones = X;
    end
    [index, area_2D] = convhull(X_no_ones);
    convex_hull = X_no_ones(index(1:end-1), :)';
%     convex_hull = X';
end