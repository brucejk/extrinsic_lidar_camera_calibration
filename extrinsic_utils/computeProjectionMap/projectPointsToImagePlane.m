function projected_points = projectPointsToImagePlane(points_mat, projection_matrix)
    [~, points_mat] = checkHomogeneousCoord(points_mat(1:3, :));
    projected_points = projection_matrix * points_mat;
    projected_points = projected_points ./ projected_points(3, :);
end