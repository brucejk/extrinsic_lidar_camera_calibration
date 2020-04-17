function perspective_matrix = constructPerspectiveMatrix(focal_length)
    perspective_matrix = [eye(3), zeros(3,1)];
    perspective_matrix(1, 1) = focal_length;
    perspective_matrix(2, 2) = focal_length;
end