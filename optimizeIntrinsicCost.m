function cost = optimizeIntrinsicCost(X, plane, theta_x, theta_y, theta_z, T)
    H = eye(4);
    H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    H(1:3,4) = T';
    X_prime = H * X.points;
    plane_centroids = repmat(plane.centriod, [1,size(X_prime, 2)]);
    diff = abs([X_prime - plane_centroids]);
    normals = repmat(plane.unit_normals, [1,size(X_prime, 2)]);
%     cost = norm([plane.normals .* diff(1:3,:)],'fro');
    cost = sum(abs(dot(normals, diff(1:3,:))));
end