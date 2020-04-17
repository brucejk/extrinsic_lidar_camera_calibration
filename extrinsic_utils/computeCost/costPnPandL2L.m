function cost = costPnPandL2L(theta_x, theta_y, theta_z, T, lidar_target, camera_target, intrinsic)
    H = eye(4);
    H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    H(1:3,4) = T';
    P = intrinsic * [eye(3) zeros(3,1)] * H;
    cost = compare3D2DLines(lidar_target.lidar_target.lines, camera_target.camera_target.lines, P) + ...
           verifyCornerAccuracy(lidar_target.X_train, camera_target.Y_train, P);
%     cost = compare3D2DLines(lidar_target.lidar_target.lines, camera_target.camera_target.lines, P);
%     cost = verifyCornerAccuracy(lidar_target.X_train, camera_target.Y_train, P);   
end