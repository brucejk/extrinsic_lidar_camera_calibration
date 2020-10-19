function square_summed_error = cost4PointsForConvexRelaxation(x, X, Y, intrinsic)
    R = reshape(x(1:9), 3, 3);
    T = reshape(x(10:12), 3, 1);
    H = eye(4);
    H(1:3, 1:3) = R;
    H(1:3, 4) = T;
    P = intrinsic *  [eye(3), zeros(3,1)] * H;
    square_summed_error= verifyCornerAccuracyNoChecking(X, Y, P);
end