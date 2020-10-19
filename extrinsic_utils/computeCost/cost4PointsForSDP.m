function square_summed_error = cost4PointsForSDP(R, T, X, Y, intrinsic)
    P = intrinsic *  [eye(3), zeros(3,1)] * [R, T; 0 0 0 1];
    square_summed_error= verifyCornerAccuracyNoChecking(X, Y, P);
end