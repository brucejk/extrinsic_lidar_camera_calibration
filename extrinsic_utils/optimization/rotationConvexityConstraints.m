function [c, ceq] = rotationConvexityConstraints(x)
    rotm = reshape(x(1:9), 3, 3);
    cons_matrix_true = constructConstrainMatrix(rotm);
    evalus_true = eig(cons_matrix_true);
    c = -real(evalus_true);
    ceq = 0;
end