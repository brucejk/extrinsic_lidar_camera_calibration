%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [H, final_P, cost] = optimizeConvex4PointsFmincon(opt, X, Y, intrinsic, display)
    [~, X_h] = checkHomogeneousCoord(X(1:3, :), 1);

    if isstruct(opt)
        x0.R = rotx(opt.rpy_init(1))*roty(opt.rpy_init(2))*rotz(opt.rpy_init(3));
        x0.T = opt.T_init;
    else
        x0.R = rotx(opt(1))*roty(opt(2))*rotz(opt(3));
        if length(opt) > 3
            x0.T = opt(4:6);
        else
            x0.T = [0 0 0];
        end
    end
    x0_vec = [reshape(x0.R, 9, 1); reshape(x0.T, 3, 1)];
%         mincon_cost(x, X_h, box_width, edge_t)
    [x_sol,fval,exitflag,output] = fmincon(@(x) cost4PointsForConvexRelaxation(x, X, Y, intrinsic),...
        x0_vec,...
        [], [],...
        [], [],...
        [], [],...
        @rotationConvexityConstraints);
    R_sol = reshape(x_sol(1:9), 3, 3);
    T_sol = reshape(x_sol(10:12), 3, 1);
    cost_eval = cost4PointsForConvexRelaxation(x_sol, X, Y, intrinsic)
    eigen_eval = eig(constructConstrainMatrix((R_sol)))
    H = eye(4);
    H(1:3, 1:3) = R_sol;
    H(1:3, 4) = T_sol;
    final_P = intrinsic *  [eye(3), zeros(3,1)] * H;
    cost = fval;

    
    if checkDisplay(display)
        disp('-- H_LC: ')
        disp('------- R:')
        disp(H(1:3, 1:3))
        disp('------- T:')
        disp(-inv(H(1:3, 1:3))*H(1:3, 4))
        disp('-- cost:')
        disp(fval)
        disp('Determinant of SO(3) element');
        determinant = det(R_sol)
    end
end
