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

function [H_LC, P, total_cost, RMSE] = optimize4PointsEulerAngle(opt, X, Y, intrinsic)
        
    % prepare for random initilization (not use for now)
        
%         theta_x = opt.init_rpy(1);
%         theta_y = opt.init_rpy(2);
%         theta_z = opt.init_rpy(3);
%         T = opt.init_T;
%         cost = cost4Points(theta_x, theta_y, theta_z, T, X, Y, intrinsic)
%         
        theta_x = optimvar('theta_x', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        theta_y = optimvar('theta_y', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        theta_z = optimvar('theta_z', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        
        if isfield(opt, 'T_lb') && isfield(opt, 'T_ub')
            T_v = optimvar('T_v', 1, 3, 'LowerBound', opt.T_lb, 'UpperBound', opt.T_ub);
        else
        warning("Using lb: -0.5, ub: 0.5 for translation")
        T_v = optimvar('T_v', 1, 3, 'LowerBound', -0.5, 'UpperBound', 0.5);
    end
        T = optimvar('T', 1, 3,'LowerBound',-0.5,'UpperBound',0.5);
        prob = optimproblem;
        
        f = fcn2optimexpr(@cost4Points, theta_x, theta_y, theta_z, T, X, Y, intrinsic);
        prob.Objective = f;

        if isstruct(opt)
            x0.theta_x = opt.rpy_init(1);
            x0.theta_y = opt.rpy_init(2);
            x0.theta_z = opt.rpy_init(3);
            if isfield(opt, 'T_init')
                x0.T = opt.T_init;
            elseif isfield(opt, 'init_T')
                x0.T = opt.init_T;
            else
                x0.T = [0 0 0];
            end
        else
            x0.theta_x = opt(1);
            x0.theta_y = opt(2);
            x0.theta_z = opt(3);
            if length(opt) > 3
                x0.T = opt(4:6);
            else
                x0.T = [0 0 0];
            end
        end
        cost4Points(x0.theta_x, x0.theta_y, x0.theta_z, x0.T, X, Y, intrinsic)
        
        options = optimoptions('fmincon', 'MaxIter',5e2, 'TolX', 1e-12, 'Display','off', 'FunctionTolerance', 1e-8, 'MaxFunctionEvaluations', 3e4);
        [sol, fval, ~, ~] = solve(prob, x0, 'Options', options);
        R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
        H_LC = eye(4);
        H_LC(1:3, 1:3) = R_final;
        H_LC(1:3, 4) = sol.T';    
        P = intrinsic * [eye(3) zeros(3,1)] * H_LC;

        total_cost = fval;
        RMSE = sqrt(fval/size(Y,2));
end
