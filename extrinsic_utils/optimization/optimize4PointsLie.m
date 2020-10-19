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

function [H_LC, P, total_cost, RMSE] = optimize4PointsLie(opt, X, Y, intrinsic, display)
    if ~exist('display', 'var')
        display = 0;
    end
    if isfield(opt, 'weighted_lie')
        weighted_lie = opt.weighted_lie;
    else
        weighted_lie = 0;
    end
    
    if ~isfield(opt, 'tight_optimization')
        opt.tight_optimization = 0;
    end
    
    R_v = optimvar('R_v', 1, 3); % 1x3
    if isfield(opt, 'T_lb') && isfield(opt, 'T_ub')
        T_v = optimvar('T_v', 1, 3, 'LowerBound', opt.T_lb, 'UpperBound', opt.T_ub);
    else
        warning("Using lb: -0.5, ub: 0.5 for translation")
        T_v = optimvar('T_v', 1, 3, 'LowerBound', -0.5, 'UpperBound', 0.5);
    end
    prob = optimproblem;
    
    if weighted_lie
        f = fcn2optimexpr(@costWeighted4PointsLie, R_v, T_v, X, Y, intrinsic);
    else
        f = fcn2optimexpr(@cost4PointsLie, R_v, T_v, X, Y, intrinsic);
    end
    
    prob.Objective = f;
    if isstruct(opt)
        R = rotx(opt.rpy_init(1)) * roty(opt.rpy_init(2)) * rotz(opt.rpy_init(3));
        init_Rv = Log_SO3(R);
        x0.R_v = init_Rv;
        x0.T_v = opt.T_init;
        
    else
        R = rotx(opt(1)) * roty(opt(2)) * rotz(opt(3));
        init_Rv = Log_SO3(R);
        x0.R_v = init_Rv;
        if length(opt) > 3
            x0.T_v = opt(4:6);
        else
            x0.T_v = [0 0 0];
        end
    end
    
    if weighted_lie && checkDisplay(display)
        cost_init = costWeighted4PointsLie(x0.R_v, x0.T_v, X, Y, intrinsic)
    elseif ~weighted_lie && checkDisplay(display)
        cost_init = cost4PointsLie(x0.R_v, x0.T_v, X, Y, intrinsic)
    end
    
    if opt.tight_optimization
        options = optimoptions('fmincon', 'MaxIter',5e12, 'TolX', 1e-12, 'Display','off', 'FunctionTolerance', 1e-12, 'MaxFunctionEvaluations', 5e12, 'OptimalityTolerance', 1e-12);
    else
        options = optimoptions('fmincon', 'MaxIter',5e2, 'TolX', 1e-12, 'Display','off', 'FunctionTolerance', 1e-8, 'MaxFunctionEvaluations', 3e4);
    end
%     
    [sol, fval, ~, ~] = solve(prob, x0, 'Options', options);
    H_LC = eye(4);
    H_LC(1:3, 1:3) = Exp_SO3(sol.R_v);
    H_LC(1:3, 4) = sol.T_v';
    P = intrinsic * [eye(3) zeros(3,1)] * H_LC;

    total_cost = fval;
    RMSE = sqrt(fval/size(Y,2));
end
