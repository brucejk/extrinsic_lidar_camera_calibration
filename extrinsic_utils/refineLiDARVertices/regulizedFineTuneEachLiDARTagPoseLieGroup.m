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

function [X_refined, H_LT_refined] = regulizedFineTuneEachLiDARTagPoseLieGroup(opt, X, Y, PC, H_LT, P, ideal_frame, box_width,display)
    [sorted_vertices, ~, ~] = sortVertices([], ideal_frame);
    edge_t = computeEdgeCoefficients(sorted_vertices);
% disp(regulizedCostOfFineTuneLiDARTagPoseLieGroup(x0.R_v, x0.T_v, X, Y, PC, H_LT, P, edge_t, box_width))

    R_v = optimvar('R_v', 1, 3); % 1x3
    if isstruct(opt)
        if isfield(opt, 'T_lb') && isfield(opt, 'T_ub')
            T_v = optimvar('T_v', 1, 3, 'LowerBound', opt.T_lb, 'UpperBound', opt.T_ub);
        else
            warning("Using lb: -0.1, ub: 0.1 for translation")
            T_v = optimvar('T_v', 1, 3, 'LowerBound', -0.1, 'UpperBound', 0.1);
        end
    else
        warning("Using lb: -0.1, ub: 0.1 for translation")
        T_v = optimvar('T_v', 1, 3, 'LowerBound', -0.1, 'UpperBound', 0.1);
    end
    
    prob = optimproblem;
    f = fcn2optimexpr(@regulizedCostOfFineTuneLiDARTagPoseLieGroup, R_v, T_v, ...
                             X, Y, PC, H_LT, P, edge_t, box_width);
    prob.Objective = f;
    x0.R_v = [0 0 0];
    x0.T_v = [0 0 0];           

    options = optimoptions('fmincon', 'MaxIter',5e2, 'Display','off', ...
                           'TolX', 1e-12, 'FunctionTolerance', 1e-8, ...
                           'MaxFunctionEvaluations', 3e4, 'StepTolerance', 1e-20);
    [sol, fval, ~, ~] = solve(prob, x0, 'Options', options);
    
    
    d_R = Exp_SO3(sol.R_v);
    d_H = eye(4);
    d_H(1:3, 1:3) = d_R;
    d_H(1:3, 4) = sol.T_v';

    if checkDisplay(display)
        disp('d_H_LT: ')
        disp(d_H)
        disp("previous cost:")
        disp(regulizedCostOfFineTuneLiDARTagPoseLieGroup(x0.R_v, x0.T_v, X, Y, PC, H_LT, P, edge_t, box_width))
        disp('cost:')
        disp(fval)

    end
    H_LT_refined = d_H * H_LT;
    X_refined = d_H * X;
end
