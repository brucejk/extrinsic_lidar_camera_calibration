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

function opt = optimizeConstraintCustomizedLieGroupCost(opt, X, target_size, box_width)
    R_v = optimvar('R_v', 1, 3); % 1x3
    T_v = optimvar('T_v', 1, 3);

%     computeConstraintCustomizedLieGroupCost(X, zeros(1,3), zeros(1,3), 1, 0.2)
    prob = optimproblem;
    
    if string(opt.method) == "Constraint Customize Lie Group"
        f = fcn2optimexpr(@computeConstraintCustomizedLieGroupCost, X, ...
                           R_v, T_v, target_size, box_width);
    elseif string(opt.method) == "Constraint Customize Lie Group-Optimal Shape"
        [~, X_h] = checkHomogeneousCoord(X(1:3, :), 1);
        [sorted_vertices, ~, ~] = sortVertices([], opt.ideal_frame);
        num_edges = size(sorted_vertices, 2);
        edge_lines = zeros(num_edges, 3);
        
        for edge_i = 1:num_edges
            edge_j = max(mod(edge_i+1, num_edges+1), 1);
            v1 = sorted_vertices(2:3, edge_i);
            v2 = sorted_vertices(2:3, edge_j);
            
            if abs(v1(1)- v2(1)) < 1e-5
                % x = v1(1) -> x - v1(1) = 0
                edge_lines(edge_i, :) = [1, 0, -v1(1)];
            else
                p = polyfit([v1(1) v2(1)], [v1(2), v2(2)], 1); 
                % y = p(1)x + p(2) -> p(1)x - y + p(2)
                edge_lines(edge_i, :) = [p(1), -1, p(2)];
            end
        end
        
        x_std = std(sorted_vertices, 0, 2);
        if x_std>1e-5
            warning("The ideal frame is not perpendicular to x-axis, the optimization might fail")
        end
        X_trans = opt.H_init*X_h;
        h1 = scatter3(opt.figure(1).axes, X_trans(1,:), X_trans(2, :), X_trans(3,:), 'b.');
        viewCurrentPlot(opt.figure(1).axes, "Optimization-step0", [-89, 0])
%         computeConstraintCustomizedLieGroupOptimalShapeCost(sorted_vertices, edge_lines, X_h, ones(1, 3), zeros(1, 3), box_width);
        f = fcn2optimexpr(@computeConstraintCustomizedLieGroupOptimalShapeCost, ...
                                   sorted_vertices, edge_lines, X_h, R_v, T_v, box_width);        
    else
        error('no such optimization method: %s', opt.method)
    end
    prob.Objective = f;

	if isstruct(opt)
        init_Rv = Log_SO3(opt.H_init(1:3, 1:3));
        x0.R_v = init_Rv;
        x0.T_v = opt.H_init(1:3, 4);
    else
        init_Rv = Log_SO3(opt.H_init(1:3, 1:3));
        x0.R_v = init_Rv;
        if length(opt) > 3
            x0.T_v = opt(4:6);
        else
            x0.T_v = [0 0 0];
        end
	end
%     opt.H_init
    % x0.theta_x = opt.rpy_init(1);
    % x0.theta_y = opt.rpy_init(2);
    % x0.theta_z = opt.rpy_init(3);
    % x0.T = opt.H_init(1:3, 4);
%             options = optimoptions('fmincon', 'MaxIter',5e2,'Display','iter', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
%     options = optimoptions('fminunc', 'MaxIter',5e3, 'Display','iter', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
    options = optimoptions('fminunc', 'MaxIter',5e3, 'Display','off', 'TolX', 1e-6, ...
                'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4, 'Algorithm','quasi-newton');

%     [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
    max_trail = 5;
    num_tried = 1;
    status = -1;
    while status < 0 
%         [sol,fval,status,osur] = surrogateopt(f,lb,ub,opts);
        [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
        if status < 0 
            warning("optimization failed")
        end
        num_tried = num_tried + 1;
        if (num_tried + 1 > max_trail)
            error("tried too many time, optimization still failed, current status: %s", status)
        end
    end
    opt.H_opt = eye(4);
    opt.H_opt(1:3, 1:3) = Exp_SO3(sol.R_v);
    opt.H_opt(1:3, 4) = sol.T_v';
    opt.opt_total_cost = fval;
    
%     X_final = opt.H_opt*X_h;
%     h2 = scatter3(opt.figure(1).axes, X_final(1,:), X_final(2, :), X_final(3,:), 'r*');
%     viewCurrentPlot(opt.figure(1).axes, "Optimization-step2", [-89, 0])
end
