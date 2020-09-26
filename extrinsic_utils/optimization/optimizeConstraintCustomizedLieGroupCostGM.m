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
 * AUTHOR: Isabel Taylor (igtaylor[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}


function [opt, weights] = optimizeConstraintCustomizedLieGroupCostGM(opt, Y, target_size, box_width)
    % EDIT Geman cost funtion Mclure parameters
    tau = 1; % chosen param
    mu = 10000; % chosen param, needs to be large number, will decrease to 1
    cbar = 1; % chosen param
    [tmp, N] =  size(Y); % from data size(Y)
    Y_prev = Y; % set raw data into variable optimzied over
    omega_prev = ones(1,N); % chosen starting weights, can be changed to [0,1]
   

    
    % recovers the original robust cost function when mu = 1. So iterate over
    % mu until reaching 1, EDIT the iteration step at bottom of loop

    max_itr = 10; % EDIT needed in case of loop getting stuck
    itr = 0;
    while (mu ~= 0000)
        display(mu);
        if itr > max_itr
            warning("to many iterations over mu")
            break;
        end
        display(itr);
        itr = itr + 1;
        
        % optimize weights and variable to closer to robust cost function
        
        %% variable update
        % set optimization varibales, R_v and T_v are roation and
        % translation of for Lie Group element
        R_v = optimvar('R_v', 1, 3); % 1x3
        T_v = optimvar('T_v', 1, 3); 
        prob = optimproblem;
        % define fmincon function
        f = fcn2optimexpr(@computeConstraintCustomizedLieGroupCostGMvar, Y_prev, ...
                           R_v, T_v, target_size, box_width, omega_prev);
        prob.Objective = f;
        
        if itr == 1 % use intial x0 on first iteration, on all else, use the last solution as the intial guess
            if isstruct(opt)
                opt.H_prev = eye(4);
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
        else
            x0.R_v = Log_SO3(opt.H_opt(1:3,1:3));
            x0.T_v = opt.H_opt(1:3,4)';
        end
        
        % x0.theta_x = opt.rpy_init(1);
        % x0.theta_y = opt.rpy_init(2);
        % x0.theta_z = opt.rpy_init(3);
        % x0.T = opt.H_init(1:3, 4);
    %             options = optimoptions('fmincon', 'MaxIter',5e2,'Display','iter', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
        options = optimoptions('fminunc', 'MaxIter',5e2, 'Display','off', 'TolX', 1e-6, 'TolFun', 1e-6);
        max_trail = 5;
        num_tried = 1;
        status = -1;
        while status <0 
            [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
            if status <0 
                warning("optimization failed")
            end
            num_tried = num_tried + 1;
            if (num_tried + 1 > max_trail)
                warning("tried too many time, optimization still failed, current status:")
                disp(status)
                break;
            end
        end
        
        % define Y_new using the optimized variable for this interation of mu
        H_opt = eye(4);
        H_opt(1:3, 1:3) = Exp_SO3(sol.R_v);
        H_opt(1:3, 4) = sol.T_v';
        opt.opt_total_cost = fval;
        opt.H_opt = H_opt;
        
       %% weight update

        cost_z = 0;
        cost_x = 0;
        cost_y = 0;
        for i = 1:size(Y_prev, 2)
            % ith point
            % calculate residual
            z_prime = Y_prev(3,i);
            y_prime = Y_prev(2,i);
            x_prime = Y_prev(1,i);
            cost_z = checkCost(z_prime, -target_size/2, target_size/2);
            cost_y = checkCost(y_prime, -target_size/2, target_size/2);
            cost_x = checkCost(x_prime, -box_width, box_width);
            res = cost_z + cost_y + cost_x;
            % equation 12
            omega_new(i) = ((mu * cbar^2) / (res^2 + mu * cbar^2))^2;
        end
        omega_prev = omega_new; % set new weights as old
%% EDIT iteration over mu parameters
       if mu <= 100
           mu = mu - 1;
       else
           mu = mu - 100;
       end
        
     end % end of while loop
     weights = omega_prev; % return final weights 
    
end