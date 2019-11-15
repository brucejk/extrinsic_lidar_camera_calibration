function [delta, opt]= estimateDelta(opt, data, plane, delta)
    tic;
    for ring = 1:length(data)

        if size(data(ring).points,2) == 0
            delta(ring).H = eye(4);
            continue
        end
        ring
%         dbstop in estimateDelta.m at 13 if ring>=32
        theta_x = optimvar('theta_x', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
        theta_y = optimvar('theta_y', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
        theta_z = optimvar('theta_z', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
        T = optimvar('T', 1, 3); % 1x3

%         theta_x = 0;
%         theta_y = 0;
%         theta_z = 0;
%         T = [0 0 0];
%         cost = optimizeIntrinsicCost(data(ring), plane, theta_x, theta_y, theta_z, T);
%                        
        prob = optimproblem;
        f = fcn2optimexpr(@optimizeIntrinsicCost, data(ring), plane, ...
                           theta_x, theta_y, theta_z, T);
        prob.Objective = f;
        x0.theta_x = opt.rpy_init(1);
        x0.theta_y = opt.rpy_init(2);
        x0.theta_z = opt.rpy_init(3);
        x0.T = opt.T_init;

        options = optimoptions('fmincon', 'MaxIter',5e2, 'Display','off', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
        max_trail = 5;
        num_tried = 1;
        status = 0;
        while status <=0 
            [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
            if status <=0 
                warning("optimization failed")
            end
            num_tried = num_tried + 1;
            if (num_tried + 1 > max_trail)
                warning("tried too many time, optimization still failed, current status:")
                disp(status)
                break;
            end
        end
        R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
        delta(ring).H = eye(4);
        delta(ring).H(1:3, 1:3) = R_final;
        delta(ring).H(1:3, 4) = sol.T;
        delta(ring).opt_total_cost = fval;
        opt.computation_time = toc;
    end
end