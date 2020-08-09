%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
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

function opt = optimizeCost(opt, X, target_size, box_width)


    if opt.UseCentroid
        [~, centroid, U] = computePlaneReturnR(X(1:3,:));

        % rotm_init = U(:, [1 2 3])';
        % rotm_init = U(:, [1 3 2])';
        % rotm_init = U(:, [2 1 3])';
        % rotm_init = U(:, [2 3 1])';
        
        % project onto yz plane
        if abs(det(U) -1) > 1e-5
            rotm_init = U(:, [3 2 1])';
        else
            rotm_init = U(:, [3 1 2])';
        end
        opt.rpy_init = rad2deg(rotm2eul(rotm_init, "XYZ"));
        opt.T_init = -rotm_init*centroid(1:3);
        opt.H_init = constructHByRPYXYZMovingPoints(opt.rpy_init, opt.T_init);       
    end
    opt.H_init = constructHByRPYXYZMovingPoints(opt.rpy_init, opt.T_init);

    switch opt.method
        case 'Customize'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            optimizeCustomizedCost(opt);
            opt.computation_time = cputime - t_start;
        case 'Constraint Customize'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case {'Constraint Customize Lie Group', 'Constraint Customize Lie Group-Optimal Shape'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedLieGroupCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case {'Constraint Customize Lie Group-2-step-Optimal Shape'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedLieGroup2StepsCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
            
        case {'Constraint Customize Lie Group-Searching-Optimal Shape'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedLieGroupSearchingCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case {'Constraint Customize Lie Group-Global-Optimal Shape'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedLieGroupGlobalCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case {'Constraint Customize Lie Group-ManOpt-Optimal Shape'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedLieGroupManOptCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case {'Constraint Customize Convex Relaxation'}
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeConstraintCustomizedConvexRelaxationCost(opt, X, target_size, box_width);
            opt.computation_time = cputime - t_start;
        case 'The Wall'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            t_start = cputime;
            opt = optimizeBoundaries(opt, opts, X);
            opt.computation_time = cputime - t_start;
        case 'Coherent Point Drift'
            opt.metric = "--";
            opt.unit = "RMSE";
            t_start = cputime;
            optimizeCPD(opt);
            opt.computation_time = cputime - t_start;
        case 'Iterative Closest Point (point)'
            opt.metric = "PointToPoint";
            opt.unit = "RMSE";
            t_start = cputime;
            optimizeICPPoint(opt);
            opt.computation_time = cputime - t_start;
        case 'Iterative Closest Point (plane)'
            opt.metric = "PointToPlane";
            opt.unit = "RMSE";
            t_start = cputime;
            optimizeICPPlane(opt);
            opt.computation_time = cputime - t_start;
        case 'Normal-distributions Transform'
            opt.metric = "--";
            opt.unit = "RMSE";
            t_start = cputime;
            optimizeNDT(opt);
            opt.computation_time = cputime - t_start;
        case 'GICP-SE3'
            t_start = cputime;
            optimizeGICP_SE3(opt)
            opt.computation_time = cputime - t_start;
        case 'GICP-SE3 (plane)'
            t_start = cputime;
            optimizeGICPPlane_SE3(opt)
            opt.computation_time = cputime - t_start;
        case 'GICP-SE3-costimized'
            t_start = cputime;
            optimizeCustomizedGICP_SE3(opt);
            opt.computation_time = cputime - t_start;
        case 'Two Hollow Strips'
            t_start = cputime;
            optimizeHollowStrips(opt);
            opt.computation_time = cputime - t_start;
        case '3D IoU'
        case 'Project'
            t_start = cputime;
            OptimizeUsingRobustNormalVector(opt);
            opt.computation_time = cputime - t_start;
        otherwise
            disp('no such optimization method')
            return
    end
end
