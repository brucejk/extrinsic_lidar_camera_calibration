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

function cost = regulizedCostOfFineTuneLiDARTagPoseLieGroup(R_v, T_v, ...
                     X, Y, PC, H_LT, P, edge_t, box_width)
    % X: estimated lidar vertices
    % Y: camera corners
    % PC: target point cloud
    % H_LT: transformation from lidar to target
    % P: projection map from lidar to camera

    
    % New delta H_LT
    R = Exp_SO3(R_v);
    d_H_LT = eye(4);
    d_H_LT(1:3,1:3) = R;
    d_H_LT(1:3, 4) = T_v';
    
    % move origanl estimated vertices and compute pnp cost
    X_prime = d_H_LT * convertToHomogeneousCoord(X(1:3, :));  % moved vertices in lidar frame 
    C_X_transformed = projectionMap(X_prime, P); % project the moved vertices to image plane
    pnp_cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro')^2;
    
    % move target point cloud back to the ideal frame and compute L1 cost
    cur_H_TL =  inv(H_LT) * inv(d_H_LT);
    PC_h = convertToHomogeneousCoord(PC(1:3, :));
    l1_cost = computeConstraintCustomizedLieGroupOptimalShapeSE3(cur_H_TL, [], PC_h, box_width, edge_t);

    
    % pnp_cost is in pixel; l1_cost is in meter
%     cost = 1*pnp_cost + 1e2*l1_cost; %1e3 for RSS paper for now
    cost = 1*pnp_cost + 1e-1*l1_cost; %1e3 for RSS paper for now
end