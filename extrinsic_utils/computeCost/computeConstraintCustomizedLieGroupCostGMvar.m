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
 * AUTHOR: Isabel Taylor (igtalyor[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function total_cost = computeConstraintCustomizedLieGroupCostGMvar(X, R_v, T_v, target_size, box_width, omega_prev)
    H = eye(4);
    H(1:3, 1:3) = Exp_SO3(R_v);
    H(1:3, 4) = T_v;

    [~, X_h] = checkHomogeneousCoord(X);
    % transform data points before calxulations
    Y = H * X_h;
    
    total_weight_cost = 0;
    cost_z = 0;
    cost_x = 0;
    cost_y = 0;
    for i = 1:size(Y, 2)
        % ith point
        z_prime = Y(3,i);
        y_prime = Y(2,i);
        x_prime = Y(1,i);
        % residual cost
        cost_z = cost_z + checkCost(z_prime, -target_size/2, target_size/2);
        cost_y = cost_y + checkCost(y_prime, -target_size/2, target_size/2);
        cost_x = cost_x + checkCost(x_prime, -box_width, box_width);
        % total cost of all points
        total_weight_cost = total_weight_cost + (omega_prev(i) * (cost_z + cost_y + cost_x)^2); 
    end
    
    total_cost = total_weight_cost; % sum of omega(i) * residual error(i)

end




