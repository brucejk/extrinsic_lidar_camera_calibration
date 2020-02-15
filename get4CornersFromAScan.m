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

function bag_data = get4CornersFromAScan(opt, bag_data)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
    for tag = 1:bag_data.num_tag
%         fprintf("----Tag %i/%i\n", tag, bag_data.num_tag)
        X = bag_data.lidar_target.payload_points;
        target_len = bag_data.lidar_target(tag).tag_size;
        target_len = 0.22;
        
        % clean data
        [X_clean, X_ref, X_std, L_infinity, N] = cleanLiDARTargetCore(opt.H_TL, X, target_len);
        bag_data.lidar_target(tag).clean_up.std = N*(X_std(1));
        bag_data.lidar_target(tag).clean_up.L_infinity = L_infinity;
        bag_data.lidar_target(tag).clean_up.L_1 = sum(X_ref(1:3,:), 1);

        % cost
        opt_tmp = optimizeCost(opt.H_TL, X_clean, target_len, bag_data.lidar_target(tag).clean_up.std/2);
        target_lidar = [0 -target_len/2 -target_len/2 1;
                        0 -target_len/2  target_len/2 1;
                        0  target_len/2  target_len/2 1;
                        0  target_len/2 -target_len/2 1]';

        corners = opt_tmp.H_opt \ target_lidar;
        corners = sortrows(corners', 3, 'descend')';
        [centroid, normals] = computeCentroidAndNormals(corners);

        bag_data.lidar_target(tag).corners = corners;
        bag_data.lidar_target(tag).four_corners_line = point3DToLineForDrawing(corners);
        bag_data.lidar_target(tag).pc_points_original = X;
        bag_data.lidar_target(tag).pc_points = X_clean;
        bag_data.lidar_target(tag).centroid = centroid;
        bag_data.lidar_target(tag).normal_vector = normals;
        bag_data.lidar_target(tag).H_LT = inv(opt_tmp.H_opt);
    end
end
