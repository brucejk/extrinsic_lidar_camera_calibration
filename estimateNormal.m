function [normals, distance]= estimateNormal(opt, X, target_len)
    opt = optimizeCost(opt, X, target_len);
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    corners = inv(opt.H_opt) * target_lidar;
    corners = sortrows(corners', 3, 'descend')';
    normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
    normals = normals/(norm(normals));
    distance = dot(normals, corners(1:3,1));
    normals = distance * normals;
    
    if normals(1) < 0
        normals = -normals;
    end
end