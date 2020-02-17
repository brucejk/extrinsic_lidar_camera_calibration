function edge = findEdgePointsInIdealFrame_v02(H_TL, X_ref, tag_length)
    % X_ref: points that have been transformed back to the ideal frame
    % X_ref(1,:) : x
    % X_ref(2,:) : y
    % X_ref(3,:) : z
    % X_ref(4,:) : I
    % X_ref(5,:) : R
    top_ring = max(X_ref(5, :));
    bottom_ring = min(X_ref(5, :));
    
    % left, bottom, right, top (y-axis, -z-axis, -y-axis, z-axis)
    edge_list(1).line = [-tag_length/2 0; -tag_length/2 1];
    edge_list(2).line = [0 -tag_length/2; 1 -tag_length/2];
    edge_list(3).line = [tag_length/2 0; tag_length/2 1];
    edge_list(4).line = [0 tag_length/2; 1 tag_length/2];
    edge(4).points = [];
    
    % choose one point per ring in this scan
    current_scan_points = X_ref;
    for i = bottom_ring:top_ring
        if size(current_scan_points(2:3, current_scan_points(5,:)==i), 2) > 1
            current_ring_points = current_scan_points(1:3, current_scan_points(5,:)==i);
            extreme_point1 = current_ring_points(:, 1);
            extreme_point2 = current_ring_points(:, end);
            dist1 = zeros(1, 4);
            dist2 = zeros(1, 4);
            for k = 1:4
                dist1(1, k) = pointToLineDistance(extreme_point1(2:3, :)', edge_list(k).line(1, :), edge_list(k).line(2, :));
                dist2(1, k) = pointToLineDistance(extreme_point2(2:3, :)', edge_list(k).line(1, :), edge_list(k).line(2, :));
            end
            [~, edge1_i] = min(dist1);
            [~, edge2_i] = min(dist2);
            edge_point_3D_1 = inv(H_TL) * [extreme_point1; 1];
            edge_point_3D_2 = inv(H_TL) * [extreme_point2; 1];
            edge(edge1_i).points = [edge(edge1_i).points, edge_point_3D_1];
            edge(edge2_i).points = [edge(edge2_i).points, edge_point_3D_2];
        end
    end
end