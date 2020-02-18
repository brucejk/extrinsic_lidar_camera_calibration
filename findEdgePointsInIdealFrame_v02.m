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
%     fig_hangle = figure(99999);
%     cla(fig_hangle)
%     X_move_back = inv(H_TL) * [X_ref(1:3,:); ones(1, size(X_ref(1, :), 2))];
%     scatter3(X_move_back(1,:), X_move_back(2,:), X_move_back(3,:), 'k.')
%     %     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:), 'k.')
%     hold on
%     scatter3(edge(1).points(1,:), edge(1).points(2,:), edge(1).points(3,:), 'ro', 'filled')
%     scatter3(edge(2).points(1,:), edge(2).points(2,:), edge(2).points(3,:), 'go', 'filled')
%     scatter3(edge(3).points(1,:), edge(3).points(2,:), edge(3).points(3,:), 'bo', 'filled')
%     scatter3(edge(4).points(1,:), edge(4).points(2,:), edge(4).points(3,:), 'mo', 'filled')
%     title('edge parsing results')
%     xlabel("x")
%     ylabel("y")
%     zlabel("z")
%     axis equal
end