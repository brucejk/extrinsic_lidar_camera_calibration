function edge = findEdgePointsInIdealFrame(H_TL, X_ref, tag_length)
    % X_ref: points that have been transformed back to the ideal frame
    % X_ref(1,:) : x
    % X_ref(2,:) : y
    % X_ref(3,:) : z
    % X_ref(4,:) : I
    % X_ref(5,:) : R
    % X_ref(6,:) : scan
    % X_ref(7,:) : ExpNmbr
    top_ring = max(X_ref(5, :));
    bottom_ring = min(X_ref(5, :));
    % Ideal frame (top-left, bottom-left, bottom-right, top-right)
%     ideal_frame.x = zeros(1,8);
%     ideal_frame.y = [-tag_length/2, -tag_length/2, -tag_length/2, tag_length/2, tag_length/2, tag_length/2, tag_length/2, -tag_length/2];
%     ideal_frame.z = [tag_length/2, -tag_length/2, -tag_length/2, -tag_length/2, -tag_length/2, tag_length/2, tag_length/2, tag_length/2];
    
    % left, bottom, right, top (y-axis, -z-axis, -y-axis, z-axis)
    edge_list = [tag_length/2, -tag_length/2, -tag_length/2, tag_length/2];
    edge(4).points = [];
    for i = bottom_ring:top_ring
        current_ring_points = X_ref(:, X_ref(5,:)==i);
        first_point = current_ring_points(:, 1);
        edge_distance = [abs(first_point(2) - edge_list(1)), ...
                         abs(first_point(3) - edge_list(2)), ...
                         abs(first_point(2) - edge_list(3)), ...
                         abs(first_point(3) - edge_list(4))];
        [~, first_index] = min(edge_distance);
        first_point = inv(H_TL) * [first_point(1:3); 1];
        edge(first_index).points = [edge(first_index).points, first_point];
        last_point = current_ring_points(:, end);
        edge_distance = [abs(last_point(2) - edge_list(1)), ...
                         abs(last_point(3) - edge_list(2)), ...
                         abs(last_point(2) - edge_list(3)), ...
                         abs(last_point(3) - edge_list(4))];
        [~, last_index] = min(edge_distance);
        last_point = inv(H_TL) * [last_point(1:3); 1];
        edge(last_index).points = [edge(last_index).points, last_point];
    end
%     figure(100)
%     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:), 'k.')
%     hold on
%     scatter3(edge(1).points(1,:), edge(1).points(2,:), edge(1).points(3,:), 'ro', 'filled')
%     scatter3(edge(2).points(1,:), edge(2).points(2,:), edge(2).points(3,:), 'go', 'filled')
%     scatter3(edge(3).points(1,:), edge(3).points(2,:), edge(3).points(3,:), 'bo', 'filled')
%     scatter3(edge(4).points(1,:), edge(4).points(2,:), edge(4).points(3,:), 'mo', 'filled')
%     xlabel("x")
%     ylabel("y")
%     zlabel("z")
%     axis equal
end