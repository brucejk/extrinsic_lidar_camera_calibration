%% plot
% clc, close all;
% num_targets=1;
function plotSanityCheckLie(num_targets, plane,data_split_with_ring,delta)
    mesh_size = [-1, 1];
    for i =1:num_targets
        w = null(plane{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
        [P,Q] = meshgrid(mesh_size(1):mesh_size(2)); % Provide a gridwork (you choose the size)
        X = plane{i}.centriod(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
        Y = plane{i}.centriod(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
        Z = plane{i}.centriod(3)+w(3,1)*P+w(3,2)*Q;
        figure(1);
        surf(X,Y,Z);
        hold on;
    %     axis equal
    %     scatter3(data{i}(1,:),data{i}(2,:),data{i}(3,:), 30, 'b.');
        for ring =1:32
            if(checkRingsCrossDataset(data_split_with_ring, delta, num_targets, ring))
                continue
            end
            if (isempty(data_split_with_ring{i}(ring).points) || size(data_split_with_ring{i}(ring).points, 2) < 10)
                continue;
            else
%                 ring
                scatter3(data_split_with_ring{i}(ring).points(1,:),...
                         data_split_with_ring{i}(ring).points(2,:),...
                         data_split_with_ring{i}(ring).points(3,:), 200, 'b.');
                transformedPoints = delta(ring).Affine * data_split_with_ring{i}(ring).points;
                scatter3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:), 200, 'r.');
            end
        end   
    end
end