% %% plot
% close all;
% % num_targets=1;
function plotSanityCheckSpherical(num_targets,plane,data_split_with_ring, data, delta)
    for i =1:num_targets
        w = null(plane{i}.unit_normals'); % Find two orthonormal vectors which are orthogonal to v
        [P,Q] = meshgrid(-1:1); % Provide a gridwork (you choose the size)
        X = plane{i}.centriod(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
        Y = plane{i}.centriod(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
        Z = plane{i}.centriod(3)+w(3,1)*P+w(3,2)*Q;
        figure(1);
        surf(X,Y,Z);
        hold on;
        scatter3(data{i}(1,:),data{i}(2,:),data{i}(3,:), 30, 'b.');
        for ring =1:32
            if (isempty(data_split_with_ring{i}(ring).points))
                continue;
            else
%                 scatter3(data_split_with_ring_raw{i}(ring).points(1,:),...
%                          data_split_with_ring_raw{i}(ring).points(2,:),...
%                          data_split_with_ring_raw{i}(ring).points(3,:), 50, 'b.');
                transformedPoints = zeros(size(data_split_with_ring{i}(ring).points));
    %             transformedPoints(1,:) = (data_split_with_ring{i}(ring).points(1,:)).*sin(data_split_with_ring{i}(ring).points(2,:)).*cos(data_split_with_ring{i}(ring).points(3,:));
    %             transformedPoints(2,:) = (data_split_with_ring{i}(ring).points(1,:)).*sin(data_split_with_ring{i}(ring).points(2,:)).*sin(data_split_with_ring{i}(ring).points(3,:));
    %             transformedPoints(3,:) = (data_split_with_ring{i}(ring).points(1,:)).*cos(data_split_with_ring{i}(ring).points(2,:));
                transformedPoints(1,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*cos(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
                transformedPoints(2,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*sin(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
                transformedPoints(3,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*cos(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta);
                scatter3(transformedPoints(1,:),transformedPoints(2,:),transformedPoints(3,:), 50, 'r.');
            end
        end
    end
end