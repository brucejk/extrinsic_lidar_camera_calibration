function distance = point2PlaneDistance(data, plane, num_beams, num_targets)
    distance(num_beams) = struct();
    for n = 1:num_beams
        X = [];
        for t = 1:num_targets
            point = data{t}(n).points;
            if isempty(point) 
                continue;
            end
            plane_centroids = repmat(plane{t}.centriod, [1,size(point, 2)]);
            diff = [point - plane_centroids];
            normals = repmat(plane{t}.unit_normals, [1,size(point, 2)]);
            X = [X abs(dot(normals, diff(1:3,:)))];
        end
        distance(n).mean = mean(X,2);
        distance(n).std = std(X);
    end


end