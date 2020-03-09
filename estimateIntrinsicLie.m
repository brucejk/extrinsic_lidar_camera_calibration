function [delta, plane, valid_targets] = estimateIntrinsicLie(num_beams, num_targets, num_scans, data_split_with_ring, object_list, check_rings)
    delta(num_beams).H = struct();
    delta(num_beams).Affine = struct();
    if ~exist('check_rings', 'var')
        check_rings = 1;
    end
    
    for i = 1: num_scans
        % Calculate 'ground truth' points by projecting the angle onto the
        % normal plane
        %
        % Assumption: we have the normal plane at this step in the form:
        % plane_normal = [nx ny nz]

        % example normal lies directly on the x axis
        opt.corners.rpy_init = [45 2 3];
        opt.corners.T_init = [2, 0, 0];
        opt.corners.H_init = eye(4);
        opt.corners.method = "Constraint Customize"; %% will add more later on
        opt.corners.UseCentroid = 1;

        plane = cell(1,num_targets);
        
        for t = 1:num_targets
            if ~exist('object_list', 'var')
                X = [];
                for j = 1: num_beams
                    X = [X, data_split_with_ring{t}(j).points];
                end
                [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 1.5);
            else
                if ~isfield(object_list(t), 'centroid') || ~isfield(object_list(t), 'normal')
                    [normal, centroid] = computePlane(object_list(t));
                    plane{t}.centroid = [centroid; 1];
                    plane{t}.normals =  normal;
                    plane{t}.unit_normals = normal/(norm(normal));
                else
                    plane{t}.centroid =  [object_list(t).centroid; 1];
                    plane{t}.normals =  object_list(t).normal;
                    plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
                end
            end
        end


        opt.delta.rpy_init = [0 0 0];
        opt.delta.T_init = [0, 0, 0];
        opt.delta.scale_init = 1;
        opt.delta.H_init = eye(4);
        [delta, ~, valid_targets] = estimateDeltaLie(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets, check_rings);
    end
end