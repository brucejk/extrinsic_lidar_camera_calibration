function  [delta, plane, valid_targets] = estimateIntrinsicFromBL3(num_beams, num_targets, num_scans, data_split_with_ring, data_split_with_ring_cartesian, object_list)
    delta(num_beams).D_s = struct();
    delta(num_beams).D = struct();
    delta(num_beams).A_c = struct();%azimuth correction
    delta(num_beams).V_c = struct();%elevation correction
    delta(num_beams).H_oc = struct();
    delta(num_beams).V_oc = struct();

    %%
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
                    X = [X, data_split_with_ring_cartesian{t}(j).points];
                end
                [plane{t}, ~] = estimateNormal(opt.corners, X(1:3, :), 1.5);
            else
                plane{t}.centroid =  [object_list(t).centroid; 1];
                plane{t}.normals =  object_list(t).normal;
                plane{t}.unit_normals = object_list(t).normal/(norm(object_list(t).normal));
            end
        end

        opt.delta.D_s_init = 1;
        opt.delta.D_init = 0;
        opt.delta.A_c_init = 0;
        opt.delta.V_c_init = 0;%TODO: find a more reasonable initial guess
        opt.delta.H_oc_init = 0;
        opt.delta.V_oc_init = 0;

        [delta, ~, valid_targets] = estimateDeltaFromBL3(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets);
    end
end
