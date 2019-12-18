function  estimateIntrinsicSpherical(mat_file_path)
    num_beams = 32;
    num_targets = length(mat_file_path);
    pc = cell(1,num_targets);
    for t = 1:num_targets
        pc{t} = loadPointCloud(mat_file_path{t});
    end
    num_scans = 1;
    delta(num_beams).D = struct();
    delta(num_beams).theta = struct();
    delta(num_beams).phi = struct();
    %%
    for i = 1: num_scans
        scans = 1;
        data = cell(1,num_targets);% XYZIR 
        for t = 1:num_targets
            data{t} = getPayload(pc{t}, i , 1);
        end
        % Step 2: Calculate 'ground truth' points by projecting the angle onto the
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
        spherical_data = cell(1,num_targets);
        data_split_with_ring = cell(1,num_targets);
        data_split_with_ring_raw = cell(1,num_targets);
        for t = 1:num_targets
            [plane{t}, ~] = estimateNormal(opt.corners, data{t}(1:3, :), 0.8051);
            spherical_data{t} = Cartesian2Spherical(data{t});
            data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, num_beams);
            data_split_with_ring_raw{t} = splitPointsBasedOnRing(data{t}, num_beams);
        end

        opt.delta.D_corr_init = 0;
        opt.delta.theta_corr_init = 0;
        opt.delta.phi_corr_init = 0;
        delta = estimateDeltaSpherical(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets);
    end
    disp('done')
    plotSanityCheckSpherical(num_targets, plane,data_split_with_ring, data_split_with_ring_raw, delta);
end
