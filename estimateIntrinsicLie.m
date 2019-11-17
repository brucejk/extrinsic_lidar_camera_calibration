clear; clc;
num_beams = 32; % config of lidar
 
% Single variable called point_cloud
% mat_file_path = '../repo/LiDARTag_data/velodyne_points-lab8-closer-big--2019-09-06-15-28.mat';
num_targets = 2;
mat_file_path = {'F:\Calibration\extrinsic_lidar_camera_calibration\LiDARTag_data\velodyne_points-lab3-closer-big--2019-09-06-08-38.mat',...
                 'F:\Calibration\extrinsic_lidar_camera_calibration\LiDARTag_data\velodyne_points-lab7-closer-big--2019-09-06-15-14.mat'};
             
pc = cell(1,num_targets);
for t = 1:num_targets
    pc{t} = loadPointCloud(mat_file_path{t});
end
num_scans = 1;
delta(num_beams).H = struct();
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
    data_split_with_ring = cell(1,num_targets);
    for t = 1:num_targets
        [plane{t}, ~] = estimateNormal(opt.corners, data{t}(1:3, :), 0.8051);
        data_split_with_ring{t} = splitPointsBasedOnRing(data{t}, num_beams);
    end
    
    opt.delta.rpy_init = [0 0 0];
    opt.delta.T_init = [0, 0, 0];
    opt.delta.H_init = eye(4);
    delta = estimateDelta(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets);
end
disp('done')



