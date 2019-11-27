clear; clc;
num_beams = 32; % config of lidar
 
% Single variable called point_cloud
% mat_file_path = '../repo/LiDARTag_data/velodyne_points-lab8-closer-big--2019-09-06-15-28.mat';

path = "/media/brucebot/Linux_BH/";
mat_file_path = {path + 'velodyne_points-Intrinsic-LargeTag--2019-11-21-22-04.mat',...
                 path + 'velodyne_points-Intrinsic-SmallTag--2019-11-21-22-00.mat',...
                 path + 'velodyne_points-Intrinsic2-LargeTag--2019-11-22-13-35.mat',...
                 path + 'velodyne_points-Intrinsic2-SmallTag--2019-11-22-13-38.mat',...
                 path + 'velodyne_points-Intrinsic3-LargeTag--2019-11-22-14-21.mat',...
                 path + 'velodyne_points-Intrinsic3-SmallTag--2019-11-22-14-29.mat',...
                 path + 'velodyne_points-Intrinsic4-LargeTag--2019-11-22-22-51.mat',...
                 path + 'velodyne_points-Intrinsic4-SmallTag--2019-11-22-22-54.mat',...
                 path + 'velodyne_points-Intrinsic5-LargeTag--2019-11-22-23-02.mat',...
                 path + 'velodyne_points-Intrinsic5-SmallTag--2019-11-22-23-00.mat',...
                 path + 'velodyne_points-Intrinsic-further-LargeTag--2019-11-22-23-05.mat',...
                 path + 'velodyne_points-Intrinsic-further-SmallTag--2019-11-22-23-09.mat',...
                 path + 'velodyne_points-Intrinsic-further2-LargeTag--2019-11-22-23-15.mat',...
                 path + 'velodyne_points-Intrinsic-further2-SmallTag--2019-11-22-23-17.mat'};
% mat_file_path = {'/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/velodyne_points-wavefield3-big--2019-09-07-19-04.mat'};
num_targets = size(mat_file_path,2);                          
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
%         spherical_data{t} = Cartesian2Spherical(data{t});
%         data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, num_beams);
%         data_split_with_ring_raw{t} = splitPointsBasedOnRing(data{t}, num_beams);
    end
    
    opt.delta.rpy_init = [0 0 0];
    opt.delta.T_init = [0, 0, 0];
    opt.delta.H_init = eye(4);
    delta = estimateDelta(opt.delta, data_split_with_ring, plane, delta(num_beams), num_beams, num_targets);
end
disp('done')



