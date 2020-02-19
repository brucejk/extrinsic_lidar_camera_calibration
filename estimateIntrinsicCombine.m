%% Load datasets
clc; % DO NOT add 'clear' here

if exist('pc','var') &&  exist('data', 'var') && exist('mat_files', 'var')
% if exist('pc','var') 
    disp("Data have been loaded, it may take a while to reload them.")
    prompt = "Are you sure you want to reload them again? [Y] [N]";
    urs_ans = input(prompt, 's');
    if contains(urs_ans, 'N','IgnoreCase', true)
        disp('Reload datasets cancelled')
        return
    else
        clear
        disp("ALL varialbes cleared! ")
        disp("Keep reloading datasets...")
    end
end

% Single variable called point_cloud

% opts.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/slider/intrinsic_latest/data/";
% opts.path = "./data/";
opts.path = "..\intrinsic_lidar_calibration\data\";
opts.load_all = 1;

opts.show_results = 1;
opt_formulation = ["Lie","Spherical"]; % Lie or Spherical
opts.method = 1; % Lie; Spherical
opts.iterative = 1;

opts.num_beams = 32;
opts.num_scans = 1;
opts.num_iters = 10; % user defined iterations

% path = "/home/chenxif/Documents/me590/Calibration/IntrinsicCalibration/extracted_tags/";

disp("Loading names of data sets...")
if ~opts.load_all
    mat_files = struct('file_name', {opts.path+'velodyne_points-Intrinsic-LargeTag--2019-11-21-22-04.mat';...
                     opts.path+'velodyne_points-Intrinsic-SmallTag--2019-11-21-22-00.mat';...
                     opts.path+'velodyne_points-Intrinsic4-SmallTag--2019-11-22-22-54.mat';...
                     opts.path+'velodyne_points-Intrinsic5-LargeTag--2019-11-22-23-02.mat';...
                     opts.path+'velodyne_points-Intrinsic5-SmallTag--2019-11-22-23-00.mat';...
                     opts.path+'velodyne_points-Intrinsic-further-LargeTag--2019-11-22-23-05.mat';...
                     opts.path+'velodyne_points-Intrinsic-further-SmallTag--2019-11-22-23-09.mat';...
                     opts.path+'velodyne_points-Intrinsic-further2-LargeTag--2019-11-22-23-15.mat';...
                     opts.path+'velodyne_points-Intrinsic-further2-SmallTag--2019-11-22-23-17.mat';...
                     opts.path+'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat';...
                     opts.path+'velodyne_points-upper2-SmallTag--2019-12-05-20-16.mat';...
                     opts.path+'velodyne_points-upper3-SmallTag--2019-12-05-20-19.mat';...
                     opts.path+'velodyne_points-upper4-SmallTag--2019-12-05-20-22.mat';...
                     opts.path+'velodyne_points-upper5-SmallTag--2019-12-05-20-23.mat';...
                     opts.path+'velodyne_points-upper6-SmallTag--2019-12-05-20-26.mat';...
                     opts.path+'velodyne_points-upper7-SmallTag--2019-12-05-20-29.mat';...
                     opts.path+'velodyne_points-upper8-SmallTag--2019-12-05-20-29.mat'});
    for file = 1:length(mat_files)
        mat_files(file).tag_size = identifyTagSizeFromAName(mat_files(file).file_name);
    end
else
    mat_files = loadFilesFromAFolder(opts.path, '*Tag*.mat');
end
num_targets = length(mat_files);

disp("Loading point cloud from .mat files")
pc = struct('point_cloud', cell(1,num_targets));
for t = 1:num_targets
    pc(t).point_cloud = loadPointCloud(mat_files(t).file_name);
end

disp("Pre-processing payload points...")
data = struct('point_cloud', cell(1,num_targets), 'tag_size', cell(1,num_targets));% XYZIR 
for i = 1: opts.num_scans
    for t = 1:num_targets
        data(t).payload_points = getPayload(pc(t).point_cloud, i , opts.num_scans);
        data(t).tag_size = mat_files(t).tag_size;
    end
end
disp("Done loading data!")


%% Optimize intrinsic parameters
clc
% if ones want to re-run this process
opts.iterative = 1;
opts.method = 1; % Lie; Spherical
opts.num_iters = 20;

if (opt_formulation(opts.method) == "Lie")
    data_split_with_ring_cartesian = cell(1,num_targets);

    disp("Parsing data...")
    for t = 1:num_targets
        data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams);
    end 
    
    disp("Optimizing using Lie Group method...")
    if ~opts.iterative
       opts.num_iters = 1;
    end
    distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
    distance(opts.num_iters).ring(opts.num_beams) = struct();
    distance(opts.num_iters).mean = 0;
%     distance(opts.num_iters).std = 0;
    for k = 1: opts.num_iters
        fprintf("--- Working on %i/%i\n", k, opts.num_iters)
        [delta, plane, valid_rings_and_targets] = estimateIntrinsicLie(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring_cartesian);
        if k == 1
            distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 
        end
        % update the corrected points
        data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring_cartesian, delta, opt_formulation(opts.method));
        distance(k) = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets);
    end

elseif (opt_formulation(opts.method) == "Spherical")
    % preprocess the data
    spherical_data = cell(1,num_targets);
    data_split_with_ring = cell(1, num_targets);
    data_split_with_ring_cartesian = cell(1, num_targets);

    disp("Parsing data...")
    for t = 1:num_targets
        spherical_data{t} = Cartesian2Spherical(data(t).payload_points);
        data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, opts.num_beams);
        data_split_with_ring_cartesian{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams);
    end
    
    disp("Optimizing using a mechanical model...")
    if ~opts.iterative
       opts.num_iters = 1;
    end
    distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
    distance(opts.num_iters).ring(opts.num_beams) = struct(); 
    distance(opts.num_iters).mean = 0;
     % iteratively optimize the intrinsic parameters
    for k = 1: opts.num_iters
        fprintf("--- Working on %i/%i\n", k, opts.num_iters)
        [delta, plane, valid_rings_and_targets] = estimateIntrinsicFromMechanicalModel(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring, data_split_with_ring_cartesian);
        if k == 1
            distance_original = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 
        end
        
        % update the corrected points
        data_split_with_ring = updateDatacFromMechanicalModel(opts.num_beams, num_targets, data_split_with_ring, delta, valid_rings_and_targets);
        data_split_with_ring_cartesian = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring, delta, opt_formulation(opts.method));
        distance(k) = point2PlaneDistance(data_split_with_ring_cartesian, plane, opts.num_beams, num_targets); 
    end
end
disp('Done optimization')

%% Show graphical results
if opts.show_results
    disp("Now plotting....")
    plotCalibratedResults(num_targets, plane, data_split_with_ring_cartesian, data);
%     plotCalibratedResults(num_targets, plane, data_split_with_ring_cartesian, data, opt_formulation(opts.method));
    disp("Done plotting!")
end


%% show numerical results
disp("Showing numerical results...")
disp("Showing current estimate")
results = struct('ring', {distance(end).ring(:).ring}, ...
                 'num_points', {distance(end).ring(:).num_points}, ...
                 'mean_original', {distance_original.ring(:).mean}, ...
                 'mean_calibrated', {distance(end).ring(:).mean}, ...
                 'mean_diff', num2cell([distance_original.ring(:).mean] - [distance(end).ring(:).mean]), ...
                 'mean_diff_in_mm', num2cell(([distance_original.ring(:).mean] - [distance(end).ring(:).mean]) * 1e3), ...
                 'std_original', {distance_original.ring(:).std}, ...
                 'std_calibrated', {distance(end).ring(:).std}, ...
                 'std_diff', num2cell([distance_original.ring(:).std] - [distance(end).ring(:).std]), ...
                 'std_diff_in_mm', num2cell(([distance_original.ring(:).std] - [distance(end).ring(:).std])* 1e3));
struct2table(distance(end).ring(:))
disp("Showing comparison")
struct2table(results)

% check if ring mis-ordered
disp("If the rings are mis-ordered...")
[order mis_ordered_list] = checkRingOrder(data_split_with_ring_cartesian, delta, num_targets, opts.num_beams, opts);

disp("All processes has finished")