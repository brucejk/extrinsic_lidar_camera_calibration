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
opts.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/slider/intrinsic_latest/data/";
opts.path = "./data/";
opts.load_all = 1;

opts.show_results = 0;
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
% mat_file_path = {'/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/velodyne_points-wavefield3-big--2019-09-07-19-04.mat'};
num_targets = length(mat_files);

disp("Loading point cloud from .mat files")
pc = struct('point_cloud', cell(1,num_targets));
for t = 1:num_targets
    pc(t).point_cloud = loadPointCloud(mat_files(t).file_name);
end

disp("Pre-processing payload points...")
for i = 1: opts.num_scans
    data = struct('point_cloud', cell(1,num_targets), 'tag_size', cell(1,num_targets));% XYZIR 
    for t = 1:num_targets
        data(t).payload_points = getPayload(pc(t).point_cloud, i , 1);
        data(t).tag_size = mat_files(t).tag_size;
    end
end
disp("Done loading data!")
%% optimization intrinsic parameters
clc
% if ones want to re-run this process
opts.iterative = 1;
opts.method = 1; % Lie; Spherical


if (opt_formulation(opts.method) == "Lie")
    data_split_with_ring = cell(1,num_targets);

    disp("Parsing data...")
    for t = 1:num_targets
        data_split_with_ring{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams);
    end 
    
    disp("Optimizing using Lie Group method...")
    if ~opts.iterative
       opts.num_iters = 1;
    end
    distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
    distance(opts.num_iters).ring(opts.num_beams) = struct();
    for k = 1: opts.num_iters
        fprintf("--- Working on %i/%i\n", k, opts.num_iters)
        [delta, plane] = estimateIntrinsicLie(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring);
        if k == 1
            distance_original = point2PlaneDistance(data_split_with_ring, plane, opts.num_beams, num_targets); 
        end
        % update the corrected points
        data_split_with_ring = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring, delta, opt_formulation(opts.method));
        distance(k) = point2PlaneDistance(data_split_with_ring, plane, opts.num_beams, num_targets); 
    end

    disp('Done optimization')
    if opts.show_results
        disp("Now plotting....")
        plotSanityCheckLie(num_targets, plane, data, data_split_with_ring);
        disp("Done plotting!")
    end
    
elseif (opt_formulation(opts.method) == "Spherical")
    % preprocess the data
    spherical_data = cell(1,num_targets);
    data_split_with_ring = cell(1, num_targets);
    data_split_with_ring_raw = cell(1, num_targets);

    disp("Parsing data...")
    for t = 1:num_targets
        spherical_data{t} = Cartesian2Spherical(data(t).payload_points);
        data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, opts.num_beams);
        data_split_with_ring_raw{t} = splitPointsBasedOnRing(data(t).payload_points, opts.num_beams);
    end
    
    disp("Optimizing using a mechanical model...")
    if ~opts.iterative
       opts.num_iters = 1;
    end
    distance = []; % if re-run, it will show error of "Subscripted assignment between dissimilar structures"
    distance(opts.num_iters).ring(opts.num_beams) = struct(); 
    
     % iteratively optimize the intrinsic parameters
    for k = 1: opts.num_iters
        fprintf("--- Working on %i/%i\n", k, opts.num_iters)
        [delta, plane] = estimateIntrinsicFromMechanicalModel(opts.num_beams, num_targets, opts.num_scans, data_split_with_ring, data_split_with_ring_raw);
        if k == 1
            distance_original = point2PlaneDistance(data_split_with_ring, plane, opts.num_beams, num_targets); 
        end
        % update the corrected points
        data_split_with_ring = updateDatacFromMechanicalModel(opts.num_beams, num_targets, data_split_with_ring, delta);
        data_split_with_ring_raw = updateDataRaw(opts.num_beams, num_targets, data_split_with_ring, delta, opt_formulation(opts.method));
        distance(k) = point2PlaneDistance(data_split_with_ring_raw, plane, opts.num_beams, num_targets); 
    end
    disp('Done optimization')
    if opts.show_results
        disp("Now plotting....")
        plotSanityCheckSpherical(num_targets, plane, data_split_with_ring_raw, data);   
        disp("Done plotting!")
    end
end


% showing results
disp("Showing results...")
results = struct('ring', {distance(end).ring(:).ring}, ...
                 'num_points', {distance(end).ring(:).num_points}, ...
                 'mean_original', {distance_original.ring(:).mean}, ...
                 'std_original', {distance_original.ring(:).std}, ...
                 'mean_diff', num2cell([distance_original.ring(:).mean] - [distance(end).ring(:).mean]), ...
                 'std_diff', num2cell([distance_original.ring(:).std] - [distance(end).ring(:).std]), ...
                 'mean_diff_in_mm', num2cell(([distance_original.ring(:).mean] - [distance(end).ring(:).mean]) * 1e3), ...
                 'std_diff_in_mm', num2cell(([distance_original.ring(:).std] - [distance(end).ring(:).std])* 1e3));
struct2table(distance(end).ring(:))
struct2table(results)

%% check if ring mis-ordered
disp("If the rings are mis-ordered...")
[order mis_ordered_list] = checkRingOrder(data_split_with_ring, delta, num_targets, opts.num_beams, opts);

disp("All processes has finished")