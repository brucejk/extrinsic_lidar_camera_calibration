% path = '/home/chenxif/Documents/me590/Calibration/ExtrinsicCalibration/data/bagfile';
% clc, clear
% path = 'moving_bags/';
% data = t_getSceneData(path,'*.bag', 3)
% data = t_getSceneData(path,'*.bag')
% disp("done")

function BagData = getSceneData(path, ext, scene, pair_num)

    files_from_a_folder = dir(fullfile(path, ext));
    if exist('scene', 'var')
        if scene > length(files_from_a_folder)
            error("Wrong scene number: %i/%i, scene", scene ,length(files_from_a_folder))
        else
            start_scene = scene;
            num_scene = scene;
        end
    else 
        start_scene = 1;
        num_scene = length(files_from_a_folder);
    end
    
    
    for scene = start_scene:num_scene
        selected_file = convertCharsToStrings(path) + convertCharsToStrings(files_from_a_folder(scene).name);
        RawData = getData(selected_file);
        BagData(scene).meta = files_from_a_folder(scene);
        BagData(scene).bagfile = convertCharsToStrings(files_from_a_folder(scene).name);
        BagData(scene).array = [];

        if exist('pair_num', 'var')
            start_scan = pair_num;
            num_scan = pair_num;
        else
            start_scan = 1;
            num_scan = size(RawData, 1);
        end

        
        % prepare for parfor loop
        scans(num_scan).num_tag = [];
        scans(num_scan).image = [];
        scans(num_scan).lidar_target = [];
        scans(num_scan).camera_target = [];

        parfor scan = start_scan : num_scan
%             scan
            scans(scan).num_tag = length(RawData{scan}.Detections);
            if scans(scan).num_tag == 0
                continue
            end
            scans(scan).image = getImagefromStruct(RawData{scan}.Detections(1).Image); 

            for i =1:scans(scan).num_tag
                scans(scan).lidar_target(i).payload_points = getPointsfromStruct(RawData{scan}.Detections(i).LidartagDetection.Points); % [x;y;z;1]
                scans(scan).lidar_target(i).tag_size = RawData{scan}.Detections(i).LidartagDetection.Size;

                camera_corners = [RawData{scan}.Detections(i).ApriltagDetection.OuterCorners.X
                                  RawData{scan}.Detections(i).ApriltagDetection.OuterCorners.Y];
                camera_corners = sortrows(camera_corners', 2)';
                scans(scan).camera_target(i).corners = [camera_corners;
                                                           1, 1, 1, 1];
            end
        end
        
        BagData(scene).scans = scans;
        clear scans;
    end
end




