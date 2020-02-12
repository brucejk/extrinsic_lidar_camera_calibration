%  path = '/home/chenxif/Documents/me590/Calibration/ExtrinsicCalibration/data/bagfile';
%  data = t_getSceneData(path,'*.bag',2,1)


function BagData = getSceneData(path, ext, scene, pair_num)

    files_from_a_folder = dir(fullfile(path, ext));
    if scene > length(files_from_a_folder)
        error("Wrong scene number: %i/%i, scene", scene ,length(files_from_a_folder))
    end
    selected_file = convertCharsToStrings(path) + "/" + convertCharsToStrings(files_from_a_folder(scene).name);
    RawData = getData(selected_file);
    BagData(scene).bagfile = path;
    BagData(scene).num_tag = length(RawData{pair_num}.Detections);
    BagData(scene).image = getImagefromStruct(RawData{pair_num}.Detections(1).Image); 
   
    for i =1:BagData(scene).num_tag
        BagData(scene).lidar_target(i).payload_points = getPointsfromStruct(RawData{pair_num}.Detections(i).LidartagDetection.Points); % [x;y;z;1]
        BagData(scene).lidar_target(i).tag_size = RawData{pair_num}.Detections(i).LidartagDetection.Size;
        
        camera_corners = [RawData{pair_num}.Detections(i).ApriltagDetection.OuterCorners.X
                          RawData{pair_num}.Detections(i).ApriltagDetection.OuterCorners.Y];
        camera_corners = sortrows(camera_corners', 2)';
        BagData(scene).camera_target(i).corners = [camera_corners;
                                                   1, 1, 1, 1];
    end

end




