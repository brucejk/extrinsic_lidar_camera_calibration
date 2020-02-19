function confidence_of_range = computeConfidenceOfRange(P, bag_data, indices, method, refinement, fig_title)
    num_senes = length(indices);
    title_txt = fig_title;
    for scene = 1:num_senes % which dataset
        current_index = indices(scene);
        num_scan = length(bag_data(current_index).scans(:));

        for scan_num = 1:num_scan
            num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
            for tag_num = 1:num_tag % which tag in this dataset
                % refinement and no-refinement should be the same
                if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners) 
                    continue
                else
                    if strcmpi(refinement, 'no_refinement')
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                    else
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_corners];
                    end
                    current_corners_Y = [bag_data(current_index).scans(scan_num).camera_target(tag_num).(method).corners];
                    scan_cost = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                    
                    [centroid, normals] = computeCentroidAndNormals(current_corners_X);
                    dist_to_lidar = norm(centroid);
                end
            end
            title_txt = title_txt + " - ccene: " + bag_data(current_index).bagfile;
        end
    end
    title(title_txt)
end