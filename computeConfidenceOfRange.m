function confidence_of_range = computeConfidenceOfRange(P, bag_data, validation_indices, method, refinement)
    num_senes = length(validation_indices);
    
    for scene = 1:num_senes % which dataset
        current_index = validation_indices(scene);
        num_scan = length(bag_data(current_index).scans(:));
        confidence_of_range(scene).bagfile = bag_data(current_index).bagfile;
        
        for scan_num = 1:num_scan
            num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
            
            for tag_num = 1:num_tag % which tag in this dataset
                % refinement and no-refinement should be the same
                if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners) 
                    continue
                else
                    if strcmpi(refinement, 'no_refinement')
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                        H_LT = bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).H_LT;
                        
                    else
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_corners];
                        H_LT = bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_H_LC;
                    end
                    num_corners = size(H_LT, 2) / 4;
                    
                    current_corners_Y = [bag_data(current_index).scans(scan_num).camera_target(tag_num).(method).corners];
                    squared_sum = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                    
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).RMSE = sqrt(squared_sum / num_corners);
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).distance  = norm(H_LT(1:3, 4));
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).rpy = rad2deg(rotm2eul(H_LT(1:3, 1:3), 'xyz'));
                end
            end
        end
    end
end