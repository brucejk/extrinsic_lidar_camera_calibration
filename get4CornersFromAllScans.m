function bag_data = get4CornersFromAllScans(opt, bag_data)
    num_scan = length(bag_data.scans(:));
    scans_t(num_scan).all = struct();
%         fprintf('Progress:\n');
%         fprintf(['\n' repmat('.',1,num_scan) '\n\n']);
    parfor scan = 1:num_scan
        fprintf("----working on scan #%i/%i\n", scan, num_scan)
        if bag_data.scans(scan).num_tag == 0
            warning("skipped scan_num: %i in training set due to no tag detected.", scan)
            continue;
        end

        % optimize lidar targets corners
        scans_t(scan).all = get4CornersFromAScan(opt, bag_data.scans(scan));

%             fprintf('\b|\n');
    end
    
    for scan = 1:num_scan
        if size([scans_t(scan).all], 2) == 0
            continue;
        end
        bag_data.scans(scan) = scans_t(scan).all;
        
        bag_data.training_x_ary = [bag_data.training_x_ary, scans_t(scan).all.lidar_target.corners];
        bag_data.tag_size_ary = [bag_data.tag_size_ary, scans_t(scan).all.lidar_target.tag_size];
        bag_data.target_H_LT_ary = [bag_data.target_H_LT_ary, scans_t(scan).all.lidar_target.H_LT];
        bag_data.training_y_ary = [bag_data.training_y_ary, scans_t(scan).all.camera_target.corners];
        bag_data.num_tag_ary = [bag_data.num_tag_ary, scans_t(scan).all.num_tag];
        
        
        if length([scans_t(scan).all.lidar_target.corners]) ~= length([scans_t(scan).all.camera_target.corners])
            warning("LiDAR vertices and camera corners do not match up at scan #%i!", scan)
            warning("--- (LiDAR, Camrea): (%i, %i)", ...
                    length([scans_t(scan).all.lidar_target.corners]), ...
                    length([scans_t(scan).all.camera_target.corners]))
        end
%         bag_data.trainining_x = 
    end
end