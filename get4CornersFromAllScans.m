function bag_data = get4CornersFromAllScans(opt, bag_data)
    num_scan = length(bag_data.scans(:));
    scans_t(num_scan).all = struct();
    scans_t(num_scan).trainining_x = []; % lidar vertices
    scans_t(num_scan).trainining_y = []; % camera corners
    scans_t(num_scan).target_H_LT = [];
    scans_t(num_scan).num_tag_array = [];
    scans_t(num_scan).tag_size = [];
%         fprintf('Progress:\n');
%         fprintf(['\n' repmat('.',1,num_scan) '\n\n']);
    for scan = 1:num_scan
        fprintf("----working on scan #%i/%i\n", scan, num_scan)
        if bag_data.scans(scan).num_tag == 0
            warning("skipped scan_num: %i in training set due to no tag detected.", scan)
            continue;
        end

        % optimize lidar targets corners
        scans_t(scan).all = get4CornersFromAScan(opt, bag_data.scans(scan));
        scans_t(scan).trainining_x_ary = [scans_t(scan).trainining_x_ary, scans_t(scan).all.lidar_target.corners];
        scans_t(scan).trainining_y_ary = [scans_t(scan).trainining_y_ary, scans_t(scan).all.camera_target.corners];
        scans_t(scan).target_H_LT_ary = [scans_t(scan).target_H_LT_ary, scans_t(scan).all.lidar_target.H_LT];
        scans_t(scan).num_tag_ary = [scans_t(scan).num_tag_ary, scans_t(scan).all.num_tag];
        scans_t(scan).tag_size_ary = [scans_t(scan).tag_size_ary, scans_t(scan).all.lidar_target.tag_size];
        if length(scans_t(scan).trainining_x_ary) ~= lentgh(scans_t(scan).trainining_y_ary)
            warning("LiDAR vertices and camera corners do not match up!")
            warning("--- LiDAR: %i\n--- Camrea: %i", ...
                    length(scans_t(scan).trainining_x_ary), ...
                    lentgh(scans_t(scan).trainining_y_ary))
        end
%             fprintf('\b|\n');
    end
    
%     bag_data.trainining_x = [];
%     bag_data.trainining_y = [];
%     bag_data.training_H_LT = [];
%     bag_data.num_tag_array = [];
%     bag_data.tag_size
    for i = 1:num_scan
        if size([scans_t(i).all], 2) == 0
            continue;
        end
        bag_data.scans(i) = scans_t(i).all;
%         bag_data.trainining_x = 
    end
end