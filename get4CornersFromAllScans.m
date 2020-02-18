function bag_data = get4CornersFromAllScans(opt, opts, bag_data)
%     save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + path.event_name + '_all_scan_refined_corners.mat', 'refinement_scan_total');    
    num_scan = length(bag_data.scans(:));
    scans_t(num_scan).all = struct();
%         fprintf(['\n' repmat('.',1,num_scan) '\n\n']);
    
    parforProgress(num_scan);
    parfor scan = 1:num_scan
%         if scan == 1 || mod(scan, floor(num_scan/10)) == 0 || scan == num_scan
%             fprintf("----working on scan #%i/%i\n", scan, num_scan)
%         end
        if bag_data.scans(scan).num_tag.original == 0
            warning("skipped scan_num: %i in training set due to no tag detected.", scan)
            continue;
        end
        % passing scan number only for warning if any scan is skipped
        scans_t(scan).all = get4CornersFromAScan(opt, opts, bag_data.scans(scan));
        parforProgress;
%             fprintf('\b|\n');
    end
    parforProgress(0);

    bag_data.array.L1_inspired.training_x = []; % will be used for other functions
    bag_data.array.L1_inspired.target_H_LT = []; % will be used for other functions
    bag_data.array.L1_inspired.training_y = []; % will be used for other functions
    bag_data.array.L1_inspired.num_tag = []; % will be used for other functions
    bag_data.array.L1_inspired.tag_size = []; % will be used for other functions
    
    if isfield(scans_t(1).all.lidar_target(1), 'ransac_normal')
        flag_use_rn = 1;
        bag_data.array.ransac_normal.training_x = []; % will be used for other functions
        bag_data.array.ransac_normal.training_y = []; % will be used for other functions
        bag_data.array.ransac_normal.edges = []; % will be used for other functions
        bag_data.array.ransac_normal.num_tag = []; % will be used for other functions
        bag_data.array.ransac_normal.tag_size = []; % will be used for other functions
    else
        flag_use_rn = 0;   
    end
    
    for scan = 1:num_scan
        if size([scans_t(scan).all], 2) == 0
            continue;
        end
        bag_data.scans(scan) = scans_t(scan).all;
%         figure(100)
%         title("scan#" + num2str(scan))
%         cla
        for tag = 1:bag_data.scans(scan).num_tag.original
            if isempty(scans_t(scan).all.lidar_target(tag).L1_inspired.corners)
                warning("Scan#%i, tag#%i/%i has been skipped using L1_inspired method.", scan, tag, bag_data.scans(scan).num_tag.original)
                bag_data.scans(scan).num_tag.L1_inspired = bag_data.scans(scan).num_tag.L1_inspired - 1;
            else     
                bag_data.array.L1_inspired.training_x = [bag_data.array.L1_inspired.training_x, scans_t(scan).all.lidar_target(tag).L1_inspired.corners];
                bag_data.array.L1_inspired.target_H_LT = [bag_data.array.L1_inspired.target_H_LT, scans_t(scan).all.lidar_target(tag).L1_inspired.H_LT];
                bag_data.array.L1_inspired.training_y = [bag_data.array.L1_inspired.training_y, scans_t(scan).all.camera_target(tag).corners];
                bag_data.array.L1_inspired.tag_size = [bag_data.array.L1_inspired.tag_size, bag_data.scans(scan).lidar_target(tag).tag_size];
                bag_data.array.L1_inspired.num_tag = [bag_data.array.L1_inspired.num_tag, bag_data.scans(scan).num_tag];
            end
            
%             scatter3(scans_t(scan).all.lidar_target(tag).L1_inspired.corners(1,:),scans_t(scan).all.lidar_target(tag).L1_inspired.corners(2,:), scans_t(scan).all.lidar_target(tag).L1_inspired.corners(3,:) )
%             hold on
%             scatter3(bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(1, :), ...
%                      bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(2, :), ...
%                      bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(3, :))
            
            if flag_use_rn
                if isempty(scans_t(scan).all.lidar_target(tag).ransac_normal.corners)
                    bag_data.scans(scan).num_tag.ransac_normal = bag_data.scans(scan).num_tag.ransac_normal - 1;
                    warning("Scan#%i, tag#%i has been skipped using baseline method.", scan, tag, opts.base_line.edge_method)
                else
                    bag_data.array.ransac_normal.training_x = [bag_data.array.ransac_normal.training_x, scans_t(scan).all.lidar_target(tag).ransac_normal.corners];
                    bag_data.array.ransac_normal.edges = [bag_data.array.ransac_normal.edges, scans_t(scan).all.lidar_target(tag).ransac_normal.edges];
                    bag_data.array.ransac_normal.training_y = [bag_data.array.ransac_normal.training_y, scans_t(scan).all.camera_target(tag).ransac_normal.corners];
                    bag_data.array.ransac_normal.tag_size = [bag_data.array.ransac_normal.tag_size, bag_data.scans(scan).lidar_target(tag).tag_size];
                    bag_data.array.ransac_normal.num_tag = [bag_data.array.ransac_normal.num_tag, bag_data.scans(scan).num_tag];
                end
            end
        end
%         axis equal
%         hold off
%         pause
    end
    fprintf("Data set:\n --- L1-inspired: %i\n --- ransac_normal: %i\n", size([bag_data.array.L1_inspired.training_x], 2), size([bag_data.array.ransac_normal.training_x], 2))
end