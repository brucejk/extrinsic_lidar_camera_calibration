% clear, cl
% path.bag_file_path = '/home/brucebot/workspace/catkin/bagfiles/optimal_shape/Aug-08-2020/'; 
% path.mat_file_path = "/home/brucebot/workspace/lc-calibration/data/optimal_shape/Aug-08-2020/calibration/";
% opts.num_scans = 1;
% [quan_data, qual_data] = t_linkBagfilesWithMatfiles(path.bag_file_path, path.mat_file_path, opts)

function [quan_data, qual_data] = linkBagfilesWithMatfiles(bag_path, mat_path, opts, opt)
    bag_files = dir(fullfile(bag_path, "*.bag"));
    quan_data = struct();
    qual_data = struct();
    scan_num = 1;
    num_qual = 1;
    num_quan = 1;
    checkImageFiles(opts, mat_path, bag_files, scan_num);
    
    for bag = 1:size(bag_files, 1)
        name = convertCharsToStrings(bag_files(bag).name);
        filepath = convertCharsToStrings(bag_files(bag).folder) + "/";
        mat_name = convertCharsToStrings(bag_files(bag).name(1:strfind(name,'.')-1));
        mat_files = dir(fullfile(mat_path, mat_name + "*.mat"));
        quan_flag = 1;
        if isempty([mat_files(:).name])
            continue
        end
        if ~contains([mat_files(:).name], "target", 'IgnoreCase', true)
            quan_flag = 0;
        end
        num_targets = 0;
        
        for mat = 1:length(mat_files)
            mat_file = mat_files(mat).name;
            
            if contains(mat_file, "target", 'IgnoreCase', true) &&  ~contains(mat_file, "imgCorner", 'IgnoreCase', true)
                num_targets = num_targets + 1;
%                 name        
%                 mat_file
                quan_data(num_quan).bagfile = name;
                quan_data(num_quan).bag_path = filepath;
                quan_data(num_quan).mat_path = mat_path;
                quan_data(num_quan).scans(scan_num).image = loadBagFileImage(filepath, name, [], scan_num);
                pc = loadPointCloud(mat_path + mat_file);
                
                % camera conrers
                image_corner_name = strcat(mat_file(1:strfind(mat_file, ".")-1),  '-imgCorner.mat');
                if isfile(mat_path + image_corner_name) && ~opts.reclick_iamge_corners
                    load(mat_path + image_corner_name);
                    checkOneImageFile(opts,filepath, name, mat_path, image_corner_name, scan_num, camera_corners)
                else
                    camera_corners = [];
                    checkOneImageFile(opts,filepath, name, mat_path, image_corner_name, scan_num, camera_corners)
                end
                
                quan_data(num_quan).scans(scan_num).camera_target(num_targets).corners = camera_corners;
                quan_data(num_quan).scans(scan_num).camera_target(num_targets).four_corners_line = point2DToLineForDrawing(camera_corners);
                
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).mat_file = mat_file;
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.payload_points_i= getPayloadWithIntensity(pc, 1, opts.num_scans);
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.payload_points_h = getPayload(pc, 1, opts.num_scans);
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).target_scale = identifyOptimalTargetScaleFromName(mat_file);
                
                % lidar vertices
                tmp_name = string(mat_file(1:strfind(mat_file, ".")-1));
                if isfile(opts.path.save_load_all_vertices + tmp_name + "-EstimatedVertices.mat") && ~opts.optimize_all_corners
                    load(opts.path.save_load_all_vertices + tmp_name + "-EstimatedVertices.mat")
                    fprintf("%s loaded\n", tmp_name + "-EstimatedVertices.mat")
                else
                    output_t = esimateVertices(opts, opt, ...
                        mat_file, ...
                        quan_data(num_quan).scans(1).lidar_target(num_targets).L1_inspired.payload_points_i, ...
                        quan_data(num_quan).scans(1).lidar_target(num_targets).target_scale);
                    save(opts.path.save_load_all_vertices + tmp_name + "-EstimatedVertices.mat", 'output_t');
                end
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.H_LT = output_t.H_LT; 
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.cost = output_t.cost; 
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.box_width = output_t.box_width;
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.ideal_frame = output_t.ideal_frame; 
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.corners = output_t.projected_vertices;
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).L1_inspired.four_corners_line = point3DToLineForDrawing(output_t.projected_vertices, 2);
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).baseline.corners = output_t.RANSAC_vertices;
                quan_data(num_quan).scans(scan_num).lidar_target(num_targets).baseline.four_corners_line =  point3DToLineForDrawing(output_t.RANSAC_vertices, 2);
                
            elseif contains(mat_file, "full", 'IgnoreCase', true)
                if quan_flag
                    pc = loadPointCloud(mat_path + mat_file);
                    quan_data(num_quan).scans(scan_num).full_scan_i = getPayloadWithIntensity(pc, 1, opts.num_scans);
                    quan_data(num_quan).scans(scan_num).image = loadBagFileImage(filepath, name, [], scan_num);
                else
                    qual_data(num_qual).bag_file = name;
                    qual_data(num_qual).bag_path = filepath;
                    qual_data(num_qual).mat_path = mat_path;
                    qual_data(num_qual).mat_file = mat_file;
                    pc = loadPointCloud(mat_path + mat_file);
                    qual_data(num_qual).scans(scan_num).full_scan = getPayloadWithIntensity(pc, 1, opts.num_scans);
                    qual_data(num_qual).scans(scan_num).image = loadBagFileImage(filepath, name, [], scan_num);
                end
            end
        end
        
        if ~quan_flag
            qual_data(num_qual).scans(scan_num).num_targets = num_targets;
            num_qual = num_qual + 1;
        else
            quan_data(num_quan).scans(scan_num).num_targets = num_targets;
            num_quan = num_quan + 1;
        end
    end
    %% Refine vertices
    if isfile(opts.path.save_load_all_vertices + "RefinedEstimatedVertices.mat") && ~opts.refine_all_corners
        load(opts.path.save_load_all_vertices + "RefinedEstimatedVertices.mat")
    else
        validation_flag = 0;
        [training_t, ~, ~] = ...
                splitTrainingValidationDatasets(opts, validation_flag, quan_data);
        X_train = [training_t(:).X_train];
        Y_train = [training_t(:).Y_train];


        show_pnp_numerical_result = 1;
        opt.method = "LieGroup";
        switch opts.calibration_method
            case "4 points"
                disp('---------------------')
                disp(' Refinement Step...')
                disp('---------------------')
                for scene = 0: opts.num_refinement-1
                    disp('---------------------')
                    disp('--- SR_H_LC ...')
                    disp('---------------------')
                    % square with refinement
                    [SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimize4Points(opt, ...
                                                                                           X_train, Y_train, ... 
                                                                                           opt.intrinsic_matrix, show_pnp_numerical_result); 

                    % NOT square with refinement
                    if exist('X_not_square_refinement', 'var') && ~isempty(X_not_square_refinement)
                        [NSR_H_LC, NSR_P, NSR_opt_total_cost, NSR_final, NSR_All] = optimize4Points(opt, ...
                                                                                                    X_not_square_refinement, Y_base_line, ...
                                                                                                    opt.intrinsic_matrix, show_pnp_numerical_result); 

                    else
                        [NSR_H_LC, NSR_P, NSR_opt_total_cost, NSR_final, NSR_All] = deal(zeros(4,4), zeros(3,4), 0, 0, 0);
                    end

                    if scene == opts.num_refinement-1
                        break;
                    else
                        disp('------------------')
                        disp(' Refining SR_H_LC ...')
                        disp('------------------')
                        training_t = regulizedFineTuneLiDARTagPoseLieGroup(opt, training_t, SR_P, show_pnp_numerical_result);
                        X_train = [training_t(:).X_train_refined];

                        if exist('X_not_square_refinement', 'var') && ~isempty(X_not_square_refinement)
                            X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, Y_base_line,...
                                                                                X_base_line_edge_points, NSR_P, ...
                                                                                opts.correspondance_per_pose, show_pnp_numerical_result);
                        end
                    end
                end

            case "IoU"
                error("refinement for IoU hasn't been implenmented yet. Will come soon!")
        end
        save(opts.path.save_load_all_vertices + "RefinedEstimatedVertices.mat", 'training_t');
    end
    
    
    indx = 1;
    for i = 1:size(quan_data, 2)
        if any(ismember(i, opts.skip_indices))
            continue
        end
        for j = 1:quan_data(i).scans(scan_num).num_targets
            quan_data(i).scans(scan_num).lidar_target(j).L1_inspired.refined_corners = training_t(indx).X_train_refined;
            indx = indx + 1;
        end
    end
end