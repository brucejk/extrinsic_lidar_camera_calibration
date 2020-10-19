function [training_t, validation_t, baseline_t] = ...
            splitTrainingValidationDatasets(indices, validation_flag, quan_data)
    num_datasets = size(quan_data, 2);
    if ~isfield(indices, 'bag_training_indices')
        indices.bag_training_indices = linspace(1, num_datasets, num_datasets);
    end
    
    if ~isfield(indices, 'bag_validation_indices')
        indices.bag_validation_indices = 0;
    end
    
    if ~isfield(indices, 'bag_chosen_indices')
        indices.bag_chosen_indices = linspace(1, num_datasets, num_datasets);
    end
    
    if ~isfield(indices, 'bag_with_tag_list')
        indices.bag_with_tag_list = [quan_data(:).bagfile];
    end    
    training_t(length(indices.bag_training_indices)) = struct();
    validation_t(length(indices.bag_validation_indices)) = struct();
    baseline_t(length(indices.bag_training_indices)) = struct();
    training_index = 1;
    validation_index = 1;

    
    for k = 1:length(indices.bag_chosen_indices)
        current_index = indices.bag_chosen_indices(k);
        
        % skip undesire index
        if any(ismember(current_index, indices.skip_indices))
            continue
        end
        
        % if don't want to get validation set, skip
        % everything else but the traing set
        if ~validation_flag
            if ~any(ismember(indices.bag_training_indices, current_index))
                continue;
            end
        end

        if any(ismember(indices.bag_training_indices, current_index))
            %% training set
            for target = 1:quan_data(current_index).scans(1).num_targets
                % matfile name
                training_t(training_index).mat_file = quan_data(current_index).scans(1).lidar_target(target).mat_file;
                
                % target scale
                training_t(training_index).target_scale = quan_data(current_index).scans(1).lidar_target(target).target_scale;
                
                % ideal shape
                training_t(training_index).ideal_frame = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.ideal_frame;
                
                % box width
                training_t(training_index).box_width = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.box_width;
                
                % H_LT
                training_t(training_index).H_LT = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.H_LT;
                
                % target points
                training_t(training_index).payload_points_h = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.payload_points_h;
                
                % 4 x M*i, M is correspondance per scan, i is scan
                training_t(training_index).X_train = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.corners; 

                % 3 x M*i, M is correspondance per image, i is image
                training_t(training_index).Y_train = quan_data(current_index).scans(1).camera_target(target).corners; 
                
                % baseline
                training_t(training_index).baseline_corners = convertToHomogeneousCoord(quan_data(current_index).scans(1).lidar_target(target).baseline.corners);

                training_index = training_index + 1;
                
                fprintf("--- Got training set: %s\n\n", indices.bag_with_tag_list(current_index))
            end

        else 
            %% validation set
            for target = 1:quan_data(current_index).scans(1).num_targets
                 % matfile name
                validation_t(training_index).mat_file = quan_data(current_index).scans(1).lidar_target(target).mat_file;
                
                % target scale
                validation_t(validation_index).target_scale = quan_data(current_index).scans(1).lidar_target(target).target_scale;
                
                % ideal shape
                validation_t(training_index).ideal_frame = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.ideal_frame;
                
                % box width
                validation_t(training_index).box_width = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.box_width;
                
                % H_LT
                validation_t(validation_index).H_LT = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.H_LT;
                
                % target points
                validation_t(validation_index).payload_points_h = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.payload_points_h;
                
                % 4 x M*i, M is correspondance per scan, i is scan
                validation_t(validation_index).X_train = quan_data(current_index).scans(1).lidar_target(target).L1_inspired.corners; 

                % 3 x M*i, M is correspondance per image, i is image
                validation_t(validation_index).Y_train = quan_data(current_index).scans(1).camera_target(target).corners; 
                
                % baseline
                validation_t(training_index).baseline_corners = convertToHomogeneousCoord(quan_data(current_index).scans(1).lidar_target(target).baseline.corners);

                validation_index = validation_index + 1;
            end
            fprintf("--- Got validation set: %s\n\n", indices.bag_with_tag_list(current_index))
        end
    end
end