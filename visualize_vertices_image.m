clear, clc

path = "/home/brucebot/workspace/lc-calibration/extrinsic_lidar_camera_calibration/TRO/10-Aug-2020 20:51:36/";
load(path + "quan_data.mat");
load(path + "X_train.mat");
load(path + "Y.mat");
load(path + "save_validation.mat");
load(path + "saved_chosen_indices.mat")

%%
vertices_per_target = 4;
[axes_h, fig_h] = createFigHandleWithNumber(3, 1000, "verify corners", 1, 1);



%% ALL without vertices order
for i = 1:size(quan_data, 2)
    i
    current_index = i;
    cla(axes_h(2))
    plotOriginalAxisWithText(axes_h(3), "LiDAR", eye(4,4), 0.2, 'k')
    showBagFileImage(fig_h(1), quan_data(current_index).bag_path, quan_data(current_index).bagfile);
    num_target = quan_data(current_index).scans(1).num_targets;
    
    for j = 1:num_target
        cla(axes_h(2))
        bag_file = quan_data(current_index).bagfile;
        mat_name = quan_data(current_index).scans(1).lidar_target(j).mat_file;
        img_corner_name = string(mat_name(1:strfind(mat_name, ".")-1)) + "-imgCorner.mat"
        current_vertices = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.corners;
        current_pc = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.payload_points_i;
        current_corners =  quan_data(current_index).scans(1).camera_target(j).corners;
        line = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.four_corners_line;
%         line = line(:, [1 2 3 6 6 4 5 7]);
        line = line(:, [1 2 6 4 1]);
        plot3(axes_h(2), line(1, :), line(2, :), line(3, :));

        viewCurrentPlot(axes_h(1), bag_file)
        viewCurrentPlot(axes_h(2), mat_name)
        viewCurrentPlot(axes_h(3), "LiDAR Vertices with origin")
        
        h1 = scatter(axes_h(1), current_corners(1, :),  current_corners(2, :), 'fill', 'bo');
        h2 = scatter3(axes_h(2), current_vertices(1, :), current_vertices(2, :), current_vertices(3, :), 'fill', 'ro');
        h3 = scatter3(axes_h(2), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');


        h4 = scatter3(axes_h(3), current_vertices(1, :), current_vertices(2, :), current_vertices(3, :), 'fill', 'ro');
        h5 = scatter3(axes_h(3), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
        disp("press enter to continue")
        pause
    end
end
%% ALL with vertices order
indicator = 1;
for i = 1:size(quan_data, 2)
    
    current_index = i;
    if any(ismember(current_index, indices.skip_indices))
            continue
    end
    cla(axes_h(2))
    plotOriginalAxisWithText(axes_h(3), "LiDAR", eye(4,4), 0.2, 'k')
    showBagFileImage(fig_h(1), quan_data(current_index).bag_path, quan_data(current_index).bagfile);
    num_target = quan_data(current_index).scans(1).num_targets;
    
    for j = 1:num_target
        bag_file = quan_data(current_index).bagfile;
        mat_name = quan_data(current_index).scans(1).lidar_target(j).mat_file;
        img_corner_name = string(mat_name(1:strfind(mat_name, ".")-1)) + "-imgCorner.mat"
        current_vertices = X_train(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        current_pc = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.payload_points_i;
        current_corners =  Y_train(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        line = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.four_corners_line;
%         line = line(:, [1 2 3 6 6 4 5 7]);
        line = line(:, [1 2 6 4 1]);
        plot3(axes_h(2), line(1, :), line(2, :), line(3, :));
        title(axes_h(2), bag_file)
        
        indicator = indicator + 1;
        
        for k = 1:vertices_per_target
            h1 = scatter(axes_h(1), current_corners(1, k),  current_corners(2, k), 'fill', 'bo');
            h2 = scatter3(axes_h(2), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h3 = scatter3(axes_h(2), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            h4 = scatter3(axes_h(3), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h5 = scatter3(axes_h(3), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            disp("press enter to continue")
            pause
        end
    end
end


%% training set with vertices order
indicator = 1;
for i = 1:length(bag_training_indices)
    current_index = bag_training_indices(i);
    cla(axes_h(2))
    plotOriginalAxisWithText(axes_h(3), "LiDAR", eye(4,4), 0.2, 'k')
    showBagFileImage(fig_h(1), quan_data(current_index).bag_path, quan_data(current_index).bagfile);
    num_target = quan_data(current_index).scans(1).num_targets;
    
    for j = 1:num_target
        bag_file = quan_data(current_index).bagfile;
        mat_name = quan_data(current_index).scans(1).lidar_target(j).mat_file;
        img_corner_name = string(mat_name(1:strfind(mat_name, ".")-1)) + "-imgCorner.mat"
        current_vertices = X_train(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        current_pc = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.payload_points_i;
        current_corners =  Y_train(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        line = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.four_corners_line;
%         line = line(:, [1 2 3 6 6 4 5 7]);
        line = line(:, [1 2 6 4 1]);
        plot3(axes_h(2), line(1, :), line(2, :), line(3, :));
        title(axes_h(2), bag_file)
        
        indicator = indicator + 1;
        
        for k = 1:vertices_per_target
            h1 = scatter(axes_h(1), current_corners(1, k),  current_corners(2, k), 'fill', 'bo');
            h2 = scatter3(axes_h(2), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h3 = scatter3(axes_h(2), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            h4 = scatter3(axes_h(3), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h5 = scatter3(axes_h(3), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            disp("press enter to continue")
            pause
        end
    end
end

%% validation set with vertices order
indicator = 1;
for i = 1:length(bag_validation_indices)
    current_index = bag_validation_indices(i);
    cla(axes_h(2))
    plotOriginalAxisWithText(axes_h(2), "LiDAR", eye(4,4), 0.2, 'k')
    showBagFileImage(fig_h(1), quan_data(current_index).bag_path, quan_data(current_index).bagfile);
    num_target = quan_data(current_index).scans(1).num_targets;
    
    for j = 1:num_target
        mat_name = quan_data(current_index).scans(1).lidar_target(j).mat_file;
        img_corner_name = string(mat_name(1:strfind(mat_name, ".")-1)) + "-imgCorner.mat"
        current_vertices = X_validation(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        current_corners =  Y_validation(:, (vertices_per_target*indicator-vertices_per_target+1):vertices_per_target*indicator);
        
        line = quan_data(current_index).scans(1).lidar_target(j).L1_inspired.four_corners_line;
%         line = line(:, [1 2 3 6 6 4 5 7]);
        line = line(:, [1 2 6 4 1]);
        plot3(axes_h(2), line(1, :), line(2, :), line(3, :));
        indicator = indicator + 1;
        
        for k = 1:vertices_per_target
            h1 = scatter(axes_h(1), current_corners(1, k),  current_corners(2, k), 'fill', 'bo');
            h2 = scatter3(axes_h(2), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h3 = scatter3(axes_h(2), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            h4 = scatter3(axes_h(3), current_vertices(1, k), current_vertices(2, k), current_vertices(3, k), 'fill', 'ro');
            h5 = scatter3(axes_h(3), current_pc(1, :), current_pc(2, :), current_pc(3, :), 'g.');
            
            
            disp("press enter to continue")
            pause
        end
    end
end