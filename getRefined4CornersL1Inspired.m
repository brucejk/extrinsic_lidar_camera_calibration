function bag_data = getRefined4CornersL1Inspired(opt, opts, bag_data, tag_num)
    X_train = [bag_data.lidar_target(tag_num).L1_inspired.corners];
    Y_train = [bag_data.camera_target(tag_num).corners];
    H_LT = [bag_data.lidar_target(tag_num).L1_inspired.H_LT];
    tag_size = bag_data.lidar_target(tag_num).tag_size;
    show_pnp_numerical_result = 0;
    
    for i = 1: opts.num_refinement
        if opts.calibration_method == "4 points"
            [SR_H_LC, SR_P, ~, ~, ~] = optimize4Points(opt.H_LC.rpy_init, ...
                                           X_train, Y_train, ... 
                                           opt.intrinsic_matrix, ...
                                           show_pnp_numerical_result);
        elseif opts.calibration_method == "IoU"
            [SR_H_LC, SR_P, ~, ~, ~] = optimizeIoU(opt.H_LC.rpy_init, ...
                                         X_train, Y_train, ... 
                                         opt.intrinsic_matrix, ...
                                         show_pnp_numerical_result); % square with refinement
        else
            error("This refinement method is not yet implemented: %s",...
                    opts.calibration_method);
        end

        if i == opts.num_refinement
            break;
        else
            X_train = regulizedFineTuneEachLiDARTagPose(tag_size, ...
                        X_train, Y_train, H_LT, SR_P, ...
                        show_pnp_numerical_result);
        end
    end

    [centroid, normals] = computeCentroidAndNormals(X_train);
    bag_data.lidar_target(tag_num).L1_inspired.refined_centroid = centroid;
    bag_data.lidar_target(tag_num).L1_inspired.refined_normals = normals;
    bag_data.lidar_target(tag_num).L1_inspired.refined_corners = X_train;
    bag_data.lidar_target(tag_num).L1_inspired.refined_H_LC = SR_H_LC;
    bag_data.lidar_target(tag_num).L1_inspired.refined_P = SR_P;
end