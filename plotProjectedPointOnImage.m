function plotProjectedPointOnImage(P, bag_data, indices, fig_handles, method, fig_title, show_training_results, pause_each_scan, start_scan)

if ~exist('start_scan', 'var')
    start_scan = 1;
end
num_senes = length(indices);

for scene = 1:num_senes % which dataset
    current_index = indices(scene);
    num_scan = length(bag_data(current_index).scans(:));
    start_scan = min(start_scan, num_scan);
    
    for scan_num = start_scan:num_scan

        imshow(bag_data(current_index).scans(scan_num).image, 'Parent', fig_handles(scene));
        num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
        for tag_num = 1:num_tag % which tag in this dataset
            if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners)
                continue
            else
                current_corners = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                current_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).payload_points_h];
        %         if show_baseline_results
        %             projectBackToImage(training_img_fig_handles(i), NSNR_P, current_corners_SR, 5, 'kd', "training_SR", "not display", "Not-Clean");
        %         end

                projectBackToImage(fig_handles(scene), P, current_corners, 50, 'g*', fig_title, "not display", "Not-Clean");
                projectBackToImage(fig_handles(scene), P, current_X, 3, 'r.', fig_title, "not display", "Not-Clean"); 
                showLinedAprilTag(fig_handles(scene), bag_data(current_index).scans(scan_num).camera_target(tag_num), show_training_results);
            end
        end
        title_txt = fig_title + " [Scene: " + bag_data(current_index).bagfile + " at scan #" + num2str(scan_num) + "/" + num2str(num_scan) + "]";
        drawnow
        title(fig_handles(scene), title_txt)
        if pause_each_scan
            pause
        else
            break;
        end
    end
end