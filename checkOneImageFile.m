function checkOneImageFile(opts,filepath, name, mat_path, image_corner_name, scan_num, camera_corners)
    
    if isempty(camera_corners)
        warning("No image corners found, please click on the image. Follow top-left-right-bottom order")
    elseif size(camera_corners, 2) ~= opts.correspondance_per_pose
        warning("Corners found but the number of corners are not correct. Please click on the image again and follow top-left-right-bottom order")
    else
        corner_flag = 1;
        fprintf("%s loaded\n", image_corner_name)
    end
    if ~corner_flag
        img = loadBagFileImage(filepath, name, [], scan_num);
        camera_corners = clickImageGetCorners(img, image_corner_name);
        camera_corners = camera_corners(:, end-3:end);
        camera_corners = refineCameraCorners(camera_corners, img, "display", 1, 0);
        camera_corners = camera_corners(:, [2 1 3 4]);
        save(mat_path + image_corner_name, 'camera_corners')
        fprintf("%s saved/n", image_corner_name)
    end
end