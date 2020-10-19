function checkImageFiles(opts, mat_path, bag_files, scan_num)
    for bag = 1:size(bag_files, 1)
        name = convertCharsToStrings(bag_files(bag).name);
        filepath = convertCharsToStrings(bag_files(bag).folder) + "/";
        mat_name = convertCharsToStrings(bag_files(bag).name(1:strfind(name,'.')-1));
        mat_files = dir(fullfile(mat_path, mat_name + "*.mat"));
        
        for mat = 1:length(mat_files)
            mat_file = mat_files(mat).name;
            if contains(mat_file, "target", 'IgnoreCase', true) &&  ~contains(mat_file, "imgCorner", 'IgnoreCase', true)
                image_corner_name = strcat(mat_file(1:strfind(mat_file, ".")-1),  '-imgCorner.mat');
            else
                continue;
            end
             % camera conrers
            if isfile(mat_path + image_corner_name) && ~opts.reclick_iamge_corners
                load(mat_path + image_corner_name);
                checkOneImageFile(opts,filepath, name, mat_path, image_corner_name, scan_num, camera_corners)
            else
                camera_corners = [];
                checkOneImageFile(opts,filepath, name, mat_path, image_corner_name, scan_num, camera_corners)
            end
        end
    end
end