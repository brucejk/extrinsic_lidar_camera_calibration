function valid_targets = checkRingsCrossDataset(data, num_targets, ring, minimun_target_for_this_ring, minimun_point_on_a_target)
    % If not 'enough' data for a ring, don't try to calibrate it
    %
    % For enough data, by default we mean :
    % (1) on a target, a ring has to have a least 10 points to calibrate
    % (2) on the whole scene (containing many many targets), the ring has
    % to appear on targets at least 5 times.
    
    if ~exist('minimun_point_on_a_target', 'var') 
        minimun_point_on_a_target = 10;
    end
    
    if ~exist('minimun_target_for_this_ring', 'var') 
        minimun_target_for_this_ring = 1;
    end
    
    valid_targets.skip = false;
    valid_targets.valid = zeros(1, num_targets);
    valid_targets.num_points = zeros(1, num_targets);
    
    for target = 1:num_targets
        % minimun 10 points
        num_point_on_this_target = size(data{target}(ring).points, 2);
        valid_targets.num_points(1, target) = num_point_on_this_target;
        if  num_point_on_this_target >= minimun_point_on_a_target
            valid_targets.valid(1, target) = 1;
        end
    end

%     if valid_target_num < min(3, num_targets) % less than this, skip the ring
    if sum(valid_targets.valid) < minimun_target_for_this_ring
        valid_targets.skip = true;
        warning("Ring #%i has been skipped due to not enough valid targets %i:", ring, sum(valid_targets.valid));
    end
end