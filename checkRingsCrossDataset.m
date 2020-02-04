% If not enough data for a ring, don't try to calibrate it
function skip = checkRingsCrossDataset(data, num_targets, ring)
    valid_target_num = num_targets;
    for target = 1:num_targets
        if size(data{target}(ring).points, 2) == 0
%             delta(ring).H = eye(4);
%             delta(ring).Affine = eye(4);
            valid_target_num = valid_target_num -1;
        end
    end

%     if valid_target_num < min(3, num_targets) % less than this, skip the ring
    if valid_target_num < num_targets
        skip = true;
    else
        skip = false;
    end
end