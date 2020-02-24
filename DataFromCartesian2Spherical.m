function spherical = DataFromCartesian2Spherical(num_beams, num_targets, cartesian)
    spherical = cell(size(cartesian));    
    for i =1:num_targets
        for ring = 1: num_beams
            if(isempty(cartesian{i}(ring).points))
%                 warning("empty input into the DataFromCartesian2Spherical function, skipped!");
                continue;
            end
            %D = x^2 + y^2 + z^2;
            %elevation_angle = arccos(z/D)
            %azimuth_angle = arctan(y/x)
            spherical{i}(ring).points(1,:) = sqrt(cartesian{i}(ring).points(1,:).^2 + cartesian{i}(ring).points(2,:).^2 + cartesian{i}(ring).points(3,:).^2);
            spherical{i}(ring).points(2,:) = acos(cartesian{i}(ring).points(3,:)./ spherical{i}(ring).points(1,:));
            spherical{i}(ring).points(3,:) = atan2(cartesian{i}(ring).points(2,:), cartesian{i}(ring).points(1,:));            
        end
    end
end