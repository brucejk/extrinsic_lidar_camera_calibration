function spherical_data = Cartesian2Spherical(data)
    if(isempty(data))
        warning("empty input into the Cartesian2Spherical function");
    end
    spherical_data = data;
    %D = x^2 + y^2 + z^2;
    %elevation_angle = arccos(z/D)
    %azimuth_angle = arctan(y/x)
    spherical_data(1,:) = sqrt(data(1,:).^2 + data(2,:).^2 + data(3,:).^2);
    spherical_data(2,:) = acos(data(3,:)./spherical_data(1,:));
    spherical_data(3,:) = atan2(data(2,:), data(1,:));
end