function cartesian_data = Spherical2Cartesian(data)
    if(isempty(data))
        warning("empty input into the Spherical2Cartesian function");
    end
    cartesian_data = data;
    cartesian_data(1,:) = data(1,:).*sin(data(2,:)).*cos(data(3,:));
    cartesian_data(2,:) = data(1,:).*sin(data(2,:)).*sin(data(3,:));
    cartesian_data(3,:) = data(1,:).*cos(data(2,:));
end