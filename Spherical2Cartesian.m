function Cartesianpoints = Spherical2Cartesian(spherical_data, model, delta)
    Cartesianpoints = zeros(size(spherical_data));
    if model == "Basic"
        Cartesianpoints(1,:) = spherical_data(1,:).*sin(spherical_data(2,:)).*cos(spherical_data(3,:));
        Cartesianpoints(2,:) = spherical_data(1,:).*sin(spherical_data(2,:)).*sin(spherical_data(3,:));
        Cartesianpoints(3,:) = spherical_data(1,:).*cos(spherical_data(2,:));
        Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    elseif model == "BaseLine2"
        dxy = (spherical_data(1,:)*delta.D_s + delta(ring).D) * delta.S_vc -delta.C_voc;
        Cartesianpoints(1,:) = dxy .* cos(spherical_data(3,:)- delta.A_c)- delta.H_oc *sin(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(2,:) = dxy .* sin(spherical_data(3,:)- delta.A_c)+ delta.H_oc *cos(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(3,:) = (spherical_data(1,:)+ delta.D)*delta.C_vc + delta.S_voc;
        Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    elseif model == "BaseLine3"
        dxy = (spherical_data(1,:)*delta.D_s + delta.D).*sin(delta.V_c);
        Cartesianpoints(1,:) = dxy .* cos(spherical_data(3,:)- delta.A_c)- delta.H_oc *sin(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(2,:) = dxy .* sin(spherical_data(3,:)- delta.A_c)+ delta.H_oc *cos(spherical_data(3,:)- delta.A_c);
        Cartesianpoints(3,:) = (spherical_data(1,:)*delta.D_s+ delta.D).*cos(delta.V_c) + delta.V_oc;
        Cartesianpoints(4,:) = ones(1, size(spherical_data,2));
    end
end