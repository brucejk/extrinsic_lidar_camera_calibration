function data = updateDataRaw(num_beams, num_targets, data_split_with_ring, delta, method)
    data = data_split_with_ring;
    for i =1:num_targets
        for ring = 1: num_beams
            if (isempty(data_split_with_ring{i}(ring).points))
                  continue;
            else
                if (method == "Spherical")
                    data{i}(ring).points(1,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*cos(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
                    data{i}(ring).points(2,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*sin(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
                    data{i}(ring).points(3,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*cos(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta);
                elseif (method == "Lie")
                    data{i}(ring).points =  delta(ring).Affine * data_split_with_ring{i}(ring).points;              
                end            
            end
        end
    end
end