function data = updateDataRaw(num_beams, num_targets, data_split_with_ring, delta,valid_rings_targets, method)
%     data = data_split_with_ring;
    data = cell(size(data_split_with_ring));
    for i =1:num_targets
        for ring = 1: num_beams
%             if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
%                 warning("Ring %i has been skipped in update function", ring)
            if (isempty(data_split_with_ring{i}(ring).points))
                  data{i}(ring).points= [];
                  continue;
            else
                if (method == "BaseLine1")
                    data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "Basic");
                elseif (method == "Lie")
%                     fprintf("target_num: %i, ring_num: %i\n", i, ring)
                    data{i}(ring).points =  delta(ring).Affine * data_split_with_ring{i}(ring).points;   
                elseif (method == "BaseLine2")
                    %Note: if a ring should be skipped, there won't be
                    %any corresponding delta parameter, thus we should
                    %directly transfer the point from spherical to
                    %cartesian coordinates
                    if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points,"Basic");
                    else
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "BaseLine2", delta(ring));
                    end
                    
                elseif (method == "BaseLine3")
                    if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "Basic");
                    else
                        data{i}(ring).points = Spherical2Cartesian(data_split_with_ring{i}(ring).points, "BaseLine3", delta(ring));
                    end
                end            
            end
        end
    end
end