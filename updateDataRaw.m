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
%                     data{i}(ring).points(1,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*cos(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
%                     data{i}(ring).points(2,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*sin(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta).*sin(data_split_with_ring{i}(ring).points(3,:)+ delta(ring).phi);
%                     data{i}(ring).points(3,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D).*cos(data_split_with_ring{i}(ring).points(2,:)+delta(ring).theta);
                    data{i}(ring).points(1,:) = data_split_with_ring{i}(ring).points(1,:).*sin(data_split_with_ring{i}(ring).points(2,:)).*cos(data_split_with_ring{i}(ring).points(3,:));
                    data{i}(ring).points(2,:) = data_split_with_ring{i}(ring).points(1,:).*sin(data_split_with_ring{i}(ring).points(2,:)).*sin(data_split_with_ring{i}(ring).points(3,:));
                    data{i}(ring).points(3,:) = data_split_with_ring{i}(ring).points(1,:).*cos(data_split_with_ring{i}(ring).points(2,:));
                    data{i}(ring).points(4,:) = ones(1, size(data{i}(ring).points,2));
                elseif (method == "Lie")
%                     fprintf("target_num: %i, ring_num: %i\n", i, ring)
                    data{i}(ring).points =  delta(ring).Affine * data_split_with_ring{i}(ring).points;   
                elseif (method == "BaseLine2")
                    %Note: if a ring should be skipped, there won't be
                    %any corresponding delta parameter, thus we should
                    %directly transfer the point from spherical to
                    %cartesian coordinates
                    if (skipApplying(valid_rings_targets, ring, data_split_with_ring, i))
                        data{i}(ring).points(1,:) = data_split_with_ring{i}(ring).points(1,:).*sin(data_split_with_ring{i}(ring).points(2,:)).*cos(data_split_with_ring{i}(ring).points(3,:));
                        data{i}(ring).points(2,:) = data_split_with_ring{i}(ring).points(1,:).*sin(data_split_with_ring{i}(ring).points(2,:)).*sin(data_split_with_ring{i}(ring).points(3,:));
                        data{i}(ring).points(3,:) = data_split_with_ring{i}(ring).points(1,:).*cos(data_split_with_ring{i}(ring).points(2,:));
                        data{i}(ring).points(4,:) = ones(1, size(data{i}(ring).points,2));
                    else
                        dxy = (data_split_with_ring{i}(ring).points(1,:)*delta(ring).D_s + delta(ring).D) * delta(ring).S_vc -delta(ring).C_voc;
                        data{i}(ring).points(1,:) = dxy .* cos(data_split_with_ring{i}(ring).points(3,:)- delta(ring).A_c)- delta(ring).H_oc *sin(data_split_with_ring{i}(ring).points(3,:)- delta(ring).A_c);
                        data{i}(ring).points(2,:) = dxy .* sin(data_split_with_ring{i}(ring).points(3,:)- delta(ring).A_c)+ delta(ring).H_oc *cos(data_split_with_ring{i}(ring).points(3,:)- delta(ring).A_c);
                        data{i}(ring).points(3,:) = (data_split_with_ring{i}(ring).points(1,:)+ delta(ring).D)*delta(ring).C_vc + delta(ring).S_voc;
                        data{i}(ring).points(4,:) = ones(1, size(data{i}(ring).points,2));
                    end
                end            
            end
        end
    end
end