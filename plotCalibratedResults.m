function plotCalibratedResults(num_targets, plane, data_split_with_ring, data, method)
    if (method == "Spherical")
        plotSanityCheckSpherical(num_targets, plane, data_split_with_ring, data); 
    elseif (method == "Lie")
        plotSanityCheckLie(num_targets, plane, data, data_split_with_ring);             
    end      
end