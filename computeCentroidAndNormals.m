function [centroid, normals] = computeCentroidAndNormals(corners)
    centroid = mean(corners(1:3,:), 2);
    normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
    normals = normals/(norm(normals));
    if normals(1) > 0
        normals = -normals;
    end
end