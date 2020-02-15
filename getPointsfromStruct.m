function points = getPointsfromStruct(pointStruct)
    
    len = length(pointStruct.Fields);
    for i =1 : len
        pointField = rosmessage('sensor_msgs/PointField');
        pointField.Name = pointStruct.Fields(i).Name;
        pointField.Offset = pointStruct.Fields(i).Offset;
        pointField.Datatype = pointStruct.Fields(i).Datatype;
        pointField.Count = pointStruct.Fields(i).Count;
        pointFields(i) = pointField;
    end
    pointMsg = rosmessage('sensor_msgs/PointCloud2');
    pointMsg.Height = pointStruct.Height;
    pointMsg.Width = pointStruct.Width;
    pointMsg.PointStep = pointStruct.PointStep;
    pointMsg.RowStep = pointStruct.RowStep;
    pointMsg.Data = pointStruct.Data;
    pointMsg.Fields = pointFields;
    points = double(readXYZ(pointMsg));
    points = [points'; ones(1,size(points,1))];
end