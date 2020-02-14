function img = getImagefromStruct(imgStruct)
    imgMsg = rosmessage('sensor_msgs/Image');
    imgMsg.Encoding = imgStruct.Encoding;
    imgMsg.Height = imgStruct.Height;
    imgMsg.Width = imgStruct.Width;
    imgMsg.Data = imgStruct.Data;
    img.image = readImage(imgMsg);
    img.height = imgStruct.Height;
    img.width = imgStruct.Width;
end