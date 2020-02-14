dataset.image.name = "the-wall-2.bag";
dataset.image.path = "/home/brucebot/workspace/griztag/src/griz_tag/bagfiles/matlab/";
data.img.corner = [490 404 522;
                   235 259 331
                   1   1   1];
               
bagselect = rosbag(dataset.image.path + dataset.image.name);
bagselect2 = select(bagselect,'Time',...
    [bagselect.StartTime bagselect.StartTime + 1],'Topic','/camera/color/image_raw');
allMsgs = readMessages(bagselect2);
[img,~] = readImage(allMsgs{1});
figure()
imshow(img)