% figure(888)
% clf(888)
% data1 = rand(1,20)./2;      %# Sample data set 1
% data2 = 0.3+rand(1,20)./2;  %# Sample data set 2
% hAxes = axes('NextPlot','add',...           %# Add subsequent plots to the axes,
%              'DataAspectRatio',[1 1 1],...  %#   match the scaling of each axis,
%              'XLim',[0 10],...               %#   set the x axis limit,
%              'YLim',[0 eps],...             %#   set the y axis limit (tiny!),
%              'Color','none');               %#   and don't use a background color
% plot(data1,0,'r*','MarkerSize',10);  %# Plot data set 1
% plot(data2,0,'b.','MarkerSize',10);  %# Plot data set 2
%% 
clear, clc, close all
path = "../../LiDARTag_data/";
[BagData, TestData] = getBagData();
 % target placements
bagfile_names = ["lab3-closer-cleaner.bag", ... 
                 "lab4-closer-cleaner.bag", ...
                 "lab5-closer-cleaner.bag", ...
                 "lab7-closer-cleaner.bag", ...
                 "lab8-closer-cleaner.bag", ...
                 "wavefield3-tag.bag", ...
                 "wavefield5-tag.bag"];
pc_iter = 1;
num_scan = 5;
flipped = -1;
for index = 1:length(bagfile_names)             
    current_bag = find(strcmp(bagfile_names(index), [BagData(:).bagfile]));
    for tag = 1:BagData(current_bag).num_tag 
        pc = loadPointCloud(path, BagData(current_bag).lidar_target(tag).pc_file);
        X = getPayload(pc, pc_iter, num_scan);
        hAxes = axes('NextPlot','add',...           % Add subsequent plots to the axes,
                     'DataAspectRatio',[1 1 1],...  % mtch the scaling of each axis,
                     'XLim',[0 10],...              % set the x axis limit,
                     'YLim',[0 eps],...             % set the y axis limit (tiny!),
                     'Color','none');               % and don't use a background color
                 hold on
        % title("target placement")
        distance_to_lidar = norm(mean(X'));
        
        d = BagData(current_bag).lidar_target(tag).tag_size;
        if d < 0.5 % small tag
            plot(distance_to_lidar, 0,'r*','MarkerSize',10);  %# Plot data set 1
            text(distance_to_lidar, flipped*100*eps, "t_{" + num2str(current_bag) + "}")
%             text(distance_to_lidar, flipped*100*eps, "t_" + num2str(index) + "(" + num2str(current_bag) + ")")
        else
            % large tag
            plot(distance_to_lidar, 0,'b.','MarkerSize',10);  %# Plot data set 2
            text(distance_to_lidar, flipped*100*eps, "T_{" + num2str(current_bag) + "}")
        end
    end
    flipped = flipped * -1;
end
xlabel('Target distance to lidar [m]')