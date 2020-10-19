function camera_corners = clickImageGetCorners(image, title_txt)
    fig = figure(9999);
%     fig.MenuBar = 'none';
%     % Add menus with Accelerators
%     mymenu = uimenu('Parent',fig,'Label','Hot Keys');
%     uimenu('Parent',mymenu,'Label','Zoom','Accelerator','z','Callback',@(src,evt)zoom(fig,'on'));
%     uimenu('Parent',mymenu,'Label','Rotate','Accelerator','r','Callback',@(src,evt)rotate3d(fig,'on'));
%     uimenu('Parent',mymenu,'Label','Pan','Accelerator','p','Callback',@(src,evt)pan(fig,'on'));

    imshow(image)
    axes = gca(figure(9999));
    title(axes, title_txt);
%     vertices = [0.26944      0.10279     -0.19802     -0.17421
%                    -0.053716      0.16595     0.035211     -0.14745]';
%     vertices
%     vertices = vertices(:, [2 1]);   
%     vertices = 200*[vertices(:, 1), -vertices(:, 2)]+[50 50]; 
%     roi = images.roi.Freehand(gca,'Position',vertices);
%     roi = images.roi.Freehand(gca);
%     pause;
%     addlistener(roi,'MovingROI',@allevents);
%     addlistener(roi,'ROIMoved',@allevents);
    
    [img_x, img_y] = getpts(axes);
    camera_corners = [img_x'; img_y'];
end

% function allevents(src,evt)
% evname = evt.EventName;
%     switch(evname)
%         case{'MovingROI'}
%             disp(['ROI moving Previous Position: ' mat2str(evt.PreviousPosition)]);
%             disp(['ROI moving Current Position: ' mat2str(evt.CurrentPosition)]);
%         case{'ROIMoved'}
%             disp(['ROI moved Previous Position: ' mat2str(evt.PreviousPosition)]);
%             disp(['ROI moved Current Position: ' mat2str(evt.CurrentPosition)]);
%     end
% 
% end
