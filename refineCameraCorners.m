function refined_camera_corners = refineCameraCorners(camera_corners, img, display, t_clean)
    if nargin > 3
        clean = t_clean;
    else
        clean = 1;
    end
    extend_fac = 2;
    corner_array = [1 1 2 3
                    2 3 4 4];
    gray = rgb2gray(img);
    BW = edge(gray, 'Canny', [0.04]);

    if checkDisplay(display)
        figure(1000)
        if clean
            clf('reset')
            imshow(img)
        end
        hold on
%         title(file)
        figure(2000)
        if clean
            clf('reset')
            imshow(BW)
        end
        hold on
%         title(file)
    end
    if checkDisplay(display)
%                 disp("Before modification")
%                 disp([BagData(k).camera_target(j).corners])
    end
    for i = 1: length(corner_array)
        p1 = camera_corners(1:2, corner_array(1,i));
        p2 = camera_corners(1:2, corner_array(2,i));

        vec = [p1 - p2];
        vec_normlized = vec/norm(vec);
        vec_p = [vec_normlized(2); -vec_normlized(1)];
        p1_ext = p1 + extend_fac * vec_normlized + extend_fac * vec_p;
        p2_ext = p2 - extend_fac * vec_normlized + extend_fac * vec_p;

        p3_ext = p2 - extend_fac * vec_normlized - extend_fac * vec_p;
        p4_ext = p1 + extend_fac * vec_normlized - extend_fac * vec_p;


        corners = [p1_ext, p2_ext, p3_ext, p4_ext];
        [x2, y2] = poly2cw(corners(1,:)', corners(2,:)');
        [img_y, img_x] = size(gray);
        x_dim = linspace(1, img_x, img_x);
        x_dim = repmat(x_dim,1,img_y);
        y_dim = linspace(1, img_y, img_y);
        y_dim = repelem(y_dim,1, img_x);

        [in, ~] = inpolygon(x_dim, y_dim, x2, y2);
        x_dim = x_dim(in);
        y_dim = y_dim(in);

        data = [];
        for t = 1:size(x_dim, 2)
            if BW(y_dim(t), x_dim(t))
                data = [data, [x_dim(t); y_dim(t)]];
            end
        end
        [x, y, line_model, inlier_pts] = ransacLineWithInlier(data', 0.1);

        if checkDisplay(display)
            figure(2000)
            hold on
            scatter(camera_corners(1,:), camera_corners(2,:))
            scatter(p1(1), p1(2))
            scatter(p2(1), p2(2))
            scatter(p1_ext(1), p1_ext(2))
            scatter(p2_ext(1), p2_ext(2))
            scatter(p3_ext(1), p3_ext(2))
            scatter(p4_ext(1), p4_ext(2))
            scatter(x_dim, y_dim, 'g.');
            scatter(inlier_pts(:,1), inlier_pts(:,2), 'm.');
%                     plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])
            plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0, 1, 1])

            figure(1000)
            hold on 
            scatter(camera_corners(1,:), camera_corners(2,:))
            scatter(p1(1), p1(2))
            scatter(p2(1), p2(2))
            plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])
        end

        t_edge(i).x = x;
        t_edge(i).y = y;
        t_edge(i).line = line_model;
    end

    cross_big_2d = [];
    for i = 1: length(corner_array)
        point = intersection(t_edge(corner_array(1,i)).line, t_edge(corner_array(2,i)).line);
        cross_big_2d = [cross_big_2d, point];
    end
    cross_big_2d = sortrows(cross_big_2d', 2, 'ascend')';
    cross_big_2d = [cross_big_2d; ones(1, 4)];
    refined_camera_corners = cross_big_2d;

    if checkDisplay(display)
%                 disp("Afrer modification")
%                 disp([BagData(k).camera_target(j).corners])
        figure(2000)
        scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'filled');
        hold off

        figure(1000)
        scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'g','filled');
        hold off
        drawnow
    end

    if checkDisplay(display)
%             disp("press any key to continue")
%             pause;
%             clc
    end
end
