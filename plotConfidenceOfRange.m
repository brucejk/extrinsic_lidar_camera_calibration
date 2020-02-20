function plotConfidenceOfRange(fig_handles, confidence_of_range, t_title)
    num_scene = size(confidence_of_range, 2);
    x_plot_all = [];
    y_plot_all = [];
    title_all = t_title + ": ";
    for scene = 1:num_scene % which dataset
        num_scans = size(confidence_of_range(scene).scans, 2);
        x_plot = [];
        y_plot = [];
        for scan_num = 1:num_scans
            num_targets = size(confidence_of_range(scene).scans(scan_num).targets, 2);
            if num_targets ~= 0
                for tag_num = 1:num_targets
                    x_plot = [x_plot, confidence_of_range(scene).scans(scan_num).targets(tag_num).distance];
                    y_plot = [y_plot, confidence_of_range(scene).scans(scan_num).targets(tag_num).RMSE];
                end
            end
        end
        [x_plot, sorted_indices] = sort(x_plot);
        plot(fig_handles(scene), x_plot, y_plot(sorted_indices), '-x')
        title_txt = t_title + ": " + confidence_of_range(scene).bagfile;
        title(fig_handles(scene), title_txt)
        xlabel(fig_handles(scene), "distance [m]")
        ylabel(fig_handles(scene), "RMSE [pixel]")
        set(get(fig_handles(scene),'parent'),'visible','on');
        hold(fig_handles(scene), 'off')
        
        x_plot_all = [x_plot_all, x_plot];
        y_plot_all = [y_plot_all, y_plot(sorted_indices)];
        title_all = title_all + " -- "+confidence_of_range(scene).bagfile;
    end
    [x_plot_all, sorted_indices] = sort(x_plot_all);
    plot(fig_handles(num_scene+1), x_plot_all, y_plot_all(sorted_indices), '-x')
    title(fig_handles(num_scene+1), title_all)
    xlabel(fig_handles(num_scene+1), "distance [m]")
    ylabel(fig_handles(num_scene+1), "RMSE [pixel]")
    set(get(fig_handles(num_scene+1),'parent'),'visible','on');
    hold(fig_handles(num_scene+1), 'off')
end