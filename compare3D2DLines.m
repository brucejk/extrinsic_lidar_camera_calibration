function cost = compare3D2DLines(lines_3D, lines_2D, P)
    N = size(lines_3D, 2);
    cost = 0;
    for i = 1:N
        line_3D = lines_3D(i).line;
        x = line_3D.dX*(line_3D.m)';    % project residuals on R(:,1)
        x_min = min(x);
        x_max = max(x);
        dx = x_max-x_min;
        Xa = P * [(x_min-0.05*dx)*line_3D.m + line_3D.X0 1]';
        Xa = Xa ./ Xa(3,:);
        Xb = P * [(x_max+0.05*dx)*line_3D.m + line_3D.X0 1]';
        Xb = Xb ./ Xb(3,:);

        cost1 = pointToLineDistance((Xa(1:2,1))', ...
                            [lines_2D(i).x(1) lines_2D(i).y(1)], ...
                            [lines_2D(i).x(2) lines_2D(i).y(2)]);
        cost2 = pointToLineDistance((Xb(1:2,1))', ...
                            [lines_2D(i).x(1) lines_2D(i).y(1)], ...
                            [lines_2D(i).x(2) lines_2D(i).y(2)]);
        cost = cost + cost1 + cost2;
    end
end

%         x=dX*R(:,1);    % project residuals on R(:,1)
%         x_min=min(x);
%         x_max=max(x);
%         dx=x_max-x_min;
%         Xa=(x_min-0.05*dx)*R(:,1)' + X_ave;
%         Xb=(x_max+0.05*dx)*R(:,1)' + X_ave;
%         X_end=[Xa;Xb];
%         figure(3000)
%         axis equal
%         hold on
%         Hl=plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-r','LineWidth',3); % best fit line