function line = best_fit_3D_line(pc, display)
    X = pc(1:3,:)';
%     X = pc;
    N=size(X,1);
    
    % Find line of best fit (in least-squares sense) through X
    % -------------------------------------------------------------------------
    X_ave=mean(X,1);            % mean; line of best fit will pass through this point
    dX=bsxfun(@minus,X,X_ave);  % residuals
    C=(dX'*dX)/(N-1);           % variance-covariance matrix of X
    [R,D]=svd(C,0);             % singular value decomposition of C; C=R*D*R'
    
    % Coefficient of determination; R^2 = (explained variance)/(total variance)
    D=diag(D);
    R2=D(1)/sum(D);
    line.dX = dX;
    line.m = R(:,1)';
    line.X0 = X_ave;
    line.R2 = R2;
    if checkDisplay(display)
        % Visualize X and line of best fit
        % -------------------------------------------------------------------------
        % End-points of a best-fit line (segment); used for visualization only
        x=dX*R(:,1);    % project residuals on R(:,1)
        x_min=min(x);
        x_max=max(x);
        dx=x_max-x_min;
        Xa=(x_min-0.05*dx)*R(:,1)' + X_ave;
        Xb=(x_max+0.05*dx)*R(:,1)' + X_ave;
        X_end=[Xa;Xb];
        figure(3000)
        axis equal
        hold on
        Hl=plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-r','LineWidth',3); % best fit line
        Hg=plot3(X(:,1),X(:,2),X(:,3),'.k','MarkerSize',13);         
        set(get(gca,'Title'),'String',sprintf('R^2 = %.3f',R2),'FontSize',25,'FontWeight','normal')
        xlabel('X','FontSize',20,'Color','k')
        ylabel('Y','FontSize',20,'Color','k')
        zlabel('Z','FontSize',20,'Color','k')
        view([20 20])
        drawnow
        % Display line parameters
        % -------------------------------------------------------------------------
        fprintf('Best fit line : L(t) = Xo + t*r, where\n')
        fprintf('Xo  = [ '); fprintf('%.4f ',X_ave);   fprintf(']\n') 
        fprintf('r   = [ '); fprintf('%.4f ',R(:,1)'); fprintf(']\n') 
        fprintf('R^2 = %.4f\n',R2)
    end
end