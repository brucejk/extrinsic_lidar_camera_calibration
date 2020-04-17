% X = zeros(3,5); % correct but non-homo
% X = zeros(4,5); % correct input
% X = zeros(5,3); % incorrect nor homo
% X = zeros(5,4); % incorrect but homo
% % X = zeros(5,5); % wrong usage
% P = ones(3,4);
% t_projectionMap(X, P);

function projected_points = projectionMap(X, P)
%     X = makeWideMatrix(X);
%     [m, n] = size(X);
%     if (m ~= 3 && m ~= 4)
%         if (n ~= 3 && m ~= 3)
%             error("wrong usage: the input should be 3xn, nx3, 4xn, nx4")
%         end
%     end
    [~, X] = checkHomogeneousCoord(X);
    projected_points = P * X;
    projected_points = projected_points ./ projected_points(3, :);
end