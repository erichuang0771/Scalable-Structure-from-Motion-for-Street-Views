function [ Proj ] = estimateCameraProjRANSAC( points3D, points2D )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    % Number of point correspondances
N = size(points3D, 1);
%padarry points3D X Y Z 1
points3D = padarray(points3D,[0,1],1,'post');
 Proj = six_points( points3D(:, :), points2D(:, :));
 return;
% The threshold to decide inliers
threshold = 10;
% Max number of inliers
inlier_num = 0;
% Randomly choose the initial 7 point pairs
for i = 1 : 1000
    random = randi(N, 1, 6);
    %random = [1:6];
    % Proj using six points
    Proj_six = six_points( points3D(random, :), points2D(random, :));
    
        % The value of reprojection error
        reproj_points2D = Proj_six*points3D';
        reproj_points2D = reproj_points2D./repmat(reproj_points2D(3,:),3,1);
        error = sum((reproj_points2D'-points2D).^2,2);
        % Inliers
        inlier = abs(error) < threshold;
        if sum(inlier) > inlier_num
            inlier_num = sum(inlier);
            index = find(abs(error) < threshold);
            % Proj using eightpoint with inliers
            Proj = six_points( points3D(inlier, :), points2D(inlier, :));
            pts1_inlier_ = points3D(inlier, :);
            pts2_inlier_ = points2D(inlier, :);
        end
    end
end


