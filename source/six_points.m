function [ Proj ] = six_points( points3D, points2D,M)
%UNTITLED5 Summary of this function goes here
%   points3D N*4 
%   points2D N*3
    N = size(points3D,1);
    M = [1/M,0,0;0,1/M,0;0,0,1];
    % Scale ??
    points2D = points2D*M';
    A = [points3D zeros(N,4) -repmat(points2D(:,1),1,4).*points3D];
    B = [zeros(N,4) points3D -repmat(points2D(:,2),1,4).*points3D];
    [~,~,V] = svd([A;B]);
    P = V(:,end);
    Proj = inv(M)*reshape(P,4,3)';
end
