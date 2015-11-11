function [ Proj ] = six_points_all( points3D, points2D )
%UNTITLED5 Summary of this function goes here
%   points3D N*4 
%   points2D N*3
    N = size(points3D,1);
    
    A = [points3D zeros(N,4) -repmat(points2D(:,1),1,4).*points3D];
    B = [zeros(N,4) points3D -repmat(points2D(:,2),1,4).*points3D];
    [~,~,V] = svd([A;B]);
    P = V(:,end);
    Proj = reshape(P,4,3)';
end
