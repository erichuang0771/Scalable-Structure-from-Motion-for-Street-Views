function [ center, vector ] = cam_center_vector( p, m )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
c = p(:,1:3);
t = p(:,4);
m = [m;1];
center = - inv(c) * t;
vector = inv(c) * m;
end

