function [ output_args ] = triangulate_1(P_1,  pixels_1,P_2, pixels_2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
pixels_1 = pixels_1';
pixels_2 = pixels_2';
    [center_1, v_1] = cam_center_vector( P_1, pixels_1 );
    [center_2, v_2] = cam_center_vector( P_2, pixels_2 );
    t = center_2 - center_1;
    v_3 = cross(v_1,v_2);
    X = [v_1,v_3, - v_2];
    alpha =  inv(X) * t ;
    output_args = center_1 + v_1*alpha(1) + alpha(2)/2*v_3;

end

