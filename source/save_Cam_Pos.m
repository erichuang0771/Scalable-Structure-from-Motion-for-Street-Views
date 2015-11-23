function [] = save_Cam_Pos(camProjTable)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    camera_center = zeros(size(camProjTable,3),6);
    for i = 1:size(camProjTable,3)
        R = camProjTable(1:3,1:3,i);
        T = camProjTable(:,4,i);
        camera_center(i,:) = [(-inv(R)*T)', [128,128,0]];
    end
    save_ply('Cams.ply',camera_center);

end


