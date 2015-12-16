%% find the truth ground
clear all; clc;
load K.mat
load camProj.mat;
load camPoseTable.mat;
camProj_1 = [camPoseTable(:,:,1);[0,0,0,1]];


camera_center = zeros(size(camProjTable,3),3);
for i = 1:size(camProjTable,3)
    R = camProjTable(1:3,1:3,i);
    T = camProjTable(:,4,i);
    camera_center(i,:) = [(-inv(R)*T)'];
end

R1 = camPoseTable(1:3,1:3,1);
T1 = camPoseTable(1:3,end,1);
R2 = camPoseTable(1:3,1:3,2);
T2 = camPoseTable(1:3,end,2);
R3 = camPoseTable(1:3,1:3,3);
T3 = camPoseTable(1:3,end,3);

P3 = R3*inv(R1)*(-T1)+T3

R3*inv(R1)

