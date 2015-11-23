%% test 
clear; clc;
load camProj.mat;
[points3D] = (rand(100,3)-0.5)*10 + (rand(100,3)- 0.5)*2;
%scatter3(points3D(:,1),points3D(:,2),points3D(:,3))
cam1 = camProjTable(:,:,1);
points3D = padarray(points3D,[0,1],1,'post');
points2D = cam1*points3D';
points2D = points2D./repmat(points2D(3,:),3,1);

camProj = estimateCameraProjRANSAC( points3D(:,1:3), points2D(1:3,:)', 300 );
camProj./cam1
