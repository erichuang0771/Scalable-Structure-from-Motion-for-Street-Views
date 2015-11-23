%%main loop
clc; clear all; close all;
%run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
addpath ../paper1_impl

ims = loadImages('../data/template/');
[ featureTable, camProjTable, featureCell,Z, last_feature, last_desc, last_3D, camPose  ,length]...
    = initalTwoViewRecon( ims{1}, ims{2});
 %% debbug
%     intensity = sum(featureTable(:,132:134),2);
% 
%     save_ply('what.ply',featureTable(featureTable(:,129) ~= 0 & intensity(:) > 100,129:end));
% 
%     save_Cam_Pos(camProjTable);
%     
%%
for i = 3:47
    [ featureTable, camProjTable, featureCell,Z, last_feature, last_desc, last_3D, camPose, length  ]...
    = updateStructure( ims{i},featureTable, camProjTable, featureCell,Z,last_feature, last_desc, last_3D, ims{2}, camPose, length);
    
    fprintf('finish view %d',i);
    size(find(featureTable(:,129) ~= 0),1)
    
    %% debug
%     intensity = sum(featureTable(:,132:134),2);
% 
%     save_ply('what.ply',featureTable(featureTable(:,129) ~= 0 & intensity(:) > 100,129:end));
% 
%     save_Cam_Pos(camProjTable);
    %%
end

intensity = sum(featureTable(:,132:134),2);

save_ply('what.ply',featureTable(featureTable(:,129) ~= 0 & intensity(:) > 100,129:end));

save_Cam_Pos(camProjTable);