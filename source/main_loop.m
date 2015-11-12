%%main loop
clc; clear all; close all;
%run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/template/');
[ featureTable, camProjTable, featureCell,Z  ] = initalTwoViewRecon( ims{1}, ims{2});

for i = 3:47
    [ featureTable, camProjTable, featureCell,Z ] = updateStructure( ims{i},featureTable, camProjTable, featureCell,Z,i );
    fprintf('finish view %d',i);
    size(find(featureTable(:,129) ~= 0),1)
end

intensity = sum(featureTable(:,132:134),2);

save_ply('what.ply',featureTable(featureTable(:,129) ~= 0 & intensity(:) > 100,129:end));

save_Cam_Pos(camProjTable);