%%main loop
clc; clear all; close all;
run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/template/');
[ featureTable, camProjTable, featureCell,Z  ] = initalTwoViewRecon( ims{1}, ims{2});

for i = 3:5
    [ featureTable, camProjTable, featureCell,Z ] = updateStructure( ims{i},featureTable, camProjTable, featureCell,Z, i );
    fprintf('finish view %d',i);
end
save_ply('what.ply',featureTable(:,129:end));