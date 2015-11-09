%%all view test
run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/template/');
paras = dlmread('../data/template/templeR_par.txt',' ',0,1);
width = size(ims{1},2);
height = size(ims{1},1);
M = max(width,height);

pc = []; M2_ = [eye(3),zeros(3,1)];
for i = 1:size(paras,2)-1
    K1 = reshape(paras(i,1:9),3,3)';
    K2 = reshape(paras(i+1,1:9),3,3)';
   [ points_color, M2_ ] = generatePointCloud2view( ims{i}, ims{i+1},M, K1,K2,M2_);
    pc = [pc;points_color];
    sprintf('procsing %d/%d view.....',i,size(paras,2)-1);
end