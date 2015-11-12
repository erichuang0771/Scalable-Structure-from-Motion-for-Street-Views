run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/template/');
paras = dlmread('../data/template/templeR_par.txt',' ',0,1);
width = size(ims{1},2);
height = size(ims{1},1);

camProjTable = zeros(3,4,47);

for i = 1:47
    K1 = reshape(paras(i,1:9),3,3)';
    R1 = reshape(paras(i,10:18),3,3)';
    T1 = reshape(paras(i,19:end),3,1);
    camProjTable(:,:,i) = K1*[R1 T1];
end

save_Cam_Pos(camProjTable);
%% Next step
%find intrinsic and extrinsic matirx from F
% trangulation with color
%%%how to do bundle adjustment?
% save point cloud as ply by calling save_ply function
% imgname.png k11 k12 k13 
%              k21 k22 k23 
%              k31 k32 k33
%              
%              r11 r12 r13 
%              r21 r22 r23 
%              r31 r32 r33 
%              
%              t1 t2 t3"
