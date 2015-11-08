run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/');
width = size(ims{1},2);
height = size(ims{1},1);

[ P1,P2 ] = detectSIFT( ims );
[ F, P1_inlier, P2_inlier ] = ransacF( P1(:,1:2), P2(:,1:2), max(width,height));
match_plot(ims{1},ims{2},P1_inlier(:,1:2),P2_inlier(:,1:2));
%% Next step
%find intrinsic and extrinsic matirx from F
% trangulation with color
%%%how to do bundle adjustment?
% save point cloud as ply by calling save_ply function

