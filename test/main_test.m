run('../library/vlfeat/vlfeat-0.9.20/toolbox/vl_setup.m')
addpath ../source/;
ims = loadImages('../data/template/');
paras = dlmread('../data/template/templeR_par.txt',' ',0,1);
width = size(ims{1},2);
height = size(ims{1},1);

[ P1,P2 ] = detectSIFT( ims );
[ F, P1_inlier, P2_inlier ] = ransacF( P1(:,1:2), P2(:,1:2), max(width,height));
%%get intrinsic matrix
K1 = reshape(paras(1,1:9),3,3)';
K2 = reshape(paras(2,1:9),3,3)';

E = essentialMatrix( F, K1, K2 );
M1 = [eye(3) zeros(3,1)];
M2 = camera2(E);
error = zeros(size(M2,3),1);
P = cell(size(M2,3),1);
for i=1:size(M2,3)
   [P{i},error(i),color]= triangulate_color(K1*M1,P1_inlier(:,1:2),K2*M2(:,:,i),P2_inlier(:,1:2), ims{1}, ims{2});
end
[~,ind] = min(error);
M2_ = M2(:,:,ind);

save_ply('hehe.ply',[P{ind} color]);

% R1 = reshape(paras(1,10:18),3,3)';
% R2 = reshape(paras(2,10:18),3,3)';
% 
% T1 = reshape(paras(1,19:end),3,1);
% T2 = reshape(paras(1,19:end),3,1);





match_plot(ims{1},ims{2},P1_inlier(:,1:2),P2_inlier(:,1:2));

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
