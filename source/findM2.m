% Q2.5 - Todo:
%       1. Load point correspondences
%       2. Obtain the correct M2
%       4. Save the correct M2, p1, p2, R and P to q2_5.mat

clear; close all;
load ../data/some_corresp.mat;
im1 = imread('../data/im1.png');
im2 = imread('../data/im2.png');
width = size(im1,2);
height = size(im1,1);
[ F ] = eightpoint( pts1, pts2, max(width,height) );
%displayEpipolarF(im1,im2,F);
load ../data/intrinsics.mat;
E = essentialMatrix( F, K1, K2 );
M1 = [eye(3) zeros(3,1)];
M2 = camera2(E);
error = zeros(size(M2,3),1);
P = cell(size(M2,3),1);
for i=1:size(M2,3)
   [P{i},error(i)]= triangulate(K1*M1,pts1,K2*M2(:,:,i),pts2);
end
[~,ind] = min(error);
P = P{i};
scatter3(P(:,1),P(:,2),P(:,3));