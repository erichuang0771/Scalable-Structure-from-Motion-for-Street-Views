function [ F ] = eightpoint_( pts1, pts2, M )
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.1 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save F, M, pts1, pts2 to q2_1.mat

%     Write F and display the output of displayEpipolarF in your writeup

M = [2/M,0,-1;0,2/M,-1;0,0,1];
pts1_n = padarray(pts1,[0,1],1,'post')*M';
pts2_n = padarray(pts2,[0,1],1,'post')*M';
A = pts1_n.*repmat(pts2_n(:,1),1,3);
B = pts1_n.*repmat(pts2_n(:,2),1,3);
C = [A B pts1_n];
[~,~,V] = svd(C);
F = V(:,end);
F = reshape(F,3,3)';
refineF(F,pts1,pts2);
F = M'*F*M;
end

