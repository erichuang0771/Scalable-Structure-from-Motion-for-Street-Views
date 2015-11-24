function [ F ] = sevenpoint_( pts1, pts2, M )
% sevenpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.2 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save recovered F (either 1 or 3 in cell), M, pts1, pts2 to q2_2.mat

%     Write recovered F and display the output of displayEpipolarF in your writeup
M = [1/M,0,0;0,1/M,0;0,0,1];
pts1_n = padarray(pts1,[0,1],1,'post')*M';
pts2_n = padarray(pts2,[0,1],1,'post')*M';
A = pts1_n.*repmat(pts2_n(:,1),1,3);
B = pts1_n.*repmat(pts2_n(:,2),1,3);
C = [A B pts1_n];
[~,~,V] = svd(C);
F1 = V(:,end);F1 = reshape(F1,3,3);
F2 = V(:,end-1);F2 = reshape(F2,3,3);
F1 = M'*F1*M;
F2 = M'*F2*M;
syms l 'real';
eq = det(l*F1 + (1-l)*F2);
cof = sym2poly(eq);
lambda = roots(cof);
F = cell(size(lambda,1),1);
for i = 1:size(lambda,1)
    F{i} = F1*lambda(i) + (1-lambda(i))*F2;
end

end

