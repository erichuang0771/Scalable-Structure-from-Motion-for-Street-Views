function [ F ] = eightpoint( pts1, pts2, M )
% eightpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.1 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save F, M, pts1, pts2 to q2_1.mat

%     Write F and display the output of displayEpipolarF in your writeup

% Scaling
N = size(pts1, 1);
T = diag([1 / M(1), 1 / M(2), 1]);
pts1_norm = [pts1, ones(N, 1)] * T';
pts2_norm = [pts2, ones(N, 1)] * T';
% Calculate F
U = [pts1_norm(:, 1) .* pts2_norm(:, 1), pts1_norm(:, 2) .* pts2_norm(:, 1), pts2_norm(:, 1), ...
    pts1_norm(:, 1) .* pts2_norm(:, 2), pts1_norm(:, 2) .* pts2_norm(:, 2), pts2_norm(:, 2), ...
    pts1_norm(:, 1), pts1_norm(:, 2), ones(N, 1)];
[vec, ~] = eig(U' * U);
F = reshape(vec(:, 1), 3, 3)';
% Refine and enforce singularity
F = refineF(F, pts1_norm, pts2_norm);
% Unscaling
F = T' * F * T;
% Save
% save q2_1.mat F M pts1 pts2

end

