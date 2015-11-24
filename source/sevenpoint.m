function [ F ] = sevenpoint( pts1, pts2, M )
% sevenpoint:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.2 - Todo:
%     Implement the eightpoint algorithm
%     Generate a matrix F from some '../data/some_corresp.mat'
%     Save recovered F (either 1 or 3 in cell), M, pts1, pts2 to q2_2.mat

%     Write recovered F and display the output of displayEpipolarF in your writeup

% Scaling
N = size(pts1, 1);
T = diag([1 / M(1), 1 / M(2), 1]);
pts1_norm = [pts1, ones(N, 1)] * T';
pts2_norm = [pts2, ones(N, 1)] * T';
% Calculate F1 and F2
U = [pts1_norm(:, 1) .* pts2_norm(:, 1), pts1_norm(:, 2) .* pts2_norm(:, 1), pts2_norm(:, 1), ...
    pts1_norm(:, 1) .* pts2_norm(:, 2), pts1_norm(:, 2) .* pts2_norm(:, 2), pts2_norm(:, 2), ...
    pts1_norm(:, 1), pts1_norm(:, 2), ones(N, 1)];
[vec, ~] = eig(U' * U);
F1 = reshape(vec(:, 1), 3, 3)';
F2 = reshape(vec(:, 2), 3, 3)';
% Find all real roots
syms l;
poly = sym2poly(det((1 - l) * F1 + l * F2));
root = roots(poly);
root = root(root == real(root));
% return F
F = cell(numel(root), 1);
for i = 1 : size(root)
    F{i} = T' * ((1 - root(i)) * F1 + root(i) * F2) * T;
end
% Save
% save q2_2.mat F M pts1 pts2

end

