function [ bestE ] = five_pts_solver( pts1,pts2,K )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
h_pts1 = [pts1 ones(size(pts1,1),1)];
h_pts2 = [pts2 ones(size(pts2,1),1)];


h_pts1 = K \ h_pts1';
h_pts2 = K \ h_pts2';
h_pts1 = h_pts1';
h_pts2 = h_pts2';


%% Our 5-point implementation

 EList = five_point(h_pts1, h_pts2);


%% Check error

minSumError = inf;

for i=1:length(EList)
  E = EList{i};
  i
  % Check determinant constraint! 
  error_determinate = det( E);
  % Check trace constraint
  error_trace = 2 *E*transpose(E)*E -trace( E*transpose(E))*E;

  % Check reprojection errors
  error_reprojection = diag( h_pts2 * E * h_pts1');
  sum_error_proj = sum(abs(error_reprojection))
  
  % Find E with the smallest error
  if (sum_error_proj < minSumError)
      minSumError = sum_error_proj;
      bestE = E;
  end

end

