function [ E ] = essentialMatrix( F, K1, K2 )
% essentialMatrix:
%    F - Fundamental Matrix
%    K1 - Camera Matrix 1
%    K2 - Camera Matrix 2

% Q2.3 - Todo:
%       Compute the essential matrix 
%
%       Write the computed essential matrix in your writeup

E = K1'*F*K2;

end

