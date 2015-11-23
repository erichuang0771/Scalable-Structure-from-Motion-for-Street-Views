function [omega,w] = computeCameraCenter(M,p)

%compute A and b
A = [M(:,1), M(:,2), M(:,3)];
b = [M(:,4)];

%compute omega
omega = -1*inv(A)*b;
w = inv(A)*p';