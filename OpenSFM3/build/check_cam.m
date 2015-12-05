clear all; close all; clc;

N = 0;
load('init_K.mat');
a = load(['featureTable_', num2str(N), '.mat'],'-ascii');
a = a';
a = a(:,129:end);
a(a(:,1)==0, :) = [];
hold on;
scatter3(a(:, 1), a(:, 2), a(:, 3), '.');


p = zeros(3,4,N + 2);
for i = 0 : N + 1
    p(:,:,i+1) = load(['camProjTable', num2str(i), '.mat'],'-ascii');
    p(:,:,i+1) = inv(K) * p(:,:,i+1);
end
R = p(1:3,1:3,1);
t = -R' * (p(1:3,4,1));
plotCamera('Location',t','Orientation',R,'Opacity',0,'Size',0.1);
Color = [0,1,0;0,0,1;1,0,0;1,1,0;1,0,1;0,1,1];
for i = 0 : N      
    R = p(1:3,1:3,i+2);
    t = -R' * (p(1:3,4,i+2));
    hold on;
    plotCamera('Location',t','Orientation',R,'Opacity',0,'Size',0.1,'Color',Color(mod(i,6) + 1, :));  
end


% save_Cam_Pos(p);




disp('done~')

