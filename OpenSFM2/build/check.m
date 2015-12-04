clear all; close all;
N = 2;
p = zeros(3,4,N);
p(:,:,1) = load(['camProjTable', num2str(0), '.mat'],'-ascii');
for i = 0 : N
    a = load(['featureTable_', num2str(i), '.mat'],'-ascii');
    a = a';
    a = a(:,129:end);
    a(a(:,1)==0, :) = [];
    save_ply(['test_final_', num2str(i), '.ply'], a);
    
    
    p(:,:,i+2) = load(['camProjTable', num2str(i+1), '.mat'],'-ascii');
    
end

save_Cam_Pos(p);




disp('done~')

