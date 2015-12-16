clear all; close all;
N = 0;
a = [];
for i = 0 : N
    a = [a; load(['BA_', num2str(i), '.mat'],'-ascii')];
end
a = [a, ones(size(a, 1), 1), ones(size(a, 1), 2)*200];
save_ply(['test_final_BA', num2str(i), '.ply'], a);

