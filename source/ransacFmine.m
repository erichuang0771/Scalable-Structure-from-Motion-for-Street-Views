function [ F,P1_inlier, P2_inlier, inlier_ind ] = ransacF( pts1, pts2, M )
% ransacF:
%   pts1 - Nx2 matrix of (x,y) coordinates
%   pts2 - Nx2 matrix of (x,y) coordinates
%   M    - max (imwidth, imheight)

% Q2.X - Extra Credit:
%     Implement RANSAC
%     Generate a matrix F from some '../data/some_corresp_noisy.mat'
%          - using eightpoint
%          - using ransac

%     In your writeup, describe your algorith, how you determined which
%     points are inliers, and any other optimizations you made

P1 = padarray(pts1,[0,1],1,'post');
P2 = padarray(pts2,[0,1],1,'post');
threshld = 5;
    [F, P1_inlier, P2_inlier,inlier_ind] = compute_F_RANSAC(P1,P2,300,threshld,M);
    size(P2_inlier,1)
end


function [output, P1_inlier, P2_inlier,inlier_ind] = compute_F_RANSAC(P1,P2,n,thresh,M)

P1_inlier = []; P2_inlier = [];inlier_ind = [];

    sum_ = 0; min_error = inf;
    for i = 1:n
        if size(P1,1) > 7
            P1_inlier_ = []; P2_inlier_ = [];
            perm = randperm(size(P1,1)) ;
            sel = perm(1:7);
            [ F_hat ] = sevenpoint( P1(sel,1:2), P2(sel,1:2), M );
            %H_hat = homography(P2(sel,:),P1(sel,:));
            min = Inf;
            d = [];
            for i = 1:size(F_hat,1)
                 if ~isreal(F_hat{i})
                     continue;
                 end
                 El = P1*F_hat{i};
                 Er = P2*F_hat{i}';
                 tmp = abs(sum(El.*P2,2)./sqrt(sum(El(:,1:2).^2,2)))...
                     +abs(sum(Er.*P1,2)./sqrt(sum(Er(:,1:2).^2,2)));
                 
                 if sum(abs(tmp),1) < min
                     min = sum(abs(tmp),1);
                     d = tmp;
                 end
            end
            
            counter = 1;sum_ = 0; error = 0;
            
            ind = find(d<thresh);
            P1_inlier_ = P1(ind,:);
            P2_inlier_ = P2(ind,:);
            error = sum(d(ind));
            
            sum_ = size(P1_inlier_,1);
                if error/sum_ < min_error && sum_ > 32
               %      disp('new model!')
               inlier_ind = ind;
                    min_error = error/sum_;
                    P1_inlier = P1_inlier_;
                    P2_inlier = P2_inlier_;
                end
        end
    end
    if isempty(P1_inlier) || isempty(P2_inlier)
        output = zeros(3,3);
    else
        output = eightpoint(P1_inlier(:,1:2),P2_inlier(:,1:2),M);
    end
end
