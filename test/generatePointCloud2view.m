function [ points_color, M2_ ] = generatePointCloud2view( Cam1, Cam2,M, K1,K2,M1)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    ims = cell(2);
    ims{1} = Cam1;
    ims{2} = Cam2;
    [ P1,P2 ] = detectSIFT(ims);
    [ F, P1_inlier, P2_inlier ] = ransacF( P1(:,1:2), P2(:,1:2), M);
        % match_plot(ims{1},ims{2},P1_inlier(:,1:2),P2_inlier(:,1:2));

    E = essentialMatrix( F, K1, K2 );
    M2 = camera2(E);
    R = M1(:,1:3);
    for i = 1:size(M2,3)
        M2R = R*M2(:,1:3,i);
        M2T = R*M2(:,4,i)+M1(:,4);
        M2(:,:,i) = [M2R,M2T];
    end
    error = zeros(size(M2,3),1);
    P = cell(size(M2,3),1);
    for i=1:size(M2,3)
      [P{i},error(i),color]= triangulate_color(K1*M1,P1_inlier(:,1:2),K2*M2(:,:,i),P2_inlier(:,1:2), ims{1}, ims{2});
    end
    [~,ind] = min(error);
    M2_ = M2(:,:,ind);
    points_color = [P{ind} color];
end

