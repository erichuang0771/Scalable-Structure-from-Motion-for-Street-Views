function [ featureTable, camProjTable, featureCell,Z ] = updateStructure( ims,featureTable, camProjTable, featureCell,Z )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    % extract feature    
    I = single(rgb2gray(ims));
    [f,d] = vl_sift(I);
    
    %find the correspounding
    [matches, scores] = vl_ubcmatch(d, uint8(featureTable(:,1:128)'));
    points3D = featureTable(matches(2,:),129:131);
    points2D = [f(1,matches(1,:))' f(2,matches(1,:))', ones(size(matches(1,:)))'];
    
    %estimate the camera pose
    [ Proj, pts1_inlier_, pts2_inlier_, index ] = estimateCameraProjRANSAC( points3D, points2D );
    
    


end

