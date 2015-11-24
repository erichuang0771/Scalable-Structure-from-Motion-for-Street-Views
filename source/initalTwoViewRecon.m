function [ featureTable, camProjTable, featureCell,Z, last_feature, last_desc, last_3D, camPose ,length   ] = initalTwoViewRecon( im1, im2 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    I1 = single(rgb2gray(im1));
    I2 = single(rgb2gray(im2));
    
    %%extract SIFT features
    [f1,d1] = vl_sift(I1);
    [f2,d2] = vl_sift(I2);
    
 
    
    %%match & find correspounding points
    [matches, ~] = vl_ubcmatch(d1, d2);
    P1 = [f1(1,matches(1,:))' f1(2,matches(1,:))', ones(size(matches(1,:)))'];
    P2 = [f2(1,matches(2,:))' f2(2,matches(2,:))', ones(size(matches(2,:)))'];
  
    %% RANSAC F
    width = size(im1,2);
    height = size(im1,1);
    [ F, A, B, inlier_index ] = ransacF( P1(:,1:2), P2(:,1:2), [width,height]);
    %load three.mat;
 %  inlier_index = index;
    %% find the F matrix ??? not sure
    Proj1 = [eye(3), zeros(3,1)];
    [~,~,V] = svd(F');
    b = V(:,end);
    A = -skewsymm(b)*F;
    Proj2 = [A b];
    
    NUM = size(P1,1);
    
    %% build featureTable
    featureTable = zeros(NUM,128+3+3);
    % assign SIFT decs
    featureTable(:,1:128) = d1(:,matches(1,:))';
    % assign 3D points color need to be done
    % done in triangulation
    
    
    %% camera Cell
    featureCell = cell(NUM,1);
    for i = 1:NUM
        % replace feature cell with orhinal points
        % all points are matched points
        featureCell{i} = [P1(i,1:2)' P2(i,1:2)'];
    end
    %done
    
    %% Z table
    Z = ones(2,size(P1,1));
    %done
    
    %% camProjTable
    load CamProj.mat
%     camProjTable = zeros(3,4,2);
%     %%%debug:
%         load proj1_2.mat;
%     %%%
%     camProjTable(:,:,1) = K1*[R1 T1];%Proj1;
%     camProjTable(:,:,2) = K2*[R2 T2];%Proj2;
    %done
    camProjTable(:,:,3:end) = [];
   
    
   
    %% triangulartion
     [ featureTable, camProjTable, featureCell,Z ] = MultiViewTriangulation( featureTable, camProjTable, featureCell,Z, inlier_index,im1);
    % save test.mat featureTable camProjTable  featureCell Z
     %save_ply('what.ply',featureTable(:,129:end));
     
     %% get last 3D cords and the last feature and desc to estimate the proj M for next frame
     last_3D = featureTable(inlier_index, 129:131);
     last_feature = f2(:,matches(2,inlier_index));
     last_desc = d2(:,matches(2,inlier_index));
     
     %% get the camera pose
     load camPoseTable.mat;
     camPose = cell(1,2);
     r = camProjTable(1:3,1:3,end);
     t = camProjTable(:,end);
     length = last_3D - repmat((-inv(r)*t)',size(last_3D,1),1);
     camPose{1} = camPoseTable(:,:,1);
     camPose{2} = camPoseTable(:,:,2);
     
end

