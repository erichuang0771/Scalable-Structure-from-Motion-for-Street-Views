function [ featureTable, camProjTable, featureCell,Z, last_feature, last_desc, last_3D] = updateStructure( ims,featureTable, camProjTable, featureCell,Z, last_feature, last_desc, last_3D, debug_im )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    % extract feature    
    I = single(rgb2gray(ims));
    [f,d] = vl_sift(I);
    M = max(size(ims));
    % find the correspounding
    [matches, ~] = vl_ubcmatch(d, uint8(featureTable(:,1:128)'),2);
    fprintf('Num of matches: %d\n',size(matches,2))
%     points3D = featureTable(matches(2,:),129:131);
%     points2D = [f(1,matches(1,:))' f(2,matches(1,:))', ones(size(matches(1,:)))'];
%     remove_index = find(points3D(:,1) == 0 & points3D(:,2) == 0 & points3D(:,3) == 0  );
%     points3D(remove_index,:) = [];
%     points2D(remove_index,:) = [];

    NUM = size(matches,2);

%     inlier_index = matches(1,:);
%     if size(points3D,1) == 0
%          fprintf('Num of useful 3D points: %d\n',size(matches,2)-size(remove_index,1))
%         pause;
%     end

    %% estimate the camera projection matrix
    % not sure it is correct or not
    width = size(ims,2);
    height = size(ims,1);
    [matches_F, ~] = vl_ubcmatch(last_desc,d);
    match_plot(debug_im,ims, last_feature(1:2,matches_F(1,:))',f(1:2,matches_F(2,:))');
    [ ~, ~, F_inliner_2D, F_inlier_index ] = ransacF( last_feature(1:2,matches_F(1,:))', f(1:2,matches_F(2,:))', max(width,height));
    
    [ Proj] = estimateCameraProjRANSAC( last_3D(F_inlier_index,:), F_inliner_2D, max(width,height));
    new_feature = setdiff(1:size(f,2),matches(1,:));
    
    
    
    %% build featureTable
    % assign SIFT decs
    featureTable = [featureTable; double([d(:,new_feature)' zeros(size(new_feature,2),6)])] ;
    % assign 3D points color need to be done
    % done in triangulation
    
    
    %% feature Cell
    for cnt = 1:size(new_feature,2)
        % pushback new 2d pos
        i = new_feature(cnt);
        featureCell{end+1} =  f(1:2,i);
    end
    for cnt = 1:NUM
        % add new 2d pos into already exists feature cell
        i = matches(2,cnt);
        if size(featureCell{i},2) > size(find(Z(:,i)==1),1)
            continue;
        end
        featureCell{i} = [featureCell{i} f(1:2,matches(1,cnt))];
    end
    
    
    %done
    
    %% Z table
    % insert new feature
    tmp = size(Z,2);
    Z = [Z zeros(size(Z,1),size(new_feature,2))];
    Z = [Z;  [ zeros(1,tmp) ones(1, size(new_feature,2))]];
    % should update the matching points with new frame
    for i = 1:NUM
        ind = matches(2,i);
        Z(end,ind) = 1;
    end
    %done
    
    %% camProjTable
    
      camProjTable(:,:,end+1) = Proj;
    
    %done
    
    %% triangulartion
     [ featureTable, camProjTable, featureCell,Z ] = MultiViewTriangulationAll( featureTable, camProjTable, featureCell,Z, ims);
   %  save test.mat featureTable camProjTable  featureCell Z
     %save_ply('what.ply',featureTable(:,129:end));
     
      %% get last 3D cords and the last feature and desc to estimate the proj M for next frame
     last_3D = featureTable(matches(2,:), 129:131);
     last_feature = f(:,matches(1,:));
     last_desc = d(:,matches(1,:));

end

