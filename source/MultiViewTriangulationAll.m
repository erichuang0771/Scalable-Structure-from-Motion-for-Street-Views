function [ featureTable, camProjTable, featureCell,Z ] = MultiViewTriangulationAll( featureTable, camProjTable, featureCell,Z,ims )
%UNTITLED2 Summary of this function goes here

% update every feature if it viewed by at least two cameras
    for i = 1:size(featureTable,1)
        ind = find(Z(:,i)==1);
        pts = featureCell{i};
        ProjM = camProjTable(:,:,ind);
        n = size(ind,1);
        if n < 2 continue; end;
        A = [reshape(ProjM(3,1,:),n,1).*pts(1,:)'-reshape(ProjM(1,1,:),n,1) reshape(ProjM(3,2,:),n,1).*pts(1,:)'-reshape(ProjM(1,2,:),n,1) ...
            reshape(ProjM(3,3,:),n,1).*pts(1,:)'-reshape(ProjM(1,3,:),n,1) reshape(ProjM(3,4,:),n,1).*pts(1,:)'-reshape(ProjM(1,4,:),n,1)];
        
        B = [reshape(ProjM(3,1,:),n,1).*pts(2,:)'-reshape(ProjM(2,1,:),n,1) reshape(ProjM(3,2,:),n,1).*pts(2,:)'-reshape(ProjM(2,2,:),n,1) ...
            reshape(ProjM(3,3,:),n,1).*pts(2,:)'-reshape(ProjM(2,3,:),n,1) reshape(ProjM(3,4,:),n,1).*pts(2,:)'-reshape(ProjM(2,4,:),n,1)];
       
        [~,~,V] = svd([A;B]);
        point3D = V(:,end); point3D = point3D/point3D(4);
        featureTable(i,129:131) = point3D(1:3)';
        if featureTable(i,132) == 0
            featureTable(i,132:134) = reshape(ims(round(pts(2,1)),round(pts(1,1)),:),1,3);
        end
    end
end

