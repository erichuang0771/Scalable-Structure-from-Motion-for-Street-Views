function [ pc, newfeatureTable ] = addNewView( ims, featureTable, M)
%UNTITLED2 Summary of this function goes here
%   featureTable SIFTdes|3D corrd|P matrix
% SIFTdes = 128 * N

      [f,d] = vl_sift(single(rgb2gray(ims)));
      [matches, scores] = vl_ubcmatch(d, featureTable(:,1:128)');
      xx = [f(1,matches(1,:))' f(2,matches(1,:))', ones(size(matches(1,:)))'];
      XX = featureTable(matches(2,:),129:129+2);
      [R t]= RPnP(XX,xx);
      P2 = featureTable()
      [ F, P1_inlier, P2_inlier ] = ransacF( P1(:,1:2), P2(:,1:2), M);

     
    


end

