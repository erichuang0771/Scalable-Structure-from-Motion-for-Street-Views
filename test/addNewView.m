function [ pc, newfeatureTable ] = addNewView( ims,ims2, featureTable, M,K1,K2)
%UNTITLED2 Summary of this function goes here
%   featureTable SIFTdes|3D corrd|2D corrd|P matrix|K matrix
%
% SIFTdes = 128 * N

      [f,d] = vl_sift(single(rgb2gray(ims)));
      [matches, scores] = vl_ubcmatch(d, featureTable(:,1:128)');
      xx = [f(1,matches(1,:))' f(2,matches(1,:))', ones(size(matches(1,:)))'];
      XX = featureTable(matches(2,:),129:129+2);
      [R t]= RPnP(XX,xx);

end

