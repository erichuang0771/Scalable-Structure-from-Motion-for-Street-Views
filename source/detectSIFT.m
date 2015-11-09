function [ P1,P2 ] = detectSIFT( ims )
%UNTITLED Summary of this function goes here
%   1. Extract SIFT features
%   2. Match featues
%   Input: ims N cells of images, color 
    % currectly just work on two images  -> need to improve to N images
    
    N = size(ims,1);
    f = cell(N,1);
    d = cell(N,1);
    for i = 1:2
        I = single(rgb2gray(ims{i}));
        [f{i},d{i}] = vl_sift(I);
    end
    [matches, scores] = vl_ubcmatch(d{1}, d{2});
    
    
    f1 = f{1}; f2 = f{2};
    P1 = [f1(1,matches(1,:))' f1(2,matches(1,:))', ones(size(matches(1,:)))'];
    P2 = [f2(1,matches(2,:))' f2(2,matches(2,:))', ones(size(matches(2,:)))'];

    return
% checking    
figure(1) ;
show = [ims{1} ims{2}];
imshow(show);
shift = size(ims{1},2) ;
line([f1(1,matches(1,:));f2(1,matches(2,:))+shift],[f1(2,matches(1,:));f2(2,matches(2,:))]) ;


end

