function [ output_args ] = loadImages( string_name )
%UNTITLED2 Summary of this function goes here
%   read *.jpg
    filedir = string_name;
    imagefiles = dir([filedir '*.png']);
    N = length(imagefiles);
    
    output_args = cell(N,1);
    for i = 1:N
        currentfilename = imagefiles(i).name;
        output_args{i} = imread([filedir currentfilename]);
    end
     sprintf('load %d images',N);

end

