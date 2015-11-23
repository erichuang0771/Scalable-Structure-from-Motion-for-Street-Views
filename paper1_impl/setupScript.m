% Setup script

clear;

load('corresp_7pt.mat');

% Taken from the camera parameters file in the Middlebury data set
% See http://vision.middlebury.edu/mview/data/
load('intrinsics.mat');

EList = five_point(pts_1, pts_2, K1, K2);

load('many_corresp.mat');

for i = 1:length(EList)
    E = EList{i};
    
    P = triangulateWithE(pts_1, pts_2, E, K1, K2);
    figure()
    plot3(P(:,1), P(:,2), P(:,3), '.')
end

%demonstrate camera matrix recovery
M2 = robustCameraRecovery(EList{1}, pts_1,pts_2)
cameraCenter = computeCameraCenter(M2)