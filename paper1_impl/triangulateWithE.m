function [ points ] = triangulateWithE( pts_1, pts_2, E, K1, K2 )
%triangulate Finds the 3D points given the 2D correspondences and camera
%intrinsics.

M1 = K1 * eye(3,4);

% camera2 generates 4 possible camera projection matrices. We will display
% with each of them.
%M2 = camera2(E', K2);


% figure();
%
% for i = 1:4
%     points = backproject( pts_1, pts_2, M1, M2{i} );
%
%     subplot(2,2,i);
%     plot3(points(:,1), points(:,2), points(:,3), '.');
%     axis equal;
%
% end

M2 = K2 * robustCameraRecovery(E, pts_1, pts_2);
figure();
points = backproject( pts_1, pts_2, M1, M2);
plot3(points(:,1), points(:,2), points(:,3), '.');
axis equal;


end