%% INPUT: filename: Filename to save.
%%        colored_point_cloud: N X 6 matrix
%%        i-th row is: [X_i, Y_i, Z_i, R_i, G_i, B_i] where R_i, G_i, B_i \in [0,255].
%% Authors: D.Kottas, L.Carrillo
%% UMN May 2014
function save_ply(filename,colored_point_cloud)
assert(size(colored_point_cloud,1)>=1)
assert(size(colored_point_cloud,2)==6)
%% Write PLY header.
file_io = fopen(filename,'w');
fprintf(file_io,'ply\n');
fprintf(file_io,'format ascii 1.0\n');
fprintf(file_io,'comment PCL generated\n');
fprintf(file_io,'element vertex %d\n',size(colored_point_cloud,1));
fprintf(file_io,'property float x\n');
fprintf(file_io,'property float y\n');
fprintf(file_io,'property float z\n');
fprintf(file_io,'property uchar red\n');
fprintf(file_io,'property uchar green\n');
fprintf(file_io,'property uchar blue\n');
fprintf(file_io,'element camera 1\n');
fprintf(file_io,'property float view_px\n');
fprintf(file_io,'property float view_py\n');
fprintf(file_io,'property float view_pz\n');
fprintf(file_io,'property float x_axisx\n');
fprintf(file_io,'property float x_axisy\n');
fprintf(file_io,'property float x_axisz\n');
fprintf(file_io,'property float y_axisx\n');
fprintf(file_io,'property float y_axisy\n');
fprintf(file_io,'property float y_axisz\n');
fprintf(file_io,'property float z_axisx\n');
fprintf(file_io,'property float z_axisy\n');
fprintf(file_io,'property float z_axisz\n');
fprintf(file_io,'property float focal\n');
fprintf(file_io,'property float scalex\n');
fprintf(file_io,'property float scaley\n');
fprintf(file_io,'property float centerx\n');
fprintf(file_io,'property float centery\n');
fprintf(file_io,'property int viewportx\n');
fprintf(file_io,'property int viewporty\n');
fprintf(file_io,'property float k1\n');
fprintf(file_io,'property float k2\n');
fprintf(file_io,'end_header\n');
fclose(file_io);
%% Write point cloud.
dlmwrite(filename,colored_point_cloud,'delimiter',' ','-append');
%% Write PLY footer.
file_io = fopen(filename,'a+');
fprintf(file_io,'0 0 0 1 0 0 0 1 0 0 0 1 0 0 0 0 0 %d 1 0 0\n',size(colored_point_cloud,1));
fclose(file_io);
end