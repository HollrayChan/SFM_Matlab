%% read camera Intrinsic and print
load('C:\Users\A-206\Desktop\test4\data'); %input the path
fid=fopen('camera_intrinsics_per_view.txt','w');
fdxy = cameraParams.IntrinsicMatrix(1,1);
u = cameraParams.IntrinsicMatrix(3,1);
v = cameraParams.IntrinsicMatrix(3,2);
vn = length(camPoses.ViewId);
for i = 1:vn
   fprintf(fid,'%3.3f\r\n',fdxy);
   fprintf(fid,'%3.3f\r\n',u);   
   fprintf(fid,'%3.3f\r\n',v); 
end
%% read tracks and get camera motion
fid=fopen('motion.txt','w'); 
fprintf(fid,'header:\r\n');
fprintf(fid,'  seq: 0\r\n');
fprintf(fid,'  stamp: 0.000000000 \r\n');
fprintf(fid,'  frame_id: 0\r\n');
fprintf(fid,'poses[]:\r\n');
for i = 1:vn
    p = camPoses.Location{i};
    px = p(1,1);
    py = p(1,2);
    pz = p(1,3);
    n = i-1;
    fprintf(fid,'  poses[%i]:\r\n',n);
    fprintf(fid,'    position:\r\n');
    fprintf(fid,'      x: %.6f \r\n',px);
    fprintf(fid,'      y: %.6f \r\n',py);
    fprintf(fid,'      z: %.6f \r\n',pz);
    %欧拉角转换
    q = dcm2quat(camPoses.Orientation{i});
    ow = q(1,1);
    oy = q(1,2);
    oz = q(1,3);
    ox = q(1,4);
    fprintf(fid,'    orientation:\r\n');
    fprintf(fid,'      x: %.6f\r\n',ox);
    fprintf(fid,'      y: %.6f\r\n',oy);
    fprintf(fid,'      z: %.6f\r\n',oz);
    fprintf(fid,'      w: %.6f\r\n',ow);
end
fclose(fid);
%% get the view involving restructure
fid=fopen('view_indexes_per_point.txt','w'); 
a = -1;
[l,num] = size(tracks);
for i = 1:num
   fprintf(fid,'%i\r\n',a);
   [b,c] = size(tracks(1,i).ViewIds); %得到数组的大小
   for j = 1:c
   I = tracks(1,i).ViewIds(1,j);
   fprintf(fid,'%i\r\n',I);
   end 
end
fclose(fid);
%% selected_indexes
fid=fopen('selected_indexes.txt','w');
for i = 1:vn
    fprintf(fid,'%i\r\n',i);
end
fclose(fid);
%% structure_filtered.txt
fid=fopen('structure_filtered.txt','w');
fprintf(fid,'ply\r\n');
fprintf(fid,'format ascii 1.0\r\n');
fprintf(fid,'element vertex %i\r\n',num);
fprintf(fid,'property float x\r\n');
fprintf(fid,'property float y\r\n');
fprintf(fid,'property float z\r\n');
fprintf(fid,'end_header\r\n');
dlmwrite('structure_filtered.txt',xyzPoints,'-append',...
'delimiter',' ','precision','%.6f')
fid=fopen('structure_filtered.txt','a+');
fclose(fid);
%% copy the same data
copyfile('selected_indexes.txt','visible_view_indexes_filtered');
copyfile('selected_indexes.txt','selected_indexes');
copyfile('selected_indexes.txt','visible_view_indexes');
copyfile('selected_indexes.txt','selected_indexes');
copyfile('view_indexes_per_point.txt','view_indexes_per_point_filtered');
copyfile('camera_intrinsics_per_view.txt','camera_intrinsics_per_view');
copyfile('motion.txt','motion.yaml');
copyfile('structure_filtered.txt','structure_filtered.ply');
delete('motion.txt');
delete('selected_indexes.txt');
delete('camera_intrinsics_per_view.txt');
delete('structure_filtered.txt');
delete('view_indexes_per_point.txt');
delete('view_indexes_per_point_filtered.txt');
delete('visible_view_indexes.txt');
delete('visible_view_indexes_filtered.txt');

%% photoes rename
path ='C:\Users\A-206\Desktop\test4\';%把路径设置为这个文件夹 很重要
files= dir(strcat(path,'*.jpg'));
for i=1:length(files)    
    oldname =files(i).name;%取出第一个文件的名称         
    str = sprintf('%08d.jpg',i);%类似‘0001’为前缀的命名方式，数字依次递增     
    newname = [ str ]   
    eval(['!rename',' "',oldname,'" ',newname])
end