function alignment(image_filename, ply_filename, points_filename)

%clear all;
%close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Alignment tool
% Input ply format should be [ud, vd, X, Y, Z, r, g, b, nx, ny, nz, I]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%display('Choose the input image...')
%[ImgName, PathName] = uigetfile({'*.jpeg;*.jpg;*.png', 'Image Files (*.jpeg,*.jpg,*.png)'},'Select the jpeg image file');
%img = imread(sprintf('%s/%s',PathName,ImgName));
img = imread(image_filename);

%display('Choose the input ply file...')
%[PlyName, PathName] = uigetfile({'*.ply', 'PLY File (*.ply)'},'Select the PLY file');
%ply = simple_read_ply(sprintf('%s/%s',PathName,PlyName));
ply = simple_read_ply(ply_filename);

if size(ply,2) ~= 12
    error('Invalid File Format!!')
end

cam_center = [0 0 0]; %ply(1, 3:5);
pos =  ply(1:end,3:5);

depth = bsxfun(@minus, pos, cam_center);
depth = (sqrt(sum((depth.^2)')))';

h = max(ply(1:end,1));
w = max(ply(1:end,2));
index = (ply(1:end,2)-1)*h + ply(1:end,1);

depthmap = zeros(h,w);
depthmap(index) = depth/max(depth(:));

%% clicking
display('Plese indicate the corresponding points (ESC: compute the projection matrix, BS: remove the points from the list)')
xlist1 = [];
xlist2 = [];
count = 1;
while(1)
figure(1), imagesc(min(depthmap,0.5)), colormap(jet(256)), axis equal;
title(sprintf('%d-th point',count))

if count >= 2
    hold on
    plot(xlist1(:,1),xlist1(:,2),'go')
    hold off
end

figure(2), imshow(img), axis equal;
title(sprintf('%d-th point',count))

if count >= 2
    hold on
    plot(xlist2(:,1),xlist2(:,2),'go')
    hold off
end

figure(1),
[X,Y, key] = ginput(1);


if key == 27
    display(sprintf('%d pairs are selected!!...',count-1))
    break;
end

if key == 8
    xlist1 = xlist1(1:end-1,:);
    xlist2 = xlist2(1:end-1,:);
    display(sprintf('%d th points are removed from the lists', count-1))
    count = max(1, count - 1);
    continue;    
end

xlist1(count,1) = floor(X);
xlist1(count,2) = floor(Y);

hold on
plot(X,Y,'ro')
hold off

figure(2),
[X,Y, key] = ginput(1);

if key == 27
    display(sprintf('%d pairs are selected!!...',count-1))
    break;
end

if key == 8
    xlist1 = xlist1(1:end-1,:);
    display(sprintf('%d th points are removed from the lists', count-1))
    continue;    
end

xlist2(count,1) = floor(X);
xlist2(count,2) = floor(Y);

hold on
plot(X,Y,'go')
hold off

display(sprintf('%d th point pair : [%d %d] and [%d %d]', count, xlist1(count,1), xlist1(count,2), xlist2(count,1), xlist2(count,2)))
count = count + 1;
end

%% save file
A = zeros(count-1,5);
for i = 1 : count-1
   depth_index = find(index==(xlist1(i,1)-1)*h+xlist1(i,2));
   A(i,1) = pos(depth_index,1);
   A(i,2) = pos(depth_index,2);
   A(i,3) = pos(depth_index,3);
   A(i,4) = xlist2(i,1);
   A(i,5) = xlist2(i,2);  
end

%[file,path] = uiputfile('points.txt','Save file name');
%fid = fopen(sprintf('%s/%s',path,file),'w');
fid = fopen(points_filename, 'w');
fprintf(fid, '%g %g %g %g %g\n', A');
fclose(fid);
close all
