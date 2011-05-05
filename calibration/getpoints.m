function [xy XYZ] = getpoints(filename,xy_in,XYZ_in)
img = imread(filename);
image(img);
%axis off
axis image
% Initially, the list of points is empty.
xy = xy_in;
XYZ = XYZ_in;
n = size(xy_in,2)
% Loop, picking up the points.
disp('Left mouse button picks points.')
disp('Right mouse button picks last point.')
but = 1;
zoom on;
while but == 1
    %keyboard;
    [xi, yi, but] = ginput(1);
    hold on;
    plot(xi, yi, 'ro')
    hold off;
    n = n+1;
    xy(:, n) = [xi; yi]; % add a new column with the current values
    input = inputdlg('[X Y Z]'); % show input dialog
    XYZi = str2num(input{1}); % convert to number
    XYZ(:, n) = XYZi; % add a new column with the current values
end
end