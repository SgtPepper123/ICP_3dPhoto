% Exervice 2
%
close all;

load('Points2.mat');
XYZtmp = 29.5*XYZ;
IMG_NAME = 'images/image002.jpg';

%This function displays the calibration image and allows the user to click
%in the image to get the input points. Left click on the chessboard corners
%and type the 3D coordinates of the clicked points in to the input box that
%appears after the click. You can also zoom in to the image to get more
%precise coordinates. To finish use the right mouse button for the last
%point.
%You don't have to do this all the time, just store the resulting xy and
%XYZ matrices and use them as input for your algorithms.
%[xy XYZ] = getpoints(IMG_NAME,xy,XYZ);

figure(10);
img = imread(IMG_NAME);
imshow(img);
hold on
n = size(xy,2);
for i = 1:n
    plot(xy(1,i),xy(2,i),'ro');
end

% === Task 2 DLT algorithm ===

[K, R, t, error] = runDLT(xy, XYZtmp)

%error plot
figure(15);
hold on
n = size(xy,2);
P = K*[R,-R*t];
for i = 1:n
    tmp = P*[XYZtmp(:,i);1];
    a = xy(:,i);
    b = (tmp(1:2)/tmp(3));
    err = a - b;
    plot(err(1),err(2),'x');
end

% === Task 3 Gold Standard algorithm ===

figure(20);
imshow(img);
hold on
n = size(xy,2);
for i = 1:n
    plot(xy(1,i),xy(2,i),'ro');
end
[K, R, t, error] = runGoldStandard(xy, XYZtmp)

% === Bonus: Gold Standard algorithm with radial distortion estimation ===

%[K, R, t, error] = runGoldStandardRadial(xy, XYZ);

