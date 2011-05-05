function [xyn, XYZn, T, U] = normalization(xy, XYZ)

%data normalization
%first compute centroid
xy_centroid = mean(xy,2);
XYZ_centroid = mean(XYZ,2);

[a,b] = size(xy);

%then, compute scale
s_xy = xy - xy_centroid*ones(1,b);
s_XYZ = XYZ - XYZ_centroid*ones(1,b);

s_xy = s_xy.^2;
s_XYZ = s_XYZ.^2;

s_xy = sqrt(sum(mean(s_xy,2)));
s_XYZ = sqrt(sum(mean(s_XYZ,2)));

s_xy = s_xy / sqrt(2);
s_XYZ = s_XYZ / sqrt(3);

%create T and U transformation matrices
T = [s_xy 0 xy_centroid(1)
    0 s_xy xy_centroid(2)
    0 0 1]^-1;
U = [s_XYZ 0 0 XYZ_centroid(1)
    0 s_XYZ 0 XYZ_centroid(2)
    0 0 s_XYZ XYZ_centroid(3)
    0 0 0 1]^-1;

%and normalize the points according to the transformations
xyn = ones(3,b);
XYZn = ones(4,b);
for i = 1:b
    tmp1 = T*[xy(:,i);1];
    xyn(:,i) = tmp1(1:3);
    tmp2 = U*[XYZ(:,i);1];
    XYZn(:,i) = tmp2(1:4);
end

end