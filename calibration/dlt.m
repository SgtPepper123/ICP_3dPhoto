function [P] = dlt(xy, XYZ)
%computes DLT, xy and XYZ should be normalized before calling this function

[a,b] = size(XYZ);

XYZ_trans = XYZ';

x = xy(1,:)'*ones(1,a);
y = xy(2,:)'*ones(1,a);

A = [XYZ_trans zeros(b,a) x.*(-XYZ_trans);
    zeros(b,a) -XYZ_trans y.*XYZ_trans];

[U,S,V] = svd(A);

P = reshape(V(:,12),4,3)';