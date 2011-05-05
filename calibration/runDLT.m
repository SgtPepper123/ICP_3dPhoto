function [K, R, t, P, error] = runDLT(xy, XYZ)

%normalize data points
[xy_normalized,XYZ_normalized,T,U] = normalization(xy,XYZ);

%unnormalized version comment above - uncomment below
%xy_normalized = [xy;ones(1,size(xy,2))];
%XYZ_normalized = [XYZ;ones(1,size(XYZ,2))];
%T = diag(diag(ones(3)));
%U = diag(diag(ones(4)));

%compute DLT
[P_normalized] = dlt(xy_normalized, XYZ_normalized);

%denormalize camera matrix
P = T^-1*P_normalized*U;

%factorize camera matrix in to K, R and t
[K,R,t] = decompose(P);

%compute reprojection error

n = size(xy,2);
error = 0;
for i = 1:n
    tmp = P*[XYZ(:,i);1];
    tmp2 = tmp(1:2)/tmp(3);
    hold on;
    plot(tmp2(1),tmp2(2),'b*');
    tmp2 = [xy(:,i)] - tmp2;
    error = error + tmp2'*tmp2;
end
