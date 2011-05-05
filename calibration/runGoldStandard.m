function [K, R, t, error] = runGoldStandard(xy, XYZ)

%normalize data points
[xy_normalized,XYZ_normalized,T,U] = normalization(xy,XYZ);

%compute DLT
[Pn] = dlt(xy_normalized, XYZ_normalized);

%minimize geometric error
pn = [Pn(1,:) Pn(2,:) Pn(3,:)];
for i=1:20
    [pn] = fminsearch(@fminGoldStandard, pn, [], xy_normalized, XYZ_normalized, i/5);
end

P = [pn(1:4);pn(5:8);pn(9:12)];

%denormalize camera matrix
P = T^-1*P*U;

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

end