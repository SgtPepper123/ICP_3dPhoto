function f = fminGoldStandard(p, xy, XYZ, w)

%reassemble P
P = [p(1:4);p(5:8);p(9:12)];

K = decompose(P);

alpha = K(1,1) - K(2,2);

skew = K(1,2);

%compute squared geometric error
n = size(xy,2);
error = 0;
for i = 1:n
    bla = P*[XYZ(:,i)];
    tmp = [xy(1:2,i)] - (bla(1:2)/bla(3));
    error = error + tmp'*tmp;
end

%compute cost function value
f = error  + w*alpha.^2 + w*skew.^2;
end