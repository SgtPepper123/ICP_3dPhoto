function [ K, R, t ] = decompose(P)
%decompose P into K, R and t
M = P(1:3,1:3)^-1;
[R,K] = qr(M);
 
R = R^-1;
K = K^-1;

[u,s,v] = svd(P);
 
%dehomogenise t
t = v(1:3,4)/v(4,4);

end