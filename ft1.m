function [H,w]=ft1(x,n,NT)
k=0:NT;
w=(pi/abs(max(k)/2))*k;
H=x*(exp(-j*pi/abs(max(k)/2))).^(n'*k);