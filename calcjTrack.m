function [a,v,p] = calcjTrack(t,j,a0,v0,p0)

a = a0+t*j;
v = v0+t*a0+0.5*t^2*j;
p = p0+t*v0+0.5*t^2*a0+(1/6)*t^3*j;