function [a,v,p] = calcjTrack(t0,t1,j,a0,v0,p0)

dt = t1-t0;
a = a0+dt*j;
v = v0+dt*a0+0.5*dt^2*j;
p = p0+dt*v0+0.5*dt^2*a0+(1/6)*dt^3*j;