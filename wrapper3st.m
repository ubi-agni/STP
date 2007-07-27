function [t,j] = wrapper3st(x0,xtarget,v0,vmax,amax)
% simple wrapper for testing purposes
% The arguments are given in the same order as in the c++code.
[t,a] = calc3st(xtarget, amax,vmax,v0,x0);
plotaTracks(t,a,amax,vmax,xtarget,v0,x0,true,true);