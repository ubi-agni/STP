function [t,j] = wrapper7st(x0,xtarget,v0,vmax,a0,amax,jmax)
% simple wrapper for testing purposes
% The arguments are given in the same order as in the c++code.
[t,j] = calc7st(xtarget, jmax,amax,vmax,a0,v0,x0,true);

