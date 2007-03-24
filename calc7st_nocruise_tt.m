function [t_res,j_res] = calc7st_nocruise_tt(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0)

% [T-T] profile
% We will with calculating the time amount dt, which we would need to cut out
% off the two a=amax phases in order to reach the target exactly.
v3 = v0+a0*(-a0+amax*dir)/jmax/dir+1/2/jmax/dir*(-a0+amax*dir)^2+amax*dir*(-1/amax/dir*v0+1/2/amax/dir^2*a0^2/jmax-1/jmax*amax+1/amax*vmax);
t3 = t(3)+t(5);
dt = 1/2/amax/dir*(t3*amax*dir+2*v3-dir*(t3^2*amax^2+4*t3*amax*dir*v3+4*v3^2+4*amax*dir*dp)^(1/2));
	
% Now we compare dt to the length of the two a=amax phases.
if ((dt <= t(2)) && (dt <= t(6)))
    t_res = [t(1), t(2)-dt, t(3), t(4), t(5), t(6)-dt, t(7)];
	j_res = j;
else
    t_res = zeros(1,7);
    j_res = zeros(1,7);
end