%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 7-phases STP without cruising phase - Triangle-Trapez profile. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Numerically solve the polynomial for t1.
% We get the polynomial from 'formulars_7stp_DT_bruteforce.m'.
% Test formular from 'formulars_7stp_DT_test_bruteforce.m'.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% in t the 4 times of [D-T] profile overshooting the target by dp.
function [t_res,j_res] = calc7st_nocruise_dt_or_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0)
% test whether we really have a [D-T] profile by calculating p_end for a [D-D] profile with a(t2) = -amax*dir:
% we can only calculate p_end like this, if an imidiate reduction to acc. of zero will leave a speed smaller equal or less
% the speed we can compensate in the (a:0->+-amax->0) triangle part.
% So first, test which formular we should use:
%pend = 1/24*(-24*dir*amax^3+48*v0*jmax*amax-24*a0^2*dir*amax+48*a0*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(1/2)*amax+24*a0^2*dir^3*amax-48*a0*dir^2*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(1/2)*amax+48*a0*amax^2+12*a0^2*dir^3*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(1/2)-24*a0^2*dir*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(1/2)-4*a0^3*dir^4-12*a0^3*dir^2+24*v0*jmax*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(1/2)+24*p0*jmax^2+3*dir*(2*a0^2+4*amax^2*dir^2-4*v0*jmax*dir)^(3/2)-72*v0*jmax*a0*dir-48*a0*dir^4*amax^2+48*amax^3*dir^3-48*amax*dir^2*v0*jmax+48*a0*dir^3*v0*jmax+24*a0^3)/jmax^2
%pend = 0;
%blablabla
[t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
a3 = a0 + j_res(1)*t_res(1) + j_res(2)*(t_res(2)-t_res(1));
if (abs(a3)>amax)
%if (sign(ptarget-pend) == dir)
	% [D-T] profile
	disp(sprintf('[D-T]'));
	[t_res, j_res] = calc7st_nocruise_dt(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
else
	% [D-D] profile
	disp(sprintf('[D-D]'));
	% already called [t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
end