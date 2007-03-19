%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 7-phases STP without cruising phase - Trapez-Triangle profile. %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Numerically solve the polynomial for t4.
% We get the polynomial from 'formulars_7stp_TD_bruteforce.m'.
% Test formular from 'formulars_7stp_TD_test_bruteforce.m'.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [t_res,j_res] = calc7st_nocruise_td_or_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0)
% test whether we really have a [T-D] profile by calculating p_end for a [D-D] profile with a(t1) = amax*dir:
%pend = 1/12*(4*a0^3-6*dir^4*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)*v0*jmax+3*dir^3*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)*a0^2-6*dir^5*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)*amax^2+12*amax^3*dir^3+24*v0*jmax*dir^2*amax-12*v0*jmax*dir*a0+12*v0*jmax*dir^2*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)+12*p0*jmax^2*dir^2-12*a0^2*amax*dir+12*amax^2*dir^3*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)-6*dir*a0^2*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2))/jmax^2/dir^2

% can be imaginaer number, because sometimes we need a triangle bigger than (a=amax) in the
% first part of the profile to ensure that both speed and acc are 0 at the end of the motion. 
% pend = 1/12*(4*a0^3-6*dir^4*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2)*v0*jmax+3*dir^3*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2)*a0^2-6*dir^5*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2)*amax^2+12*amax^3*dir^3+24*v0*jmax*dir^2*amax-12*v0*jmax*dir*a0+12*v0*jmax*dir^2*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2)+12*p0*jmax^2*dir^2-12*a0^2*amax*dir+12*amax^2*dir^3*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2)-6*dir*a0^2*(4*v0*jmax-2*a0^2+4*amax^2*dir^2)^(1/2))/jmax^2/dir^2


%if (sign(ptarget-pend) == dir)

% There can occour the following complication:
% If we call the DD-case for testing if the restrictions on the acceleration
% are met, it's possible that - in the case we overshoot the target with the
% TD-profile this is not the case with the DD-profile, because it can slow
% down faster.
% We need to check for this!
[t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
a2 = a0+t_res(1)*j_res(1);
if (abs(a2) > amax)
	% [T-D] profile
	disp(sprintf('[T-D]'));
	[t_res,j_res] = calc7st_nocruise_td(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
else
	% [D-D] profile
	disp(sprintf('[D-D]'));
	%already called: [t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
end