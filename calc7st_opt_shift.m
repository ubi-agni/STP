function [DeltaT, T5, T7] = calc7st_opt_shift (t,j,dir, jmax,amax, a2,v2)

% Given a deceleration - deceleration profile with wedge-shaped second part, 
% compute the period DeltaT which must be cut from third phase (and
% inserted in second part), such that the second part becomes a triangular
% profile exactly hitting -d amax.

% 1) compute a3 and a6
[a3, v3, dummy] = calcjTrack(t(3),j(3), a2, v2, 0);
[a6, v6, dummy] = calcjTracks(t(4:6),j(4:6), a3, v3, 0);

% 2) compute discriminant of quadratic polynomial solution
root = sqrt(4*amax^2 + 2*jmax^2*(t(7)^2 - t(5)^2) + 4*dir*jmax*(a6*t(7) + a3*t(5)) + 2*a3^2);

% 3) compute T5
if (dir < 0) T5 = dir * amax + root/2;
else T5 = dir * amax - root/2; end
T5 = T5 / (dir * jmax);

% 4) compute DeltaT and T7
DeltaT = (a3 + dir * amax) / (dir * jmax) - T5;
T7 = amax / jmax;