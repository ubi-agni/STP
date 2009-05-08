% Copyright 2009 Erik Weitnauer, Robert Haschke
%
% This file is part of Smooth Trajectory Planner for Matlab.
%
% Smooth Trajectory Planner for Matlab is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Smooth Trajectory Planner for Matlab is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Smooth Trajectory Planner for Matlab.  If not, see <http://www.gnu.org/licenses/>.
function [DeltaT, T5, T7] = calc7st_opt_shift (t,j,dir, jmax,amax, a2,v2)

% Given a deceleration - deceleration profile with wedge-shaped second part, 
% compute the period DeltaT which must be cut from third phase (and
% inserted in second part), such that the second part becomes a triangular
% profile exactly hitting -d amax.

% 1) compute a3 and a6
[a3, v3, dummy] = calcjTrack(t(3),j(3), a2, v2, 0);
[a6, v6, dummy] = calcjTracks(t(4:6),j(4:6), a3, v3, 0);

% 2) compute discriminant of quadratic polynomial solution
diskriminant = 4*amax^2 + 2*jmax^2*(t(7)^2 - t(5)^2) + 4*dir*jmax*(a6*t(7) + a3*t(5)) + 2*a3^2;
if (isZero(diskriminant))
    diskriminant = 0;    
end
root = sqrt(diskriminant);

% 3) compute T5
if (dir < 0) T5 = dir * amax + root/2;
else T5 = dir * amax - root/2; end
T5 = T5 / (dir * jmax);

% 4) compute DeltaT and T7
DeltaT = (a3 + dir * amax) / (dir * jmax) - T5;
if (isZero(DeltaT))
    DeltaT = 0;
end
T7 = amax / jmax;