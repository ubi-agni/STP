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
function [isPossible, t_res] = removeArea(t,deltaV,jmax,amax)
% Takes a vector of three time intervalls and then
% deletes the passed deltaV from the area under the acceleration graph.

% We only decrease the area...
deltaV = abs(deltaV);
A_now = t(1)^2*jmax+amax*t(2);
if (A_now < deltaV)
    isPossible = false;
    t_res = t;
    return;
end
isPossible = true;
Aw_max = amax^2/jmax;
        
if (isZero(t(2)) || (Aw_max > A_now - deltaV))
    % result wedge shaped
    t_res(1) = sqrt(A_now/jmax - deltaV/jmax);
    t_res(3) = t_res(1);
    t_res(2) = 0;
else
    % result trapezoid shaped
    t_res = t;
    t_res(2) = t_res(2) - deltaV/amax;
end
