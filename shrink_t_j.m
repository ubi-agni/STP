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
function [t_res, j_res] = shrink_t_j(t, j)

% remove zero indices
nonzero = find(abs(t) > 1.e-10);
t = t(nonzero);
j = j(nonzero);

% merge identical phases
for i=1:length(t)-1
    if (j(i) == j(i+1))
        % merge
        t(i) = t(i)+t(i+1);
        t(i+1) = 0;
    end
end

% remove zero indices again
nonzero = find(t ~= 0);
t_res = t(nonzero);
j_res = j(nonzero);
