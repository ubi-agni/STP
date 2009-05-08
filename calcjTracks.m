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
% t = (t1 t2 ... tn) ... time-intervalls for the n phases
% j = (j1 j2 ... jn) ... jerks for the n phases
function [a,v,p] = calcjTracks(t,j,a0,v0,p0)

a=a0;
v=v0;
p=p0;

for i=1:length(t)
    if (t(i) ~= 0)
        [a,v,p] = calcjTrack(t(i),j(i),a,v,p);
    end
end