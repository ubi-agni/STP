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
function t_res = solveAll (EQ, VARS) 
% Solve equations EQ for vars TVARS
sol = eval (solveCmd ([EQ(EQ ~= 0) VARS]));
% convert into solution matrix: each row is a single solution
solMatrix = []; N=length(VARS);
for v=1:N
    solMatrix = [solMatrix eval(getfield(sol, char(VARS(v))))];
end

t_res=[]; numSolutions=size(solMatrix); numSolutions=numSolutions(1);
for s=1:numSolutions
    solVec = solMatrix(s,1:N);
    % skip imaginary solutions, but allow small numeric variations
    if (length(solVec(abs(imag(solVec)) < 1.0e-10)) ~= N) continue; end
    % use real parts only
    solVec = real(solVec);
    % clip small negative values to zero
    solVec(find(abs(solVec) <= 1e-10)) = 0; 
    % skip negative values
    if (length(solVec(solVec >= 0)) ~= N) continue; end 
    % store result with smallest sum
    if (length(t_res) == 0 || sum(t_res) > sum(solVec))
        t_res = solVec;
    end
end
if (length(t_res) == 0) 
    solMatrix
    error ('This should not happen: no solution');
end
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function command = solveCmd (args)
% From set of equations A, V, P with variables T and vars, 
% build an expression which can be used to solve the system of equations
command = 'solve(';
for i=1:length(args)
%    disp (char(args(i)));
    command = strcat(command, '''', char(args(i)), ''',');
end
command(length(command)) = ')';
return
