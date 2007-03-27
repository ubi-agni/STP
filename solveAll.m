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
    if (~isreal (solVec)) continue; end % skip imaginary solution
    if (length(solVec(solVec >= 0)) ~= N) continue; end % skip negative values
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
