function t_res = stp7_solve (EQ1, EQ2, TVARS, T)
% Solve equation set EQ1 for variables T1 = TVARS \ T first and
% subsequently solve equation EQ2 for (single) variable T

T1 = TVARS; T1(find(T1 == T)) = [];
% solve for all other variables
solT1 = eval (solveCmd ([EQ1 T1]));

% in some cases multiple solutions exists (positive and negative)
val = getfield(solT1, char(T1(1)));
if (length(val) == 1) solIdx=1;
else
    % simply choose solution whose value doesn't start with '-'
    s=char(val(1));
    if (s(1) ~= '-') solIdx=1; else solIdx=2; end
end

% substitute solutions for variables in T1 in equation EQ2
for i=1:length(T1)
    val = getfield(solT1, char(T1(i)));
%    disp (sprintf('%s: %s', char(T1(i)), char(val(solIdx))));
    EQ2 = subs(EQ2, T1(i), val(solIdx));
end

% solve for single variable T
solT  = solve (EQ2, char(T));

% Take the smallest real solution > 0
t = 0;
for i=1:length(solT)
    val = eval(solT(i));
	if (isreal(val) && (val > 0) && ((val < t) || (t == 0)))
		t = val;
	end
end

% Replace values and build result vector
varname = char(T); idx = str2num(varname(2));
t_res(idx) = t;
for i=1:length(T1);
    varname = char(T1(i)); idx=str2num(varname(2));
    val = getfield(solT1, varname);
    t_res(idx) = subs (val(solIdx), T, t); % replace with correct value
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
