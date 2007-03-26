function t_res = stp7_solve (EQ1, EQ2, TVARS, T)
% Solve equation set EQ1 for variables T1 = TVARS \ T first and
% subsequently solve equation EQ2 for (single) variable T

T1 = TVARS; T1(find(T1 == T)) = [];
% solve for all other variables
solT1 = eval (solveCmd ([EQ1 T1]));

% in some cases multiple solutions exists (positive and negative)
N = length(getfield(solT1, char(T1(1)))); t_res=[];
for i=1:N
    % try all possible solutions and choose best one
    trial = trySolution (solT1, i, EQ1, EQ2, T1, T);
    if (length(trial) > 0) t_res = trial; end
end

if (length(t_res) == 0) error ('This should not happen: no solution'); end

function t_res = trySolution (solT1, solIdx, EQ1, EQ2, T1, T)
% substitute solutions for variables in T1 in equation EQ2
for i=1:length(T1)
    val = getfield(solT1, char(T1(i)));
%    disp (sprintf('%s: %s', char(T1(i)), char(val(solIdx))));
    EQ2 = subs(EQ2, T1(i), val(solIdx));
end

% solve for single variable T
solT  = solve (EQ2, char(T));

% choose real solutions >= 0
solT = eval(solT);
solT = solT(imag(solT) == 0);
solT = solT(solT >= 0);
solT = sort(solT); % sort in ascending order

% create index vector containing indeces of T1 vars
idxs = [];
for i=1:length(T1);
    varname = char(T1(i)); 
    idxs = [idxs str2num(varname(2))];
end
% index of T variable
varname = char(T); idxT = str2num(varname(2));

% replace values, build and check result vector
t_res = []; % empty set indicates failure
for s=1:length(solT)
    t_res(idxT) = solT(s);
    for i=1:length(T1);
        val = getfield(solT1,char(T1(i)));
        t_res(idxs(i)) = subs (val(solIdx), T, solT(s)); % replace with correct value
        if (t_res(idxs(i)) < 0)
            disp (sprintf ('invalid: %f', t_res(idxs(i))));
            t_res = []; % reset
            break;
        end
    end
    % return first feasible solution
    if (length(t_res) > 0) return; end
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
