function [aEQ, vEQ, pEQ, TEQ, Ts, VARS] = stp7_formulas_solveit(t, j, bDoubleDec)
% Generate formulars from given profile dt, j
% returns equations for a, v, p as well as variables T

ptarget = sym('ptarget','real');
jmax = sym('jmax','real');
amax = sym('amax','real');
vmax = sym('vmax','real');
a0 = sym('a0','real');
v0 = sym('v0','real');
p0 = sym('p0','real');
d_c = sym('d_c','real');
d_a = sym('d_a','real');
if (bDoubleDec) d_ddec = -1;
else d_ddec = 1; end
N=length(t);

if (~bDoubleDec && t(4) == 0) t(5) = 0; end  % for normal profiles we combine phases 3 and 5, if t(4)=0

% build system of equations covering continuous evolution of a and v
% a(i), v(i) and p(i) represent symbolic expressions denoting the reached
% acceleration, velocity and position *after* phase i
% Because MatLab can't handle zero indices (e.g. a(0)) we distinguish
% between i==1 and greater.
aEQ=[]; vEQ=[]; TEQ(1) = sym('0'); Ts=[]; VARS=[];
for i=1:N
    if (t(i) == 0) % as t(i) == 0, values do not change
        if (i == 1) 
            a(i) = sym(a0);
            v(i) = sym(v0);
            p(i) = sym(p0);
        else
            % normally use previous expressions
            a(i) = a(i-1);
            v(i) = v(i-1);
            p(i) = p(i-1); 
            % for a nicer display we use variables
            a(i) = sym(sprintf('a%d',i-1));
            v(i) = sym(sprintf('v%d',i-1));
        end
        % add equation: ti = 0
        % attention: the equation is inserted at the corresponding index,
        % which allows a later overwriting (or removal) of this equation.
        TEQ(i) = sym(sprintf('t%d=0', i));
        % add variable ti
        Ts = [Ts sym(sprintf('t%d', i))];
    else
        T = sym(sprintf('t%d', i), 'real'); 
        if (i==1) J=sym(d_ddec*d_c*d_a*jmax); end
        if (i==2) J=sym(0); end
        if (i==3) J=sym(-d_ddec*d_c*jmax); end
        if (i==4) J=sym(0); end
        if (i==5) J=sym(-d_c*jmax); end
        if (i==6) J=sym(0); end
        if (i==7) J=sym(d_c*jmax); end

        % new time variable t_i
        Ts = [Ts T];
        
        if (i == 1)
            A0 = a0; V0 = v0; P0 = p0;
        else
            A0 = a(i-1); V0 = v(i-1); P0 = p(i-1);
            % for a nicer display we use variables
            A0 = sym(sprintf('a%d',i-1));
            V0 = sym(sprintf('v%d',i-1));
        end
        a(i) = A0 + J * T;
        v(i) = V0 + T * (A0 + 1/2 * J * T);
        p(i) = P0 + T * (V0 + T * (1/2 * A0 + 1/6 * J * T));
    end
    
    % add equations
    if (i ~= 7)
        aEQ = [aEQ sym(sprintf('a%d = %s', i, char(a(i))))];
        vEQ = [vEQ sym(sprintf('v%d = %s', i, char(v(i))))];
        VARS = [VARS sym(sprintf('a%d',i)) sym(sprintf('v%d',i))];
    end
end

%%% add additional equations %%%
pEQ=addEQ ([], ptarget, p(N)); % reach target
% we end at zero speed and acceleration
aEQ = addEQ (aEQ, 0, a(7));
vEQ = addEQ (vEQ, 0, v(7));

% the following things work only for "correct" 7-phases profiles
if (N==7)
    a1 = 'a1'; a5 = 'a5'; a3 = 'a3';
    
    % if we have a cruise phase, a3 must be zero
    if (t(4) ~= 0) aEQ = addEQ (aEQ, a3, 0); end

    if (bDoubleDec) % double deceleration
        % add max limits for second trapezoidal profile
        if (t(6) ~= 0) aEQ = addEQ (aEQ, a5, -d_c*amax); end
        % for first trapezoidal profile we add the limit as an equation
        % for t(1) because it actually fixes t(1)
        % this can be overwritten by calling routine if neccessary
        if (t(2) ~= 0) TEQ(1) = sym(strcat(char(sym(a1)),'=',char(sym(-d_c*amax)))); end
    else % normal profile
        % add max limits for trapezoidal profiles (and remove corr. variable)
        if (t(2) ~= 0 && t(1) ~= 0) aEQ = addEQ (aEQ, a1,  d_c*amax); end
        if (t(6) ~= 0) aEQ = addEQ (aEQ, a5, -d_c*amax); end
    end
end
return

function EQ = addEQ (EQ, lhs, rhs)
% Function to add a further equations to the set of equations EQ
% Both lhs and rhs can be symbols or strings
EQ = [EQ sym(strcat(char(sym(lhs)), '=', char(sym(rhs))))];
return
