%function [t,j] no_cruise_full(p_target,,jmax,amax,vmax,a0,v0,p0)

% we set p_target > p_0
% it is a0 <= amax (otherwise there would be a cruising phase

%t1 = (amax-a0)/jmax;
%t2 = (a0^2-2*amax^2)/(2*jmax*amax) + (vmax-v0)/amax;
%t3 = 2*amax/jmax;
%t4 = vmax/amax - amx/jmax;
%t5 = amax/jmax;


% input variables
dir = sym('dir');
a0 = sym('a0');
v0 = sym('v0');
p0 = sym('p0');
vmax = sym('vmax');
amax = sym('amax');
jmax = sym('jmax');
ptarget = sym('ptarget');

% the values we need to calculate
t1 = sym('t1');
t2 = sym('t2');
t3 = sym('t3');
t4 = sym('t4');
t5 = sym('t5');
t6 = sym('t6');
t7 = sym('t7');

% setting the jerk pulses
j1 = jmax*dir;
j2 = 0;
j3 = -jmax*dir;
j4 = 0;
j5 = jmax*dir;

% before phase 1
a1 = a0;
v1 = v0;
p1 = p0;

% before phase 2
a2 = a1 + j1*t1;
v2 = v1 + a1*t1 + (1/2)*j1*t1^2;
p2 = p1 + v1*t1 + (1/2)*a1*t1^2 + (1/6)*j1*t1^3;
% equations
eq1 = sym(strcat(char(a2),' = amax*dir'));
t1 = simple(eval(solve(eq1,'t1')))
a2 = simple(eval(a2))

% before phase 3
a3 = a2 + j2*t2;
v3 = v2 + a2*t2 + (1/2)*j2*t2^2
p3 = p2 + v2*t2 + (1/2)*a2*t2^2 + (1/6)*j2*t2^3;

% before phase 4
a4 = a3 + j3*t3;
v4 = v3 + a3*t3 + (1/2)*j3*t3^2;
p4 = p3 + v3*t3 + (1/2)*a3*t3^2 + (1/6)*j3*t3^3;
vm = v3 + a3*(t3/2) + (1/2)*j3*(t3/2)^2;
% equations:
eq2 = sym(strcat(char(a4),' = -amax*dir'));
t3 = (eval(solve(eq2,'t3')))
a4 = simple(eval(a4));
eq3 = sym(strcat(char(vm),' = vmax*dir'));
t2 = simple(eval(solve(eq3,'t2')))
vm = simple(eval(vm))

% before phase 5
a5 = a4 + j4*t4;
v5 = v4 + a4*t4 + (1/2)*j4*t4^2;
p5 = p4 + v4*t4 + (1/2)*a4*t4^2 + (1/6)*j4*t4^3;

% finish
a6 = a5 + j5*t5;
v6 = v5 + a5*t5 + (1/2)*j5*t5^2;
p6 = p5 + v5*t5 + (1/2)*a5*t5^2 + (1/6)*j5*t5^3;
% equations
eq4 = sym(strcat(char(a6),' = 0'));
t5 = simple(eval(solve(eq4,'t5')))
a6 = simple(eval(a6))
eq5 = sym(strcat(char(v6),' = 0'));
t4 = simple(eval(solve(eq5,'t4')))
v6 = simple(eval(v6))
p6 = simple(eval(p6));
eq6 = sym(strcat(char(p6),' = ptarget'));
vmax = simple(eval(solve(eq6,'vmax')))
vmax = vmax(1);
p6 = simple(eval(p6))
v2