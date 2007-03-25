% input variables
a0 = sym('a0');
v0 = sym('v0');
p0 = sym('p0');
vmax = sym('vmax');
amax = sym('amax');
jmax = sym('jmax');
ptarget = sym('ptarget');
dir = sym('dir');

% the values we need to calculate
t1 = sym('t1');
t2 = sym('t2');
t3 = sym('t3');

% setting the jerk pulses
j1 = jmax*dir;
j2 = -jmax*dir;
j3 = jmax*dir;

% before phase 1
a1 = a0;
v1 = v0;
p1 = p0;

% before phase 2
a2 = a1 + j1*t1;
v2 = v1 + a1*t1 + (1/2)*j1*t1^2;
p2 = p1 + v1*t1 + (1/2)*a1*t1^2 + (1/6)*j1*t1^3;

% before phase 3
a3 = a2 + j2*t2;
v3 = v2 + a2*t2 + (1/2)*j2*t2^2;
p3 = p2 + v2*t2 + (1/2)*a2*t2^2 + (1/6)*j2*t2^3;

% finish
a4 = a3 + j3*t3;
v4 = v3 + a3*t3 + (1/2)*j3*t3^2;
p4 = p3 + v3*t3 + (1/2)*a3*t3^2 + (1/6)*j3*t3^3;
% equations
eq1 = sym(strcat(char(a4),' = 0'))
eq2 = sym(strcat(char(v4),' = 0'))
eq3 = sym(strcat(char(a2),' = amax*dir'))
[t1,t2,t3] = solve(eq1,eq2,eq3,'t1','t2','t3')
t1 = (-a0+amax*dir)/jmax/dir;
t2 = (amax+1/2*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2))/jmax;
t3 = 1/2*(4*v0*jmax*dir-2*a0^2+4*amax^2*dir^2)^(1/2)/jmax;
p4 = simple(eval(p4))