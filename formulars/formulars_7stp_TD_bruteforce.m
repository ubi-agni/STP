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
syms t1 t2 t3 t4;

% setting the jerk pulses
j1 = jmax*dir;
j2 = 0;
j3 = -jmax*dir;
j4 = jmax*dir;

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

% before phase 4
a4 = a3 + j3*t3;
v4 = v3 + a3*t3 + (1/2)*j3*t3^2;
p4 = p3 + v3*t3 + (1/2)*a3*t3^2 + (1/6)*j3*t3^3;

% finish
a5 = a4 + j4*t4;
v5 = v4 + a4*t4 + (1/2)*j4*t4^2;
p5 = p4 + v4*t4 + (1/2)*a4*t4^2 + (1/6)*j4*t4^3;
% equations
eq1 = sym(strcat(char(a2),' = amax*dir'))
eq2 = sym(strcat(char(a5),' = 0'))
eq3 = sym(strcat(char(v5),' = 0'))
[t1,t2,t3] = solve(eq1,eq2,eq3,'t1','t2','t3')
p5 = simple(eval(p5))
eq4 = sym(strcat(char(p5),' = ptarget'))

% result:
% t1 = (-a0+amax*dir)/jmax/dir;
% t2 = 1/2*(-2*v0*jmax*dir+a0^2-2*amax^2*dir^2+2*jmax^2*dir^2*t4^2)/jmax/dir^2/amax;
% t3 = (amax+jmax*t4)/jmax;
% 0 = -ptarget + p0+1/2*v0/jmax*amax-v0/jmax/dir*a0+1/2/amax*dir*jmax^2*t4^4+1/2/jmax/dir^2/amax*a0^2*v0-1/2/dir/amax*v0^2-1/8/jmax^2/dir^3/amax*a0^4+1/2*dir*amax*t4^2-1/4*a0^2/jmax^2/dir*amax+1/3*a0^3/jmax^2/dir^2+jmax*dir*t4^3;

