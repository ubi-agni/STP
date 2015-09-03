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
j2 = -jmax*dir;
j3 = 0;
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
eq1 = sym(strcat(char(a3),' = -amax*dir'))
eq2 = sym(strcat(char(a5),' = 0'))
eq3 = sym(strcat(char(v5),' = 0'))
[t2,t3,t4] = solve(eq1,eq2,eq3,'t2','t3','t4')
p5 = simple(eval(p5))
eq4 = sym(strcat(char(p5),' = ptarget'))

% result:
% t2 = (a0+jmax*dir*t1+amax*dir)/jmax/dir;
% t3 = 1/2*(2*v0*jmax*dir+a0^2-2*amax^2*dir^2+2*jmax^2*dir^2*t1^2+4*a0*t1*jmax*dir)/jmax/dir^2/amax;
% t4 = amax/jmax;
% 0 = -ptarget + jmax*dir*t1^3+2*v0*t1+p0+3*a0*t1^2+1/jmax/dir*v0*a0+1/4/jmax^2/dir*a0^2*amax+1/3/jmax^2/dir^2*a0^3+1/2/jmax*v0*amax+2/jmax/dir*a0^2*t1+1/2*dir*t1^2*amax+1/jmax*a0*t1*amax+1/2/dir/amax*v0^2+1/2*jmax^2*dir/amax*t1^4+1/2/jmax/dir^2/amax*a0^2*v0+1/jmax/dir^2/amax*t1*a0^3+1/8/jmax^2/dir^3/amax*a0^4+jmax/amax*v0*t1^2+2*jmax/amax*a0*t1^3+5/2/dir/amax*a0^2*t1^2+2/dir/amax*v0*a0*t1
