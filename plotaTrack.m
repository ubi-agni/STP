function [v,p] = plotaTrack(t0,t1,a,v0,p0)

% plot all functions
acc = inline(num2str(a));
t = sym('t');
vel = sym('v0+(t-t0)*a');
pos = sym('p0+(t-t0)*v0+0.5*(t-t0)^2*a');
hold on;
fplot(acc,[t0 t1],'b');
fplot(char(eval(vel)),[t0 t1],'g');
fplot(char(eval(pos)),[t0 t1],'r');

t = t1;
p = eval(pos);
v = eval(vel);