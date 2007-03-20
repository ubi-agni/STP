function [v,p] = plotaTrack(t0,t1,a,next_a,v0,p0, acc_color, vel_color, pos_color)

% plot all functions

t_t0 = ['(t-',num2str(t0),')'];

acc = inline(num2str(a));

str_vel = [num2str(v0), '+', t_t0, '*', num2str(a)];
str_pos = [num2str(p0), '+', t_t0, '*', num2str(v0)];
str_pos = [str_pos, '+0.5*', t_t0, '^2*', num2str(a)];

vel = inline(str_vel);
pos = inline(str_pos);

hold on;

[x,y]=fplot(acc,[t0 t1]);  plot(x,y,'Color',acc_color,'linewidth',2,'linestyle','-');
plot([t1,t1],[a,next_a],'Color',acc_color,'linewidth',2,'linestyle','-');
[x,y]=fplot(vel,[t0 t1]);  plot(x,y,'Color',vel_color,'linewidth',2);
[x,y]=fplot(pos,[t0 t1]);  plot(x,y,'Color',pos_color,'linewidth',2);

p = pos(t1);
v = vel(t1);