function [a,v,p] = plotjTrack(t0,t1,jk,next_jk,a0,v0,p0,jerk_color,acc_color,vel_color,pos_color)

% plot all functions
jerk = inline(num2str(jk));

t_t0 = ['(t-',num2str(t0),')'];

str_acc = [num2str(a0), '+', t_t0, '*', num2str(jk)];
str_vel = [num2str(v0), '+', t_t0, '*', num2str(a0)];
str_vel = [str_vel, '+0.5*', t_t0, '^2*', num2str(jk)];
str_pos = [num2str(p0), '+', t_t0, '*', num2str(v0)];
str_pos = [str_pos, '+0.5*', t_t0, '^2*', num2str(a0)];
str_pos = [str_pos, '+(1/6)*', t_t0, '^3*', num2str(jk)];

acc = inline(str_acc);
vel = inline(str_vel);
pos = inline(str_pos);

hold on;

[x,y]=fplot(jerk,[t0 t1]); plot(x,y,'Color',jerk_color,'linewidth',2,'linestyle','--');
plot([t1,t1],[jk,next_jk],'Color',jerk_color,'linewidth',2,'linestyle','--');
[x,y]=fplot(acc,[t0 t1]);  plot(x,y,'Color',acc_color,'linewidth',2,'linestyle','-');
[x,y]=fplot(vel,[t0 t1]);  plot(x,y,'Color',vel_color,'linewidth',2,'linestyle','-');
[x,y]=fplot(pos,[t0 t1]);  plot(x,y,'Color',pos_color,'linewidth',2,'linestyle','-.');

t = t1-t0;
a = a0+t*jk;
v = v0+t*a0+0.5*t^2*jk;
p = p0+t*v0+0.5*t^2*a0+(1/6)*t^3*jk;

% jerk = inline(num2str(j));
% t = sym('t');
% acc = sym('a0+(t-t0)*j');
% vel = sym('v0+(t-t0)*a0+0.5*(t-t0)^2*j');
% pos = sym('p0+(t-t0)*v0+0.5*(t-t0)^2*a0+(1/6)*(t-t0)^3*j');
% hold on;
% fplot(jerk,[t0 t1],'c');
% fplot(char(eval(acc)),[t0 t1],'b');
% fplot(char(eval(vel)),[t0 t1],'g');
% fplot(char(eval(pos)),[t0 t1],'r');
%% legend('jerk','acc','vel','pos',-1);
%% xlabel('t');
% t = t1;
% p = eval(pos);
% v = eval(vel);
% a = eval(acc);