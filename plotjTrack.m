function [a,v,p] = plotjTrack(t0,t1,jk,next_jk,a0,v0,p0)

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

[x,y]=fplot(jerk,[t0 t1]); plot(x,y,'m','linewidth',2);
plot([t1,t1],[jk,next_jk],'m-','linewidth',2);
[x,y]=fplot(acc,[t0 t1]);  plot(x,y,'b','linewidth',2);
[x,y]=fplot(vel,[t0 t1]);  plot(x,y,'g','linewidth',2);
[x,y]=fplot(pos,[t0 t1]);  plot(x,y,'r','linewidth',2);

p = pos(t1);
v = vel(t1);
a = acc(t1);

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
%% xlabel('time');
% t = t1;
% p = eval(pos);
% v = eval(vel);
% a = eval(acc);