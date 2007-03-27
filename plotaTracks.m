% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases
% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases

function [a,v,p] = plotaTracks(t,a, amax,vmax,ptarget, v0,p0, bNice,bColor)

if (nargin < 8) bNice=true; end
if (nargin < 9) bColor = false; end

[t,a]= shrink_t_j(t,a);
tend = sum(t);
[ymin ymax] = getRanges(t,a, amax,vmax, v0,p0);

if (~bColor)
    % color values (black and white set)
    box_gray = [0.8,0.8,0.8];
    acc_color = [0,0,0];
    vel_color = [0,0,0];
    pos_color = [0,0,0];
else
    % color values (colored set)
    box_gray = [0.8,0.8,0.8];
    acc_color = 'r';
    vel_color = 'b';
    pos_color = 'k';
end

hold off; % begin new figure
% plot x-axis at y=0
line([0,tend],[0,0],'Color','k');

hold on; % continue in current figure
if (bNice) 
    % plot gray boxes in the background to make the different phases easier recognisable
    tc=0; bPaint = true;
    for i=1:length(t)
        if (bPaint)
            fill([tc,tc+t(i),tc+t(i),tc], [ymin,ymin,ymax,ymax],box_gray,'LineStyle','none')
        end
        bPaint = ~bPaint;
        tc = tc + t(i);
    end

    % plot boundries for jerk, acc and vel
    line([0,tend],[0,0],'Color','k');
    if ( amax <= ymax) line([0,tend],[amax,amax],'Color', acc_color, 'LineStyle', ':'); end
    if (-amax >= ymin) line([0,tend],[-amax,-amax],'Color', acc_color, 'LineStyle', ':'); end
    if ( vmax <= ymax) line([0,tend],[vmax,vmax],'Color', vel_color, 'LineStyle', ':'); end
    if (-vmax >= ymin) line([0,tend],[-vmax,-vmax],'Color', vel_color, 'LineStyle', ':'); end
    %line([0,tend],[p0,p0],'Color', pos_color, 'LineStyle', ':');
    %line([0,tend],[ptarget,ptarget],'Color', pos_color, 'LineStyle', ':');
end

% plot the curves
v = v0;
p = p0;
tc = 0; a = [a a(length(a))];
for i=1:length(t),
    [v,p] = plotaTrack(tc, tc+t(i),a(i),a(i+1), v,p, acc_color,vel_color,pos_color);
    tc = tc + t(i);
end

axis tight; hold off;
set(gcf,'PaperPosition',[0,0,8,3],'Color','w');
return

function [ymin, ymax, ranges] = getRanges(t,a,amax,vmax,v0,p0)
% calculate ranges for p,v,a
ranges = struct('p', [p0,p0], 'v', [v0,v0], 'a', [0,0]);
ymin = min ([ranges.p(1), ranges.v(1), ranges.a(1), a]);
ymax = max ([ranges.p(2), ranges.v(2), ranges.a(2), a]);

v=v0; p=p0;
for i=1:length(t)
    if (t(i) > 0)
        [v,p] = calcaTrack(t(i),a(i),v,p);
    end
    ranges.p(1) = min (ranges.p(1), p); ranges.p(2) = max (ranges.p(2), p);
    ranges.v(1) = min (ranges.v(1), v); ranges.v(2) = max (ranges.v(2), v);
    ymin = min ([ymin, ranges.p(1), ranges.v(1)]);
    ymax = max ([ymax, ranges.p(2), ranges.v(2)]);
end
return

function [v,p] = calcaTrack(t,a, v0,p0)
v = v0+t*a;
p = p0+t*v0+0.5*t^2*a;
return

function [v,p] = plotaTrack(t0,t1, a,a_next, v0,p0, acc_color,vel_color,pos_color)
t = sym('t');
acc = sym('a');
vel = sym('v0 + a *(t-t0)');
pos = sym('p0 + v0*(t-t0) + 0.5*a*(t-t0)^2');

line([t0,t1],[a,a],'Color',acc_color, 'linewidth',2, 'LineStyle','-');
line([t1,t1],[a,a_next],'Color',acc_color, 'linewidth',2, 'LineStyle', '-');
[x y] = fplot(char(eval(vel)),[t0 t1]); plot(x,y,'Color',vel_color,'linewidth',2,'linestyle','-');
[x y] = fplot(char(eval(pos)),[t0 t1]); plot(x,y,'Color',pos_color,'linewidth',2,'linestyle','-.');

% compute end points
t=t1;
v = eval(vel);
p = eval(pos);
return