% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases
% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases

function [a,v,p] = plotjTracks(t,j, jmax,amax,vmax,ptarget, a0,v0,p0, bNice,bColor)

if (nargin < 10) bNice=false; end
if (nargin < 11) bColor = false; end

[t,j]= shrink_t_j(t,j);
tend = sum(t);
[ymin ymax] = getRanges(t,j, jmax,amax,vmax, a0,v0,p0);

if (~bColor)
    % color values (black and white set)
    box_gray = [0.8,0.8,0.8];
    jerk_color = [0,0,0];
    acc_color = [0,0,0];
    vel_color = [0,0,0];
    pos_color = [0,0,0];
else
    % color values (colored set)
    box_gray = [0.8,0.8,0.8];
    jerk_color = 'g';
    acc_color = 'r';
    vel_color = 'b';
    pos_color = 'k';
end

hold off; % begin new figure
% plot x-axis at y=0
plot([0,tend],[0,0],'Color','k');

hold on; % continue in current figure
if (bNice) 
    % plot gray boxes in the background to make the different phases easier recognisable
    tc=0; bPaint = false;
    for i=1:length(t)
        if (bPaint)
            fill([tc,tc+t(i),tc+t(i),tc], [ymin,ymin,ymax,ymax],box_gray,'LineStyle','none')
        end
        bPaint = ~bPaint;
        tc = tc + t(i);
    end

    % plot boundries for jerk, acc and vel
    line([0,tend],[0,0],'Color','k');
    if ( jmax <= ymax) line([0,tend],[jmax,jmax],'Color', jerk_color, 'LineStyle', ':'); end
    if (-jmax >= ymin) line([0,tend],[-jmax,-jmax],'Color', jerk_color, 'LineStyle', ':'); end
    if ( amax <= ymax) line([0,tend],[amax,amax],'Color', acc_color, 'LineStyle', ':'); end
    if (-amax >= ymin) line([0,tend],[-amax,-amax],'Color', acc_color, 'LineStyle', ':'); end
    if ( vmax <= ymax) line([0,tend],[vmax,vmax],'Color', vel_color, 'LineStyle', ':'); end
    if (-vmax >= ymin) line([0,tend],[-vmax,-vmax],'Color', vel_color, 'LineStyle', ':'); end
    %line([0,tend],[p0,p0],'Color', pos_color, 'LineStyle', ':');
    %line([0,tend],[ptarget,ptarget],'Color', pos_color, 'LineStyle', ':');
end

% plot the curves
a=a0;
v=v0;
p=p0;
tc = 0; j = [j j(length(j))];
for i=1:length(t),
    [a,v,p] = plotjTrack(tc, tc+t(i),j(i),j(i+1), a,v,p, jerk_color,acc_color,vel_color,pos_color);
    tc = tc + t(i);
end

axis tight; hold off;
set(gcf,'PaperPosition',[0,0,8,3],'Color','w');
return



function [ymin, ymax, ranges] = getRanges(t,j, jmax,amax,vmax, a0,v0,p0)
% calculate ranges for p,v,a,j
ranges = struct('p', [p0,p0], 'v', [v0,v0], 'a', [a0,a0], 'j', [0,0]);
ymin = min ([ranges.p(1), ranges.v(1), ranges.a(1), ranges.j(1), j]);
ymax = max ([ranges.p(2), ranges.v(2), ranges.a(2), ranges.j(2), j]);

v=v0; p=p0; a=a0;
for i=1:length(t)
    if (t(i) > 0)
        [a2,v2,p2] = calcjTrack(t(i),j(i), a,v,p);
        % in the case of overshooting velocity or position, get the
        % maximum between v (p) and v2 (p2):
        syms tt;
        f_a = inline(eval('a+tt*j(i)'));
        f_v = inline(eval('v+tt*a+0.5*tt^2*j(i)'));
        f_p = inline(eval('p+tt*v+0.5*tt^2*a+(1/6)*tt^3*j(i)'));               
        if ((sign(a) == -sign(a2)) && ~isZero(a) && ~isZero(a2))
            t0 = fzero(f_a,[0,t(i)]);
            if (a > 0)
                ranges.v(2) = max (ranges.v(2), f_v(t0));
            else
                ranges.v(1) = min (ranges.v(1), f_v(t0));
            end
        end
        if ((sign(v) == -sign(v2)) && ~isZero(v) && ~isZero(v2))
            t0 = fzero(f_v,[0,t(i)]);
            if (v > 0)
                ranges.p(2) = max (ranges.p(2), f_p(t0));
            else
                ranges.p(1) = min (ranges.p(1), f_p(t0));
            end
        end
        a = a2; v = v2; p = p2;
        ranges.p(1) = min (ranges.p(1), p); ranges.p(2) = max (ranges.p(2), p);
        ranges.v(1) = min (ranges.v(1), v); ranges.v(2) = max (ranges.v(2), v);
        ranges.a(1) = min (ranges.a(1), a); ranges.a(2) = max (ranges.a(2), a);
        ymin = min ([ymin, ranges.p(1), ranges.v(1), ranges.a(1)]);
        ymax = max ([ymax, ranges.p(2), ranges.v(2), ranges.a(2)]);
    end
end
return



function [a,v,p] = plotjTrack(t0,t1, j,j_next, a0,v0,p0, jerk_color,acc_color,vel_color,pos_color)
t = sym('t');
acc = sym('a0+(t-t0)*j');
vel = sym('v0+(t-t0)*a0+0.5*(t-t0)^2*j');
pos = sym('p0+(t-t0)*v0+0.5*(t-t0)^2*a0+(1/6)*(t-t0)^3*j');

line([t0 t1], [j,j], 'Color',jerk_color,'linewidth',2,'linestyle','--');
line([t1,t1], [j,j_next], 'Color',jerk_color,'linewidth',2,'linestyle','--');
[x,y]=fplot(char(eval(acc)),[t0 t1]);  plot(x,y,'Color',acc_color,'linewidth',2,'linestyle','-');
[x,y]=fplot(char(eval(vel)),[t0 t1]);  plot(x,y,'Color',vel_color,'linewidth',2,'linestyle','-');
[x,y]=fplot(char(eval(pos)),[t0 t1]);  plot(x,y,'Color',pos_color,'linewidth',2,'linestyle','-.');

% compute end points
t = t1;
a = eval(acc);
v = eval(vel);
p = eval(pos);
return