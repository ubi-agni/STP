% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases

% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases

function [a,v,p] = plot7Track(t,a,amax,vmax,ptarget,v0,p0)

[t,a]= shrink_t_j(t,a);
t = [0 t];

% color values (black and white set)
%%%%%%%
box_gray = [0.8,0.8,0.8];
acc_color = [0,0,0];
vel_color = [0,0,0];
pos_color = [0,0,0];

% color values (colored set)
%%%%%%%
% box_gray = [0.8,0.8,0.8];
% acc_color = 'b';
% vel_color = 'g';
% pos_color = 'r';

<<<<<<< .mine
%clf;
hold on;
=======
clf; hold off;
% plot the curves to find axes ranges
v = v0;
p = p0;
for i=1:length(t)-1,
    if (t(i+1) > t(i))
        if (i<length(t)-1)
			[v,p] = plotaTrack(t(i),t(i+1),a(i),a(i+1),v,p,acc_color, vel_color, pos_color);
		else
			[v,p] = plotaTrack(t(i),t(i+1),a(i),0,v,p,acc_color, vel_color, pos_color);
		end
    end
end
axis tight;
ax = axis; ymin = ax(3); ymax = ax(4);
>>>>>>> .r803

clf; hold on;

% plot gray boxes in the background to make the different jerk phases
% easier recognisable
for i=1:length(t)-1,
	if (mod(i,2) == 1)
		fill([t(i),t(i+1),t(i+1),t(i)], [ymin,ymin,ymax,ymax],box_gray,'LineStyle','none')
	else 
%		fill([t(i),t(i+1),t(i+1),t(i)], [ymin,ymin,ymax,ymax],box_gray,'LineStyle','none')
	end
end

% plot boundries for jerk, acc and vel
tend = t(length(t));
line([0,tend],[0,0],'Color','k');
if ( amax <= ymax) line([0,tend],[amax,amax],'Color', acc_color, 'LineStyle', ':'); end
if (-amax >= ymin) line([0,tend],[-amax,-amax],'Color', acc_color, 'LineStyle', ':'); end
if ( vmax <= ymax) line([0,tend],[vmax,vmax],'Color', vel_color, 'LineStyle', ':'); end
if (-vmax >= ymin) line([0,tend],[-vmax,-vmax],'Color', vel_color, 'LineStyle', ':'); end
%line([0,tend],[p0,p0],'Color', pos_color, 'LineStyle', ':');
%line([0,tend],[ptarget,ptarget],'Color', pos_color, 'LineStyle', ':');

% plot the curves
v = v0;
p = p0;
for i=1:length(t)-1,
    if (t(i+1) > t(i))
        if (i<length(t)-1)
			[v,p] = plotaTrack(t(i),t(i+1),a(i),a(i+1),v,p,acc_color, vel_color, pos_color);
		else
			[v,p] = plotaTrack(t(i),t(i+1),a(i),0,v,p,acc_color, vel_color, pos_color);
		end
    end
end

axis tight; hold off;
set(gcf,'PaperPosition',[0,0,8,3],'Color','w');
