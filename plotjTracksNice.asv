% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases

% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases

function [a,v,p] = plot7Track(t,j,jmax,amax,vmax,ptarget,a0,v0,p0)

[t,j]= shrink_t_j(t,j);
t = [0 t];

% color values (black and white set)
%%%%%%%
box1_gray = [1.0,1.0,1.0];
box2_gray = [0.,0.,0.];
jerk_color = [0,0,0];
acc_color = [0,0,0];
vel_color = [0,0,0];
pos_color = [0,0,0];

% color values (colored set)
%%%%%%%
% box1_gray = [0.8,0.8,0.8];
% box2_gray = [0.65,0.65,0.65];
% jerk_color = 'c';
% acc_color = 'b';
% vel_color = 'g';
% pos_color = 'r';

clf;
hold on;

tend = t(length(t));

% plot boundries for jerk, acc and vel
line([0,tend],[0,0],'Color','k');
line([0,tend],[jmax,jmax],'Color', jerk_color, 'LineStyle', ':');
line([0,tend],[-jmax,-jmax],'Color', jerk_color, 'LineStyle', ':');
line([0,tend],[amax,amax],'Color', acc_color, 'LineStyle', ':');
line([0,tend],[-amax,-amax],'Color', acc_color, 'LineStyle', ':');
line([0,tend],[vmax,vmax],'Color', vel_color, 'LineStyle', ':');
line([0,tend],[-vmax,-vmax],'Color', vel_color, 'LineStyle', ':');
%line([0,tend],[p0,p0],'Color', pos_color, 'LineStyle', ':');
%line([0,tend],[ptarget,ptarget],'Color', pos_color, 'LineStyle', ':');

% plot the curves
a=a0;
v=v0;
p=p0;
for i=1:length(t)-1,
    if (t(i+1) > t(i))
        if (i<length(t)-1)
            [a,v,p] = plotjTrack(t(i),t(i+1),j(i),j(i+1),a,v,p,jerk_color,acc_color,vel_color,pos_color);
        else
            [a,v,p] = plotjTrack(t(i),t(i+1),j(i),0,a,v,p,jerk_color,acc_color,vel_color,pos_color);
        end
    end
end

axis tight;

% plot gray boxes in the background to make the different jerk phases
% easier recognisable
ax = axis;
ymin = ax(3);
ymax = ax(4);
for i=1:length(t)-1,
	if (mod(i,2) == 1)
%		fill([t(i),t(i+1),t(i+1),t(i)], [ymin,ymin,ymax,ymax],box1_gray,'LineStyle','none')
	else 
		fill([t(i),t(i+1),t(i+1),t(i)], [ymin,ymin,ymax,ymax],box2_gray,'LineStyle','none','FaceAlpha',0.2)
	end
end

hold off;
set(gcf,'PaperPosition',[0,0,8,4],'Color','w');
