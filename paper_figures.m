% standard 3-phases case
%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
[t,a] = calc3st(0.8,0.6,0.75,-0.4,-0.55);
plotaTracksNice(t,a,0.6,0.75,0.8,-0.4,-0.55);
set(gca,'ytick',[0],'xtick',[0 t(1),t(2),t(3)]);
text(-0.02,0.75,'v_{max}','HorizontalAlignment','right','FontSize',12')
text(-0.02,0.6,'a_{max}','HorizontalAlignment','right','FontSize',12')
text(t(1),-0.69,'t_1','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text(t(2),-0.69,'t_2','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text(t(3),-0.69,'t_3','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text(t(1)/2,-0.7,'\Delta t_1','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text((t(1)+t(2))/2,-0.7,'\Delta t_2','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text((t(2)+t(3))/2,-0.7,'\Delta t_3','HorizontalAlignment','center','VerticalAlignment','top','FontSize',12')
text(t(1)/2,-0.4,'acceleration phase','HorizontalAlignment','center','FontSize',12')
text((t(1)+t(2))/2,-0.4,'cruising phase','HorizontalAlignment','center','FontSize',12')
text((t(2)+t(3))/2,-0.4,'deceleration phase','HorizontalAlignment','center','FontSize',12')

% standard 7-phases case
%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
calc7st(1,1,0.7,0.8,0,0,-1,true);
figure;
calc7st(1.3,  1, 0.7, 0.8,  0.8, 0.7, -1.2, true);

% shortening T-T
figure;
calc7st(-0.513672, 2,1,1.5, 1.5,0.5,-2,true,false);
calc7st(0.15, 2,1,1.5, 1.5,0.5,-2,true,false);
calc7st(0.826172, 2,1,1.5, 1.5,0.5,-2, true, false);
ax=axis; tend=ax(2);
line([0,tend],[1.5,1.5],'Color','k','LineStyle',':');
line([0,tend],[1,1],'Color','k','LineStyle',':');
line([0,tend],[-1,-1],'Color','k','LineStyle',':');
set(gca,'ytick',[-1 0 1 1.5],'xtick',[]);

% shortening T-T to T-D
figure; 
calc7st(0.672039, 2,1.2,1.5, 1.7,-0.5,-2,true,false);
calc7st(-0.8, 2,1.2,1.5, 1.7,-0.5,-2,true,false);
calc7st(-1.225, 2,1.2,1.5, 1.7,-0.5,-2,true,false);
calc7st(-1.7, 2,1.2,1.5, 1.7,-0.5,-2,true,false);
calc7st(-1.941370, 2,1.2,1.5, 1.7,-0.5,-2,true,false);
ax=axis; tend=ax(2);
line([0,tend],[1.5,1.5],'Color','k','LineStyle',':');
line([0,tend],[1.2,1.2],'Color','k','LineStyle',':');
line([0,tend],[-1.2,-1.2],'Color','k','LineStyle',':');
set(gca,'ytick',[-1.2 0 1.2 1.5],'xtick',[]);

% double deceleration, shortening possebilities
figure;
t = [1.3 2.6 3.156776 3.71355287]; j = [-1 1 -1 1];
[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
t = [1 2 3 4]; j = [-1 1 -1 1];
[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
line([0,4],[1,1],'Color','k','LineStyle',':');
%figure;
%t = [1 2 3 4]; j = [-1 1 -1 1];
%[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
%t = [0.5567764 1.1135528 2.4135528 3.7135528]; j = [-1 1 -1 1];
%[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
%line([0,4],[1,1],'Color','k','LineStyle',':');
figure;
t = [1 1.5 2.118033988 3.236067977]; j = [-1 1 -1 1];
[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
t = [1 2 3 4]; j = [-1 1 -1 1];
[a v p] = plotjTracksNice(t,j, 1,1,1, 0,0,2,-2, false);
line([0,4],[1,1],'Color','k','LineStyle',':');

% double deceleration, D-D -> D-T
figure; calc7st(1.5, 2,1.5,1, 0,2,-1.5, true);
figure; calc7st(1.3, 2,1.5,1, 0,2,-1.5, true); % buggy

% double deceleration, D-T
figure; calc7st(1.7, 2,1.5,1.5, 0,2,-1.5, true);
figure; calc7st(1.5625, 2,1.5,1.5, 0,2,-1.5, true);
figure; calc7st(1., 2,1.5,1.5, 0,2,-1.5, true);
figure; calc7st(0.7, 2,1.5,1.5, 0,2,-1.5, true);
figure; calc7st(0.583334, 2,1.5,1.5, 0,2,-1.5, true);
figure; calc7st(0.4, 2,1.5,1.5, 0,2,-1.5, true);

% multidimensional, sync. and async. 3-phases case
% gray boxed and all graph plotting beside the vel-
% graph need to be switched off in plotaTracksNice.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
[t1,a] = calc3st(0.8,0.6,0.75,-0.2,-0.8);
plotaTracksNice(t1,a,0.6,0.75,0.8,-0.2,-0.8,false);
[t2,a] = calc3st(0.4,0.6,0.75,0.5,-0.8);
plotaTracksNice(t2,a,0.6,0.75,0.4,0.5,-0.8,false);
[t3,a] = calc3st(-0.4,0.4,0.75,0,0.5);
plotaTracksNice(t3,a,0.4,0.75,-0.4,0,0.5,false);
set(gcf,'PaperPosition',[0,0,8,2]);
ax = axis; ymin = ax(3); ymax = ax(4);
line([max(t1),max(t1)],[ymin,ymax],'Color','k','LineStyle','--');
line([max(t2),max(t2)],[ymin,ymax],'Color','k','LineStyle','--');
line([max(t3),max(t3)],[ymin,ymax],'Color','k','LineStyle','--');

tnew = max([t1 t2 t3]);
figure;
[t,a] = calc3st_sync(tnew,0.8,0.6,0.75,-0.2,-0.8);
[t,a] = calc3st_sync(tnew,0.4,0.6,0.75,0.5,-0.8);
[t,a] = calc3st_sync(tnew,-0.4,0.4,0.75,0,0.5);
set(gcf,'PaperPosition',[0,0,8,2]);