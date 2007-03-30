% standard 3-phases case
%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
[t,a] = calc3st(0.8,0.6,0.75,-0.4,-0.55);
plotaTracks(t,a,0.6,0.75,0.8,-0.4,-0.55, true);
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
[t, j] = calc7st(1, 1,0.7,0.8, 0,0,-1,true);
set(gca,'ytick',[0],'xtick',[0 t(1) sum(t(1:2)) sum(t(1:3)) sum(t(1:4)) sum(t(1:5)) sum(t(1:6)) sum(t(1:7))]);
text(-0.03,0.7,'v_{max}','HorizontalAlignment','right','FontSize',10')
text(-0.03,0.8,'a_{max}','HorizontalAlignment','right','FontSize',10')
text(-0.03,1.0,'j_{max}','HorizontalAlignment','right','FontSize',10')
text(-0.03,-0.7,'-v_{max}','HorizontalAlignment','right','FontSize',10')
text(-0.03,-0.8,'-a_{max}','HorizontalAlignment','right','FontSize',10')
text(-0.03,-1.0,'-j_{max}','HorizontalAlignment','right','FontSize',10')

tc = 0;
for i=1:7
  text(tc+t(i),   -1,sprintf('t_%d',i),'HorizontalAlignment','center','VerticalAlignment','top','FontSize',12');
  text(tc+t(i)/2, -1,sprintf('\\Delta t_%d',i),'HorizontalAlignment','center','VerticalAlignment','top','FontSize',12');
  tc = tc + t(i);
end
text(sum(t(1:3))/2,-0.5,'acceleration phase','HorizontalAlignment','center','FontSize',13')
text(sum(t(1:3))+t(4)/2,-0.5,'cruising phase','HorizontalAlignment','center','FontSize',13')
text(sum(t(1:4))+sum(t(5:7))/2,-0.5,'deceleration phase','HorizontalAlignment','center','FontSize',13')


% overshoot
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
[t, j] = calc7st(2, 1,1,1, 0,2,-2);
plotjTracks(t,j, 0,2,-2, true, false,true,1,true, true,false,1,true);
t = [1 1 1 0 1 0 1]; j = [-1 0 1 0 -1 0 1];
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t,j, false, -1, 1.3,1,1,1, 0,2,-2);
A = [A sym('-t1+t3=0')];
t_res = solveAll ([A V TEQ P], TVARS);
plotjTracks(t_res,j, 0,2,-2, true, false,true,true,1.3, true,true,0.7,true);

figure; 
[t, j] = calc7st(2, 1,1,1, 0,2,-2);
plotjTracks(t,j, 0,2,-2, true, false,true,1,true, true,false,1,true);
[t, j] = calc7st(1.3, 1,1,1, 0,2,-2);
plotjTracks(t,j, 0,2,-2, true, false,true,true,1.3, true,true,0.7,true);

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ab hier editiert von Risto

%%%%%%%%%
% PLOTS 1
figure;
[t, j] = calc7st (1, 2,1,1, 1.5,-0.6,0);
plotjTracks(t,j, 2,1,1, 1, 1.5,-0.6,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.335, 2,1,1, 1.5,-0.6,0);
plotjTracks(t,j, 2,1,1, 0.335, 1.5,-0.6,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (-0.160078, 2,1,1, 1.5,-0.6,0);
plotjTracks(t,j, 2,1,1, -0.160078, 1.5,-0.6,0, true, true, false, false, true, true, 1, true)

[t, j] = calc7st (1, 2,1,1, 1.5,-0.6,0);
figure; plotjTracks(t,j, 2,1,1, false, 1.5,-0.6,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.335, 2,1,1, 1.5,-0.6,0);
figure; plotjTracks(t,j, 2,1,1, false, 1.5,-0.6,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (-0.160078, 2,1,1, 1.5,-0.6,0);
figure; plotjTracks(t,j, 2,1,1, false, 1.5,-0.6,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (-0.140625, 2,1,1, 1.5,(-1.5^2/4),0);
figure; plotjTracks(t,j, 2,1,1, false, 1.5,-0.6,0, true, true, false, true, false, false, 1, true); axis off;

% 1a) normal profiles. T>T
% calc7st (2, 2,1,1, 1.5,-1.5,0, true, true);
calc7st (1, 2,1,1, 1.5,-0.6,0, true);

% 1b) normal profiles. Tw
% calc7st (1.35, 2,1,1, 1.5,(-1.5^2/4), true, true);
% calc7st (-0.55, 2,1,1, 1.5,-1.5,0, true, true);
calc7st (0.335, 2,1,1, 1.5,-0.6,0, true);

% 1c) normal profiles. T fullstop.
calc7st (-0.160078, 2,1,1, 1.5,-0.6,0, true);

% 1d) normal profiles. W fullstop.
calc7st (-0.140625, 2,1,1, 1.5,(-1.5^2/4),0,true);

%%%%%%%%%
% PLOTS 2
figure;
[t, j] = calc7st (1.46786, 2,1,1, 0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, 1.46786, 0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.46, 2,1,1, 0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, 0.46, 0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.04, 2,1,1, 0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, 0.04, 0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (-0.03007, 2,1,1, 0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, -0.03007, 0.8,-0.2,0, true, true, false, false, true, true, 1, true)

[t, j] = calc7st (1.46786, 2,1,1, 0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.46, 2,1,1, 0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.04, 2,1,1, 0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (-0.03007, 2,1,1, 0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;

% 2a) normal profiles. T>T
calc7st (1.46786, 2,1,1, 0.8,-0.2,0, true);

% 2b) normal profiles. TW
calc7st (0.46, 2,1,1, 0.8,-0.2,0, true);

% 2c) normal profiles. WW.
calc7st (0.04, 2,1,1, 0.8,-0.2,0, true);

% 2d) normal profiles. W fullstop.
calc7st (-0.03007, 2,1,1, 0.8,-0.2,0, true);

%%%%%%%%%
% PLOTS 3
figure;
[t, j] = calc7st (1, 2,1,1, 0.6,0.2,0);
plotjTracks(t,j, 2,1,1, 1, 0.6,0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.65, 2,1,1, 0.6,0.2,0);
plotjTracks(t,j, 2,1,1, 0.65, 0.6,0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.5, 2,1,1, 0.6,0.2,0);
plotjTracks(t,j, 2,1,1, 0.5, 0.6,0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.188429, 2,1,1, 0.6,0.2,0);
plotjTracks(t,j, 2,1,1, 0.188429, 0.6,0.2,0, true, true, false, false, true, true, 1, true)

[t, j] = calc7st (1, 2,1,1, 0.6,0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.6,0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.65, 2,1,1, 0.6,0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.6,0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.5, 2,1,1, 0.6,0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.6,0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.188429, 2,1,1, 0.6,0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, 0.6,0.2,0, true, true, false, true, false, false, 1, true); axis off;

% 3a) normal profiles. T<T
calc7st (1, 2,1,1, 0.6,0.2,0, true);

% 3b) normal profiles. WT
calc7st (0.65, 2,1,1, 0.6,0.2,0, true);

% 3c) normal profiles. WW.
calc7st (0.5, 2,1,1, 0.6,0.2,0, true);

% 3d) normal profiles. W fullstop.
calc7st (0.188429, 2,1,1, 0.6,0.2,0, true);

%%%%%%%%%
% PLOTS 4
figure;
[t, j] = calc7st (0.7, 2,1,1, -0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, 0.7, -0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (0.22, 2,1,1, -0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, 0.22, -0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (-0.2, 2,1,1, -0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, -0.2, -0.8,-0.2,0, true, true, false, false, true, true, 1, true)
[t, j] = calc7st (-0.275402, 2,1,1, -0.8,-0.2,0);
plotjTracks(t,j, 2,1,1, -0.275402, -0.8,-0.2,0, true, true, false, false, true, true, 1, true)

[t, j] = calc7st (0.7, 2,1,1, -0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, -0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (0.22, 2,1,1, -0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, -0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (-0.2, 2,1,1, -0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, -0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;
[t, j] = calc7st (-0.275402, 2,1,1, -0.8,-0.2,0);
figure; plotjTracks(t,j, 2,1,1, false, -0.8,-0.2,0, true, true, false, true, false, false, 1, true); axis off;

% 4a) normal profiles. T>T
calc7st (0.7, 2,1,1, -0.8,-0.2,0, true);

% 4b) normal profiles. TW
calc7st (0.22, 2,1,1, -0.8,-0.2,0, true);

% 4c) normal profiles. WW.
calc7st (-0.2, 2,1,1, -0.8,-0.2,0, true);

% 4d) normal profiles. W fullstop.
calc7st (-0.275402, 2,1,1, -0.8,-0.2,0, true);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% double deceleration figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% full stop case
[t_stop j_stop] = calc3st(0,1,1,-0.25,1.3);
[dummy, dummy, p_stop] = calcjTracks(t_stop,j_stop,-0.25,1.3,-0.85);
% zero cruise case
[t_acc a_acc] = calc3st(0.9,1,1,-0.25,1.3); % acc. part (before cruising)
[t_dec a_dec] = calc3st(0,1,1,0,0.9); % dec. part (after cruising)
t0c = [t_acc 0 t_dec]; j0c = [a_acc 0 a_dec];
[dummy, dummy, p0c] = calcjTracks(t0c,j0c, -0.25, 1.3, -0.85);
% after shift case (W-T-Grenzfall)
[a2, v2, p2] = calcjTracks(t0c(1:2),j0c(1:2), -0.25, 1.3, -0.85);
[DeltaT, T5, T7] = calc7st_opt_shift(t0c,j0c,1,1,1,a2,v2);
t_new = [t0c(1:2), t0c(3)-DeltaT, t0c(4), T5, 0, T7];
[dummy,dummy,p_new] = calcjTracks(t_new,j0c, -0.25, 1.3, -0.85);
% solution
[t,j] = calc7st(0.5, 1, 1, 0.9, -0.25, 1.3, -0.85);

% figure all with v,p (already adapted to new plotjTracks signature)
figure;
plotjTracks(t_stop,j_stop, -0.25,1.3,-0.85, true, false,false,0.9,p_stop, true, true, 1, true);
plotjTracks(t0c,j0c, -0.25,1.3,-0.85, true, false,false,0.9,p0c, true, true, 1, true);
plotjTracks(t_new,j0c, -0.25,1.3,-0.85, true, false,false,0.9,p_new, true, true, 1, true);
plotjTracks(t,j, -0.25,1.3,-0.85, true, false,false,0.9,0.5, true, true, 1, true);
text(-0.02,0.9,'v_{max}','HorizontalAlignment','right','FontSize',12')

% figure singles with a
figure;
plotjTracks(t_stop,j_stop, -0.25,1.3,-0.85, true, false,true,false,false, true, true, 1, true); axis off; figure;
plotjTracks(t0c,j0c, -0.25,1.3,-0.85, true, false,true,false,false, true, true, 1, true); axis off; figure;
plotjTracks(t_new,j0c, -0.25,1.3,-0.85, true, false,true,false,false, true, true, 1, true); axis off; figure;
plotjTracks(t,j, -0.25,1.3,-0.85, true, false,true,false,false, true, true, 1, true); axis off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figures 6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% full stop case
[t_stop j_stop] = calc3st(0,3,3,-2,2);
[dummy, dummy, p_stop] = calcjTracks(t_stop,j_stop,-2,2,0);

[t1,j1]=calc7st(2, 3, 3, 3, -2, 2, 0);
[t2,j2]=calc7st(2.5, 3, 3, 3, -2, 2, 0);
[t3,j3]=calc7st(2.8, 3, 3, 3, -2, 2, 0);
[t4,j4]=calc7st(1.5, 3, 3, 3, -2, 2, 0);
[t5,j5]=calc7st(0.88, 3, 3, 3, -2, 2, 0);
[t6,j6]=calc7st(0.75, 3, 3, 3, -2, 2, 0);
[t7,j7]=calc7st(0, 3, 3, 3, -2, 2, 0);

% figure all with v,p
figure;
plotjTracks(t_stop,j_stop, -2, 2, 0, true, false, false, 3, p_stop, true, true, 1, true);
plotjTracks(t3,j3, -2, 2, 0, true, false, false, 3, 2.8, true, true, 1, true);
plotjTracks(t4,j4, -2, 2, 0, true, false, false, 3, 1.5, true, true, 1, true);
%plotjTracks(t6,j6, -2, 2, 0, true, false, false, 3, 0.75, true, true, 1, true);
plotjTracks(t7,j7, -2, 2, 0, true, false, false, 3, 0, true, true, 1, true);

% figure singles with a
figure;
plotjTracks(t_stop,j_stop, -2, 2, 0, true, false, 3, false, false, true, true, 1, true);
figure;
plotjTracks(t3,j3, -2, 2, 0, true, false, 3, false, false, true, true, 1, true);
figure;
plotjTracks(t4,j4, -2, 2, 0, true, false, 3, false, false, true, true, 1, true);
%figure;
%plotjTracks(t6,j6, -2, 2, 0, true, false, 3, false, false, true, true, 1, true);
figure;
plotjTracks(t7,j7, -2, 2, 0, true, false, 3, false, false, true, true, 1, true);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 7 phases synchronisation case
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TcT
[t1, j1] = calc7st(1, 1,0.7,0.8, 0,0,-1);
% TW
[t2, j2] = calc7st(-0.4, 0.75,0.7,0.8, -0.35,0.2,0.5);
% WW
[t3, j3] = calc7st(0.3, 1,0.7,0.8, -0.3,0.4,-0.5);
% figure with all unsynch'ed vel
figure;
plotjTracks(t1,j1, 0, 0, -1, true, false, false, 0.8, false, true, true, 1, true);
plotjTracks(t2,j2, -0.35, 0.2, 0.5, true, false, false, 0.8, false, true, true, 1, true);
plotjTracks(t3,j3, -0.3, 0.4, -0.5, true, false, false, 0.8, false, true, true, 1, true);
ax = axis; ymin = ax(3); ymax = ax(4);
line([sum(t1),sum(t1)],[ymin,ymax],'Color','k','LineStyle','--');
line([sum(t2),sum(t2)],[ymin,ymax],'Color','k','LineStyle','--');
line([sum(t3),sum(t3)],[ymin,ymax],'Color','k','LineStyle','--');
% figure with all synch'ed vel
T = max([sum(t1) sum(t2) sum(t3)]);
[t1n, j1n] = stretch7st(t1,j1,T, 1,1,0.7,0.8, 0,0,-1);
[t2n, j2n] = stretch7st(t2,j2,T,-0.4, 0.75,0.7,0.8, -0.35,0.2,0.5);
[t3n, j3n] = stretch7st(t3,j3,T,0.3, 1,0.7,0.8, -0.3,0.4,-0.5);
figure;
plotjTracks(t1n,j1n, 0, 0, -1, true, false, false, 0.8, false, true, true, 1, true);
plotjTracks(t2n,j2n, -0.35, 0.2, 0.5, true, false, false, 0.8, false, true, true, 1, true);
plotjTracks(t3n,j3n, -0.3, 0.4, -0.5, true, false, false, 0.8, false, true, true, 1, true);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% show sync algorithm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[t1, j1] = calc7st(1, 0.5,0.4,0.7, 0,0,-1);
t2 = t1;
t2(2) = 0;
t2(6) = 0;
T = sum(t1)*4/3;
[t3, j3] = stretch7st(t1,j1,T,1, 0.5,0.4,0.7, 0,0,-1);
% figure with both v-graphs
figure;
plotjTracks(t1,j1, 0, 0, -1, true, false, false, true, false, true, true, 1, true);
plotjTracks(t2,j2, 0, 0, -1, true, false, false, true, false, true, true, 1, true);
% figure with new v-graph
figure;
plotjTracks(t3,j3, 0, 0, -1, true, false, false, true, false, true, true, 1, true);
ta = sum(t3(1:3))+t2(4);
tb = sum(t3(1:4));
ax = axis;
ymin = ax(3);
ymax = ax(4);
fill([ta,tb,tb,ta], [ymin,ymin,ymax,ymax],[0.8,0.8,0.8],'LineStyle','none');
plotjTracks(t3,j3, 0, 0, -1, true, false, false, true, false, true, true, 1, true);