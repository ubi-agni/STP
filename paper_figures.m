% standard 3-phases case
%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
[t,a] = calc3st(0.8,0.6,0.75,0.4,-0.8);
plotaTracksNice(t,a,0.6,0.75,0.8,0.4,-0.8);

% standard 7-phases case
%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
calc7st(1,1,0.7,0.8,0,0,-1,true);
figure;
calc7st(1.3,  1, 0.7, 0.8,  0.8, 0.7, -1.2, true);

% multidimensional, sync. and async. 3-phases case
% gray boxed and all graph plotting beside the vel-
% graph need to be switched off in plotaTracksNice.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
subplot(2,1,1);
[t1,a] = calc3st(0.8,0.6,0.75,-0.2,-0.8);
plotaTracksNice(t1,a,0.6,0.75,0.8,-0.2,-0.8);
[t2,a] = calc3st(0.4,0.6,0.75,0.5,-0.8);
plotaTracksNice(t2,a,0.6,0.75,0.4,0.5,-0.8);
[t3,a] = calc3st(-0.4,0.4,0.75,0,0.5);
plotaTracksNice(t3,a,0.4,0.75,-0.4,0,0.5);
tnew = max([t1 t2 t3]);
subplot(2,1,2);
[t,a] = calc3st_sync(tnew,0.8,0.6,0.75,-0.2,-0.8);
[t,a] = calc3st_sync(tnew,0.4,0.6,0.75,0.5,-0.8);
[t,a] = calc3st_sync(tnew,-0.4,0.4,0.75,0,0.5);
set(gcf,'PaperPosition',[0,0,8,6]);