% cruising, no overshooting
fig = figure;
set(fig,'name','with cruising, ptarget > p0, w > 0');

p0 = 0;
ptarget = -3;

% a0 = 0, v0 < 0
ptarget = -6;
jmax = 4;
amax = 2;
vmax = 3;
a0 = 1;
v0 = -2;
subplot(3,3,1);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 6, -6, 6]);grid on;
ptarget = -3;
% a0 = 0, v0 = 0
jmax = 3;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 0;
subplot(3,3,4);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 6]);grid on;
% a0 = 0, v0 > 0
jmax = 3;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 2;
subplot(3,3,7);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 6]);grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a0 = 0, v0 < 0
ptarget = -6;
jmax = 2;
amax = 2;
vmax = 3;
a0 = 1;
v0 = -2;
subplot(3,3,2);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 6, -6, 6]);grid on;
ptarget = -3;
% a0 = 0, v0 = 0
jmax = 1.25;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 0;
subplot(3,3,5);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 6]);grid on;
% a0 = 0, v0 > 0
jmax = 1.25;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 2;
subplot(3,3,8);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 6]);grid on;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a0 = 0, v0 < 0
ptarget = -6;
jmax = 0.8;
amax = 2;
vmax = 3;
a0 = 1;
v0 = -2;
subplot(3,3,3);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 6, -6, 6]);grid on;
ptarget = -3;
% a0 = 0, v0 = 0
jmax = 0.8;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 0;
subplot(3,3,6);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 6]);grid on;
% a0 = 0, v0 > 0
jmax = 0.8;
amax = 2;
vmax = 3;
a0 = 1;
v0 = 2;
subplot(3,3,9);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracksNice(t,j,ptarget,jmax,amax,vmax,a0,v0,p0,false)
axis([0, 12, -6, 9]);grid on;