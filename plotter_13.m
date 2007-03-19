% cruising, no overshooting
fig = figure;
set(fig,'name','with cruising, ptarget > p0, w > 0');

p0 = 0;
ptarget = 15;

% a0 = 0, v0 < 0
jmax = 3;
amax = 2.5;
vmax = 3;
a0 = 0;
v0 = -4;
subplot(2,3,1);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 14, -10, 15]);
grid on;

% a0 = 0, v0 > 0
jmax = 3;
amax = 1.5;
vmax = 3;
a0 = 0;
v0 = 4;
subplot(2,3,4);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 12, -5, 15]);
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a0 = 0, v0 < 0
jmax = 1.25;
amax = 2.5;
vmax = 3;
a0 = 0;
v0 = -4;
subplot(2,3,2);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 14, -10, 15]);
grid on;

% a0 = 0, v0 > 0
jmax = 1.5;
amax = 1.5;
vmax = 3;
a0 = 0;
v0 = 4;
subplot(2,3,5);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 12, -5, 15]);
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a0 = 0, v0 < 0
jmax = 0.75;
amax = 2.5;
vmax = 3;
a0 = 0;
v0 = -4;
subplot(2,3,3);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 14, -10, 15]);
grid on;

% a0 = 0, v0 > 0
jmax = 0.75;
amax = 1.5;
vmax = 3;
a0 = 0;
v0 = 4;
subplot(2,3,6);
title(sprintf('v0=%.2f, jmax=%.2f',v0,jmax));
[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0)
[a,v,p] = plotjTracks([0 t],j,a0,v0,p0)
axis([0, 12, -5, 15]);
grid on;