% tester for calc7st.m
% will run with random initial values till calc7st.m is returning any incorrect values.

rand('state',sum(100*clock));
c = 0
while (1 > 0)
	vmax = rand*20+0.1;
	amax = rand*10+0.1*vmax;
	jmax = rand*5+0.1*amax;
	a0 = 2*(2*rand*amax-amax);
	v0 = 2*(2*rand*vmax-vmax);
	ptarget = 20*vmax*rand-10*vmax;
	p0 = 20*vmax*rand-10*vmax;
	
	disp(sprintf('calc7st(%f, %f, %f, %f, %f, %f, %f)...',ptarget,jmax,amax,vmax,a0,v0,p0));
	[t,j] = calc7st(ptarget,jmax,amax,vmax,a0,v0,p0,false);
	[t,j] = shrink_t_j(t,j);
	
	[isCorrect, reason] = testjTracks([0 t],j,a0,v0,p0,jmax,amax,vmax,ptarget);
	if (isCorrect == false)
		disp(sprintf('Problem when calling calc7st(%f, %f, %f, %f, %f, %f, %f)...',ptarget,jmax,amax,vmax,a0,v0,p0));
		disp(sprintf('!!!!!!!!!!!'));
		disp(sprintf(reason));
		return;
	end
	c = c+1;
	if (mod(c,10)==0)
		disp(sprintf('Tested %d cases',c));
	end
end