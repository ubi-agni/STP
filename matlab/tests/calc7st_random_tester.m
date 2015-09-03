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
	
	%disp(sprintf('calc7st(%f, %f, %f, %f, %f, %f, %f)...',ptarget,jmax,amax,vmax,a0,v0,p0));
    calc7st(ptarget,jmax,amax,vmax,a0,v0,p0,false,false,true);
    
	c = c+1;
	if (mod(c,10)==0)
		disp(sprintf('Tested %d cases',c));
	end
end