% tester for calc7st.m
function calc7st_tester;

amax = 1;
vmax = 1;
jmax = 1;

p0 = 0;
a0 = [-1.2,-1.0,-0.7,0.0,0.7,1.0,1.2];
v0 = [-1.2,-1.0,-0.7,0.0,0.7,1.0,1.2];

errors = [];
for i_a = 1:length(a0),
	for i_v = 1:length(v0),
		p_fullstop = calcFullstopPosition(jmax,amax,a0(i_a),v0(i_v),p0);
		calc7st(p_fullstop, jmax, amax, vmax, a0(i_a), v0(i_v), p0, false,false,true);
		p_neg = calcZeroCruisePosition(-1, jmax, amax, vmax, a0(i_a), v0(i_v), p0);
		p_pos = calcZeroCruisePosition(1, jmax, amax, vmax, a0(i_a), v0(i_v), p0);
		dp = abs((p_neg-p_pos)/16);
		for p_target = min(p_neg,p_pos)-6*dp:dp:max(p_pos,p_neg)+6*dp
            [t,j] = calc7st(p_target, jmax, amax, vmax, a0(i_a), v0(i_v), p0,false,false,false);
            [isCorrect, reason] = testjTracks(t,j,a0(i_a),v0(i_v),p0,jmax,amax,vmax,p_target);
            try
            if (~isCorrect)
                err = sprintf('error in calc7st(%.20g, %g, %g, %g, %g, %g, %g):\n%s',p_target,jmax,amax,vmax,a0(i_a),v0(i_v),p0, reason);
                disp(err); 
                errors = [errors err];
            end
            catch
                disp(lasterr);
            end
		end
	end
end
errors

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pos = calcFullstopPosition(jmax, amax, a0, v0, p0)
	[t_stop a_stop] = calc3st(0,jmax,amax,a0,v0);
	[ah vh p_stop] = calcjTracks(t_stop,a_stop,a0,v0,p0);
	pos = p_stop;
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function pos = calcZeroCruisePosition(dir, jmax, amax, vmax, a0, v0, p0)
	% position change just from acc and dec phase:
	[t_acc a_acc] = calc3st(dir*vmax,jmax,amax,a0,v0); % acc. part (before cruising)
	[t_dec a_dec] = calc3st(0,jmax,amax,0,dir*vmax); % dec. part (after cruising)
	% postion change:
	t_zeroCruise = [t_acc 0 t_dec];
	j_zeroCruise= [a_acc 0 a_dec];
	[ah vh p_stop] = calcjTracks(t_zeroCruise,j_zeroCruise, a0, v0, p0);
	pos = p_stop;
return