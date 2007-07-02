% tester for calc7st.m
function calc7st_tester(bTestStretched);

if (nargin < 1) bTestStretched = false; end

amax = 1;
vmax = 1;
jmax = 1;

p0 = 0;
a0 = [-1.2,-1.0,-0.7,0.0,0.7,1.0,1.2];
v0 = [-1.2,-1.0,-0.7,0.0,0.7,1.0,1.2];
dt = [1.0001, 1.125, 1.25, 1.375, 1.5, 1.6667, 1.83333, 2.0, 3.0, 10.0];

errors = [];
for i_a = 1:length(a0),
	for i_v = 1:length(v0),
		p_fullstop = calcFullstopPosition(jmax,amax,a0(i_a),v0(i_v),p0);
		calc7st(p_fullstop, jmax, amax, vmax, a0(i_a), v0(i_v), p0, false,false,true);
		p_neg = calcZeroCruisePosition(-1, jmax, amax, vmax, a0(i_a), v0(i_v), p0);
		p_pos = calcZeroCruisePosition(1, jmax, amax, vmax, a0(i_a), v0(i_v), p0);
		dp = abs((p_neg-p_pos)/9);
		for p_target = min(p_neg,p_pos)-5*dp:dp:max(p_pos,p_neg)+5*dp
            [t,j] = calc7st(p_target, jmax, amax, vmax, a0(i_a), v0(i_v), p0,false,false,false);
            if bTestStretched
                for i_t = 1:length(dt),
                    Tnew = dt(i_t) * sum(t);
                    disp(sprintf('testing stretch7st(t, j, %.20g, %.20g, %g, %g, %g, %g, %g, %g, true)\n',Tnew, p_target,jmax,amax,vmax,a0(i_a),v0(i_v),p0));
                    [ts, js] = stretch7st(t,j,Tnew,p_target, jmax, amax, vmax, a0(i_a), v0(i_v), p0,false);
                    [isCorrect, reason] = testjTracks(ts,js,a0(i_a),v0(i_v),p0,jmax,amax,vmax,p_target);
                    try
                        if (~isCorrect)
                            err = sprintf('error in calc7st(%.20g, %g, %g, %g, %g, %g, %g):\n%s',p_target,jmax,amax,vmax,a0(i_a),v0(i_v),p0, reason);
                            disp(err); 
                            errors = [errors err '\n'];
                        end
                    catch
                        disp(lasterr);
                    end
                end
            else
                [t,j] = calc7st(p_target, jmax, amax, vmax, a0(i_a), v0(i_v), p0,false,false,false);
                %disp(sprintf('testing calc7st(%.20g, %g, %g, %g, %g, %g, %g, true)\n',p_target,jmax,amax,vmax,a0(i_a),v0(i_v),p0));
                [isCorrect, reason] = testjTracks(t,j,a0(i_a),v0(i_v),p0,jmax,amax,vmax,p_target);
                try
                if (~isCorrect)
                    err = sprintf('error in calc7st(%.20g, %g, %g, %g, %g, %g, %g):\n%s',p_target,jmax,amax,vmax,a0(i_a),v0(i_v),p0, reason);
                    disp(err); 
                    errors = [errors err '\n'];
                end
                catch
                    disp(lasterr);
                end
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