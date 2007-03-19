function [t_res,j_res] = calc7st_nocruise(dp,t,dir,trapezoid_stop,ptarget,jmax,amax,vmax,a0,v0,p0)
% Berechnung des zeitoptimalen Bewegungsprofils mit Jerk-Impulsen von
% j_max, bzw. -j_max im Falle einer Bewegung ohne cruising-Phase (= Phase
% mit v = +-vmax und a=0).
% Als Input erhält die Funktion neben den Begrenzungen und Startwerten von
% j, a, v und p sowie der Richtung der "hypothetischen" cruising Phase auch
% noch die Zeiten für ein Bewegunsprofil in dem erst die Maximalgeschwindig-
% keit erreicht wird und dann sofort auf Null abgebremst wird (Grenzfall
% cruising-Phase-Zeit=0).
% Da es sich aber um eine Bewegung ohne cruising-Phase handelt, wird die
% Bewegung überschießen (ein- oder zweimal, je nach dir-Flag). Die Strecke
% die zuviel zurückgelegt wird, wird in 'dp' übergeben.
% Zurückgegeben werden n Zeitintervalle mit den dazugehoerigen jerk-Werten.
% (2<=n<=7, Normalfall: n=6)
	% (1) Teste, ob v0 > vmax. Falls ja, füge eine dec-Phase ein und rufe
	% 'calc7st' rekursiv mit den neuen Startwerten auf.
	
	% (2) Fallunterscheidung in die möglichen Profile DD, DT, TD, TT anhand
	% des übergebenen Bewegungsprofils einer v0->vmax->0 Bewegung:
		% (2a) DD-Profil. Bei v0>vmax ist DD und DT möglich, ansonsten nur
		%      DD. Es wird dementsprechend entweder 
		%      'calc7st_nocruise_dt_or_dd' oder dirket 'calc7st_nocruise_dd'
		%      aufgerufen.
		% (2b) DT-Profil. Es wird sich entweder ein DD oder DT Profil ergeben
		%      'calc7st_nocruise_dt_or_dd' wird aufgerufen.
		% (2c) TD-Profil. Es wird sich entweder ein DD oder DT Profil ergeben
		%      'calc7st_nocruise_td_or_dd' wird aufgerufen.
	
	% (3) = (2d) TT-Profil. Im Falle eines TT-Profils kann aus den beiden
	% a=amax Phasen jeweils ein Zeitstück dt herausgeschnitten werden, so dass
	% genau dp Strecke weniger zurückgelegt wird.
	% Allerdings kann es dabei dazu kommen, dass eine oder beider der a=amax
	% Phasen kürzer als dt sind und wir somit in einen DD/DT/TD Fall kommen.
	% Daher folgende Fallunterscheidung:
		% (3a) Beide a=amax Phasen lang genug. Rückgabe der ber. Bewegung.
		% (3b) Erste der beiden Phasen kürzer als die zweite. Aufruf von
		%      'calc7st_nocruise_dt_or_dd' bzw. 'calc7st_nocruise_dt', je
		%	   nachdem, ob dt <= zweite Phase.
		% (3c) Zweite der beiden Phasen kürzer als die erste. Aufruf von
		%      'calc7st_nocruise_td_or_dd' bzw. 'calc7st_nocruise_td', je
		%	   nachdem, ob dt <= erste Phase.
	

% (1)
% at first test for a special case:
% If abs(v0) is bigger than vmax, we need to slow down first, but since we dont have a cruising phase,
% the acceleration will not be zero at the time the speed crosses vmax.
% Therefore if we dont overshoot the target, we will have an additional decceleration phase in the
% very beginning, followed by the normal DT/TD/DD profile. This decceleration phase has to have
% exactly the length it would have, if we wanted to reach vmax exactly, in the case of an cruising phase.
% We will insert the new phase and call the calc7st_function recursively for the rest of the motion.
if ((abs(v0)>vmax) && (sign(ptarget-p0) == dir))
	% t_dec = (a0+1/2*(2*a0^2-4*dir*jmax*vmax+4*dir*jmax*v0)^(1/2))/jmax/dir
	[t_reach_vmax j_reach_vmax] = calc3st(sign(v0)*vmax, jmax, amax, a0, v0);
	t_dec = t_reach_vmax(2);
	if (~isZero(t_dec) && (t_dec > 0))
		if (isZero(t_reach_vmax(1)-t_reach_vmax(2)))
			t_reach_vmax = t_reach_vmax(1);
			j_reach_vmax = j_reach_vmax(1);
		else
			t_reach_vmax = t_reach_vmax(1:2);
			j_reach_vmax = j_reach_vmax(1:2);
		end
		[a1,v1,p1] = calcjTracks([0, t_reach_vmax], j_reach_vmax, a0, v0, p0);
		disp(sprintf('recursive call at calc7st_nocruise'));
 		[t_res1 j_res1] = calc7st(ptarget, jmax, amax, vmax, a1, v1, p1, false);
		t_res = [t_reach_vmax, t_res1+t_reach_vmax(length(t_reach_vmax))];
		j_res = [j_reach_vmax, j_res1];
		return
	end
end

% (2)
% check which case we have...
if ((isZero(t(1)-t(2))) && (isZero(t(4)-t(5))))
	% (2a)
	% [D-D] profile
	% To reach dir*vmax and to slow down to a halt afterwards results in a
	% DD-profile, but is already overshooting the target (once or twice,
	% depending on dir).
    % Attention: This does not mean, that the final result will also be a
    % (reduced) DD-profile, too - as we can see in the problem-case #06.
    % This is because with v0 != 0 the area under the acc-graph has to be
    % non-zero, too. So if, for example, v0<0 then a reduction of the
    % profile could lead to lower minimum acc-values in the acc-profile
    % than in the v0->vmax->0 profile and so a trapezoid case could occour.
    
    % TODO: Find conditions for differentiation between the sure DD and the
    % TODO: DT or DD case!
    % Apparently this can only happen when v0>vmax??
	if (abs(v0)>vmax)
        disp(sprintf('Decided on [D-T]/[D-D] in calc7st_nocruise! (1)'));
    	[t_res,j_res] = calc7st_nocruise_dt_or_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
    else
        disp(sprintf('Decided on [D-D] in clac7st_nocruise! (1)'));
        [t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
    end
elseif (isZero(t(1)-t(2)) && (t(4) < t(5)))
	% (2b)
	% [D-T] or [D-D] profile
	% To reach dir*vmax and then slow down to a halt, we get a DT-profile,
	% which is already overshooting the target (once or twice, depending on
	% dir). So we will need to reduce this profile and it could possibly
	% result in either a DT- or a DD-profile.
	disp(sprintf('Decided on [D-T]/[D-D] in calc7st_nocruise! (2)'));
	[t_res,j_res] = calc7st_nocruise_dt_or_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
elseif ((t(1) < t(2)) && (isZero(t(4)-t(5))))
	% (2c)
	% [T-D] or [D-D] profile
	% To reach dir*vmax and then slow down to a halt, we get a TD-profile,
	% which is already overshooting the target (once or twice, depending on
	% dir). So we will need to reduce this profile and it could possibly
	% result in either a TD- or a DD-profile.
	% In the case that we overshoot the target and have an trapezoid-shaped
	% imidiate-halt profile, it is sure that we will stay in the [T-D] case,
	% though.
	if ((sign(ptarget-p0) ~= dir) && trapezoid_stop)
		disp(sprintf('Decided on [T-D] in calc7st_nocruise! (3)'));
		[t_res,j_res] = calc7st_nocruise_td(dir, ptarget,jmax,amax,vmax,a0,v0,p0);
	else
		disp(sprintf('Decided on [T-D]/[D-D] in calc7st_nocruise! (4)'));
		[t_res,j_res] = calc7st_nocruise_td_or_dd(dir, ptarget,jmax,amax,vmax,a0,v0,p0);
	end
else
	% (3)
	% [T-T] or [T-D] or [D-T] or [D-D] profile
	% To reach dir*vmax and then slow down to a halt, we get a TT-profile.
	% But since we overshoot the target (once or twice, depending on dir),
	% we can't say anything about the profile yet, because by reducing it, it
	% could become everything from TT to DD.
	% We will therefore continue with calculating the time amount dt, which we
	% would need to cut out off the two a=amax phases in order to reach the
	% target exactly.
	v3 = v0+a0*(-a0+amax*dir)/jmax/dir+1/2/jmax/dir*(-a0+amax*dir)^2+amax*dir*(-1/amax/dir*v0+1/2/amax/dir^2*a0^2/jmax-1/jmax*amax+1/amax*vmax);
	t3 = t(4)-t(2);
	dt = 1/2/amax/dir*(t3*amax*dir+2*v3-dir*(t3^2*amax^2+4*t3*amax*dir*v3+4*v3^2+4*amax*dir*dp)^(1/2));
	
	% Now we compare dt to the length of the two a=amx phases.
	if ((dt <= t(2)-t(1)) && (dt <= t(5)-t(4)))
		% (3a)
		% We can cut dt out without complications
		% - it's really a [T-T] profile.
		disp(sprintf('Decided on [T-T] in calc7st_nocruise! (5)'));
		t_res = [t(1), t(2)-dt, t(4)-dt, t(5)-2*dt, t(6)-2*dt];
		j_res = [dir*jmax, 0, -dir*jmax, 0, dir*jmax];
	elseif (t(2)-t(1) < t(5)-t(4))
		% (3b)
		% [D-T] or [D-D] profile
		% We can can't cut out dt off both a=amax phases, because at least
		% one of them is too small. We need to consider another strategie and
		% knowing that the first a=amax phase is shorter than the second, we
		% can already be sure that only a DT- or DD-profile can be the
		% solution.
		if (dt <= t(5)-t(4))
            % The second a=amax phase is long enough to cut out dt. Is it
            % sure that we have a [D-T] case now and never a [D-D] case???
            disp(sprintf('Decided on [D-T] in calc7st_nocruise! (6)'));
		    [t_res,j_res] = calc7st_nocruise_dt(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        else 
            disp(sprintf('Decided on [D-T]/[D-D] in calc7st_nocruise! (7)'));
		    [t_res,j_res] = calc7st_nocruise_dt_or_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        end
	else
		% (3c)
		% [T-D] or [D-D] profile
		% We can can't cut out dt off both a=amax phases, because at least
		% one of them is too small. We need to consider another strategie and
		% knowing that the first a=amax phase is longer than the second, we
		% can already be sure that only a TD- or DD-profile can be the
		% solution.
		% In the case that we overshoot the target and have an trapezoid-shaped
		% imidiate-halt profile, it is sure that we will stay in the [T-D] case,
		% though.
        
        if (dt <= t(2)-t(1))
            % The first a=amax phase is long enough to cut out dt. Is it
            % sure that we have a [T-D] case now and never a [D-D] case???
            disp(sprintf('Decided on [T-D] in calc7st_nocruise! (8)'));
			[t_res,j_res] = calc7st_nocruise_td(dir, ptarget,jmax,amax,vmax,a0,v0,p0);
        else
            if ((sign(ptarget-p0) ~= dir) && trapezoid_stop)
			    disp(sprintf('Decided on [T-D] in calc7st_nocruise! (9)'));
   				[t_res,j_res] = calc7st_nocruise_td(dir, ptarget,jmax,amax,vmax,a0,v0,p0);
            else
    			disp(sprintf('Decided on [T-D]/[D-D] in calc7st_nocruise! (10)'));
    			[t_res,j_res] = calc7st_nocruise_td_or_dd(dir, ptarget,jmax,amax,vmax,a0,v0,p0);
            end
        end
    end	
end