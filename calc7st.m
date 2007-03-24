function [t,j] = calc7st(p_target,jmax,amax,vmax,a0,v0,p0,plotMe,bOverWrite)

% Berechnung des zeitoptimalen Bewegungsprofils mit Jerk-Impulsen von
% j_max, bzw. -j_max. Zurückgegeben werden n Zeitintervalle mit den dazu-
% gehoerigen jerk-Werten. (1<=n<=8, Normalfall: n=7)
	% (1) Teste, ob a0 > amax. Falls ja, füge dec-Phase ein und rufe 'calc7st'
	% rekursiv mit den neuen Startwerten auf.
	
	% (2) Teste, ob bei sofortigem Reduzieren der Beschleunigung auf Null vmax
	% überschritten wird. Falls ja, füge dec-Phase ein und rufe 'calc7st'
	% rekursiv mit den neuen Startwerten auf.
	
	% (3) Berechne das direction-flag anhand der sich ergebenden Endposition
	% bei einem sofortigem Abbremsen auf Null.
	
	% (4) Berechne mit 'calc3st' den Verlauf der Bewegung bei
	% schnellstmöglichem Erreichen von vmax und anschließendem schnellst-
	% möglichen Abbremsen auf Null. (Cruising-Phase Fall mit t_cruise = 0)
	% Anhand des Verlaufs wird t_cruise berechnet - die Länge der Cruising-
	% phase, um ptarget genau zu erreichen.
	
	% (5) Ist t_cruise>=0, so ist das gesuchte Profil gefunden und wird
	% zurückgegeben, ansonsten wird 'calc7st_nocruise' aufgerufen.

if (nargin < 9) bOverWrite=true; end
if (nargin < 8) plotMe=false; end

% (1)
% at first test whether abs(a0) > amax.
% If so, insert a first phase to decrease it to amax and calculate the rest
% in a recursive call of this function:
if (~isZero(abs(a0)-amax) && (abs(a0) > amax))
	t_dec = abs(a0-amax)/jmax;
	dir = sign(amax-a0);
	a1 = a0+t_dec*dir*jmax;
	v1 = v0+a0*t_dec+(1/2)*dir*jmax*t_dec^2;
	p1 = p0+v0*t_dec+(1/2)*a0*t_dec^2+(1/6)*dir*jmax*t_dec^3;
	disp(sprintf('recursive call at calc7st #1'));
	[t1 j1] = calc7st(p_target, jmax, amax, vmax, a1, v1, p1, false);
	t = [t_dec, t1+t_dec];
	j = [dir*jmax, j1];
	if (plotMe)
		[a_end, v_end, p_end] = plotjTracksNice(t,j,jmax,amax,vmax,p_target,a0,v0,p0, bOverWrite);
	end
	return
end

% (2)
% A second special case where we need to deaccelerate first:
% When due to a0 the speed will exceed the maximum speed, we have to decrease
% the acceleration to 0 as fast as possible (maybe afterwards even further,
% but a recursive call will take care of that).
% Caution:
% Only enter this case, if v0 is smaller than vmax. Otherwise the situation will
% be dealt with at other places.
t_stop_acc = abs(a0/jmax);
vmax_stop_acc = v0 + a0*t_stop_acc - sign(a0)*0.5*jmax*t_stop_acc^2;
if (~isZero(abs(v0)-vmax) && (abs(v0) < vmax) && ~isZero(t_stop_acc) && (abs(vmax_stop_acc) > vmax))
	a1 = 0;
	v1 = vmax_stop_acc;
	p1 = p0+v0*t_stop_acc+(1/2)*a0*t_stop_acc^2-sign(a0)*(1/6)*jmax*t_stop_acc^3;
	disp(sprintf('recursive call at calc7st #2'));
	[t1 j1] = calc7st(p_target, jmax, amax, vmax, a1, v1, p1,false);
	t = [t_stop_acc, t1+t_stop_acc];
	j = [-sign(a0)*jmax, j1];
	if (plotMe)
		[a_end, v_end, p_end] = plotjTracksNice(t,j,jmax,amax,vmax,p_target,a0,v0,p0, bOverWrite);
	end
	return
end

% (3)
% calculate the dir-flag by testing if we over-shoot the target
% at imidiate halt
[t_stop a_stop] = calc3st(0,jmax,amax,a0,v0);
[ah vh p_stop] = calcjTracks([0 t_stop],a_stop,a0,v0,p0);
% do we reach amax during the stop?
if (~isZero(t_stop(1)-t_stop(2)))
	trapezoid_stop = true;
else
	trapezoid_stop = false;
end
% get direction
dir = sign(p_target-p_stop);
p_fullstop = p_stop;

% (4)
% position change just from acc and dec phase:
[t_acc a_acc] = calc3st(dir*vmax,jmax,amax,a0,v0); % acc. part (before cruising)
[t_dec a_dec] = calc3st(0,jmax,amax,0,dir*vmax); % dec. part (after cruising)
% postion change:
[ah vh p_stop] = calcjTracks([0 t_acc t_dec+t_acc(3)],[a_acc a_dec], a0, v0, p0);
% distance we need to go in cruising phase:
p_delta = (p_target-p_stop);
t_delta = p_delta / (dir*vmax);

disp (sprintf ('full stop at: %f  zero-cruise: %f', p_fullstop, p_stop));

% (5)
% case differentiation: Do we have a cruising phase?
if (t_delta < 0)
	% without cruising phase
	[t, j] = calc7st_nocruise(p_delta,[t_acc t_dec+t_acc(3)],dir,trapezoid_stop,p_target,jmax,amax,vmax,a0,v0,p0);
else
    % with cruising phase
    t = [t_acc t_delta+t_acc(3) t_dec+t_delta+t_acc(3)];
    j = [a_acc 0 a_dec];                
end

% display graph
if (plotMe)
	[a_end, v_end, p_end] = plotjTracksNice(t,j,jmax,amax,vmax,p_target,a0,v0,p0, bOverWrite);
end
