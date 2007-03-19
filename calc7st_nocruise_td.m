function [t_res,j_res] = calc7st_nocruise_td(dir,ptarget,jmax,amax,vmax,a0,v0,p0)
% Diese Funktion berechnet durch das numerische Auffinden der Nullstellen des
% Lösungspolynoms (aus 'formulars_7stp_TD_bruteforce.m') das korrekte DD-Profil.
% Mit dem Polynom wird t4 numerisch ermittelt, daraus werden dann t1, t2 und t3
% berechnet.
% Zurückgegeben werden 4 Zeitintervalle mit den dazugehoerigen jerk-Werten.
	% (1) Die kleinste reelle, positive Nullstellen des Polynoms für t4 finden.

	% (2) t1, t2 und t3 berechnen und zurückgeben.

t4 = sym('t4', 'positive');
% The polynomial.
plyn = eval(-ptarget + p0+1/2*v0/jmax*amax-v0/jmax/dir*a0+1/2/amax*dir*jmax^2*t4^4+1/2/jmax/dir^2/amax*a0^2*v0-1/2/dir/amax*v0^2-1/8/jmax^2/dir^3/amax*a0^4+1/2*dir*amax*t4^2-1/4*a0^2/jmax^2/dir*amax+1/3*a0^3/jmax^2/dir^2+jmax*dir*t4^3);

% (1)
% Find the roots.
ns = roots(sym2poly(plyn));
% Take the smallest positive real one.
t4 = ns(1);
for i=2:length(ns),
	if ((t4 < 0) || (~ isreal(t4)) || (((ns(i) >= 0) && (ns(i) < t4) && isreal(ns(i)))))
		t4 = ns(i);
	end
end

% (2)
% Calc other time intervalls and return them.
t1 = (-a0+amax*dir)/jmax/dir;
t2 = 1/2*(-2*v0*jmax*dir+a0^2-2*amax^2*dir^2+2*jmax^2*dir^2*t4^2)/jmax/dir^2/amax;
t3 = (amax+jmax*t4)/jmax;
t_res = [t1, t1+t2, t1+t2+t3, t1+t2+t3+t4];
j_res = [jmax*dir, 0, -jmax*dir, jmax*dir];