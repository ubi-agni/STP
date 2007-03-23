function [t_res,j_res] = calc7st_nocruise_dt(dir,ptarget,jmax,amax,vmax,a0,v0,p0)
% Diese Funktion berechnet durch das numerische Auffinden der Nullstellen des
% Lösungspolynoms (aus 'formulars_7stp_DT_bruteforce.m') das korrekte DD-Profil.
% Mit dem Polynom wird t1 numerisch ermittelt, daraus werden dann t2, t3 und t4
% berechnet.
% Zurückgegeben werden 4 Zeitintervalle mit den dazugehoerigen jerk-Werten.
	% (1) Die kleinste reelle, positive Nullstellen des Polynoms für t1 finden.

	% (2) t2, t3 und t4 berechnen und zurückgeben.

t1 = sym('t1', 'positive');
% The polynomial.
plyn = eval(-ptarget + jmax*dir*t1^3+2*v0*t1+p0+3*a0*t1^2+1/jmax/dir*v0*a0+1/4/jmax^2/dir*a0^2*amax+1/3/jmax^2/dir^2*a0^3+1/2/jmax*v0*amax+2/jmax/dir*a0^2*t1+1/2*dir*t1^2*amax+1/jmax*a0*t1*amax+1/2/dir/amax*v0^2+1/2*jmax^2*dir/amax*t1^4+1/2/jmax/dir^2/amax*a0^2*v0+1/jmax/dir^2/amax*t1*a0^3+1/8/jmax^2/dir^3/amax*a0^4+jmax/amax*v0*t1^2+2*jmax/amax*a0*t1^3+5/2/dir/amax*a0^2*t1^2+2/dir/amax*v0*a0*t1);

% (1)
% Find the roots.
ns = roots(sym2poly(plyn));
% Take the smallest positive real one.
t1 = ns(1);
for i=2:length(ns),
	if ((t1 < 0) || (~ isreal(t1)) || (((ns(i) >= 0) && (ns(i) < t1)  && isreal(ns(i)))))
		t1 = ns(i);
	end
end

% (2)
% Calc other time intervalls and return them.
t2 = (a0+jmax*dir*t1+amax*dir)/jmax/dir;
t3 = 1/2*(2*v0*jmax*dir+a0^2-2*amax^2*dir^2+2*jmax^2*dir^2*t1^2+4*a0*t1*jmax*dir)/jmax/dir^2/amax;
t4 = amax/jmax;
t_res = [t1, t1+t2, t1+t2+t3, t1+t2+t3+t4];
j_res = [jmax*dir, -jmax*dir, 0, jmax*dir];