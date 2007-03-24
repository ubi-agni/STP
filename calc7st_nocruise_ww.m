function [t_res,j_res] = calc7st_nocruise_dd(dir,ptarget,jmax,amax,vmax,a0,v0,p0)
% Diese Funktion berechnet durch das numerische Auffinden der Nullstellen des
% Lösungspolynoms (aus 'formulars_7stp_DD_bruteforce.m') das korrekte DD-Profil.
% Mit dem Polynom wird t3 numerisch ermittelt, daraus werden dann t2 and t1 be-
% rechnet.
% Zurückgegeben werden 3 Zeitintervalle mit den dazugehoerigen jerk-Werten.
	% (1) Spezialfall: Falls v0 Richtung Ziel, a0 entgegen dieser Richtung
	% zeigt, und wir nicht zuerst die Beschleunigung auf Null reduzieren
	% können, da wir sonst doch über das Ziel schießen würden - dann muss
	% in der Gleichung zur Berechnung von t2 und t1 das Vorzeichen umgedreht
	% werden.
	
	% (2) Alle Nullstellen des Polynoms für t3 finden.
	
	% (3) Die beste/korrekten Lösungen finden (z.T. ergeben sich mehrere
	% (Schein-)Lösungen, da das Polynom durch Quadrieren aus der Orginalformel
	% hervorgegangen ist.

% (1)
% Fist test for a (very) special case:
% If v0 is in the right direction and a0 is in the wrong direction and we have no overshoot
% and after reducing the acceleration to zero the speed is still in the right direction and
% if we overshoot the target when reducing the speed to zero after that - then and only then
% this special case occours and we will start with a jerk of (-dir*jmax) instead of (dir*jmax)
p_special = -1/12*(-12*p0*jmax^2*dir^2+12*v0*a0*jmax*dir-4*a0^3-12*(4*v0*jmax*dir-2*a0^2)^(1/2)*dir^2*v0*jmax+6*(4*v0*jmax*dir-2*a0^2)^(1/2)*dir*a0^2+6*(4*v0*jmax*dir-2*a0^2)^(1/2)*dir^4*v0*jmax-3*(4*v0*jmax*dir-2*a0^2)^(1/2)*dir^3*a0^2)/jmax^2/dir^2;
if ((sign(v0)==dir) && (sign(a0)~=dir) && (sign(ptarget-p0) == dir) && (v0*jmax*dir-0.5*a0^2 >= 0) && (sign(ptarget-p_special) ~= dir))
	s_dir = -1;
else
	s_dir = 1;
end

% Get the polynomial for t3.
t3 = sym('t3','positive');
% This is the original formular...
%%%%%%plyn = (eval('jmax*dir*t3^3+p0+1/2*v0/jmax/dir*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2)-v0/jmax/dir*a0+1/2*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2)*t3^2-1/4*a0^2/jmax^2/dir^2*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2)+1/3*a0^3/jmax^2/dir^2-ptarget'))
% Which we transform into the polinomial...
s1 = (-1/4*a0^2/jmax^2/dir^2+1/2*t3^2+1/2*v0/jmax/dir);
s2 = (-p0-jmax*dir*t3^3+v0/jmax/dir*a0-1/3*a0^3/jmax^2/dir^2+ptarget);
plyn = expand(eval(s1^2*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)-s2^2));

% (2)
% Find the roots.
ns = roots(sym2poly(plyn));
% Only keep the real, positive ones.
clear pos_real_ns;
j = 1;
for i=1:length(ns),
	if ((isreal(ns(i))) && (ns(i) >= 0))
		pos_real_ns(j) = ns(i);
		j = j+1;
	end;
end

% (3)
% Now find the right/best one under the solutions in pos_real_ns.
best_t3 = max(pos_real_ns);
j_res = [dir*jmax, -dir*jmax, dir*jmax];
for i=1:length(pos_real_ns),
	t3 = pos_real_ns(i);
	t1 = -(a0-s_dir*1/2*dir*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2))/jmax/dir;
	t2 = (jmax*dir*t3+s_dir*1/2*dir*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2))/dir/jmax;
	[a,v,p] = calcjTracks([t1, t2, t3], j_res, a0,v0,p0);
	if ((isreal(t1)) && (isreal(t2)) && (t1 >= 0) && (t2 >=0) && (isZero(a)) && (isZero(v)) && (isZero(ptarget-p)) && (t3 < best_t3))
		best_t3 = t3;
	end
end

% Take the best one and return it.
t3 = best_t3;
t1 = -(a0-s_dir*1/2*dir*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2))/jmax/dir;
t2 = (jmax*dir*t3+s_dir*1/2*dir*(4*jmax^2*dir^2*t3^2+2*a0^2-4*v0*jmax*dir)^(1/2))/dir/jmax;

% find out when acc-graph crosses zero for the 7-phases data structure
a1 = a0+jmax*dir*t1;
t_res3 = abs(a1/jmax);
t_res5 = t2-t_res3;

t_res = [t1, 0, t_res3 , 0, t_res5, 0, t3];
j_res = [jmax*dir, 0, -jmax*dir, 0, -jmax*dir, 0, jmax*dir];