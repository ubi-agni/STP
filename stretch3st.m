% Copyright 2009 Erik Weitnauer, Robert Haschke
%
% This file is part of Smooth Trajectory Planner for Matlab.
%
% Smooth Trajectory Planner for Matlab is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% Smooth Trajectory Planner for Matlab is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with Smooth Trajectory Planner for Matlab.  If not, see <http://www.gnu.org/licenses/>.
function [t, a] = stretch3st(newDuration, goal, aMax, vMax, v, cur)
% Berechnung des zeitoptimalen Bewegungsprofil mit Beschleunigungsimpulsen
% von a_max, bzw. -a_max und nachfolgendes Skalieren des Profils auf die
% übergebene Zeit. Nur Verlängerungen der Zeit sind zulässig.
% Zurückgegeben werden 3 Zeitintervalle mit den dazugehörigen acc-Werten.

bDoubleDeceleration = false;

% compute time needed for full stop
dir = -sign(v); % direction of acceleration to stop
stop = abs(v) / aMax;
% compute final position after full stop
stop = cur + stop * (v + dir * aMax/2. * stop);
   
if (goal == stop)
    % after full stop, we are already at the goal
    % no acceleration and cruising phase
    % only deceleration
    t(1) = 0;
    t(2) = 0;
    w = 0;
    acc = 0;
    t(3) = abs(v) / aMax;
    dec = dir * aMax;
else
    % direction of cruising phase
    dir = sign (goal - stop); 
    % (typical) direction of acceleration / deceleration
    acc = dir * aMax;
    dec = -dir * aMax;

    % time to reach cruising speed dir * vMax (clipping to zero?)
    t(1) = (dir * vMax - v) / acc;
    if (t(1) < 0)
        % deceleration to lower max speed than current speed needed
        acc = -acc;
        t(1)  = -t(1);
        bDoubleDeceleration = true;
    end
    % time to stop from cruising
    t(3) = vMax / aMax;

    % pos change from acceleration and deceleration only:
    deltaP = t(1) * (v + acc/2. * t(1));
    deltaP = deltaP + t(3) * (dir * vMax + dec/2. * t(3));

    % time in cruising phase:
    deltaT = (goal - cur - deltaP) / (dir * vMax);

    if (deltaT >= 0.0) % plan a complete (trapezoidal) profile:
        t(2) = deltaT;
        w = dir * vMax;
    else % plan an incomplete (triangular) profile:
        % w - speed at switching between acceleration and deceleration
        w = dir * sqrt (dir * aMax * (goal-cur) + v*v/2.);
        t(1) = (w - v) / acc;
        t(2) = 0;
        t(3) = abs (w / dec); % duration
    end
end

a(1) = acc;
a(2) = 0;
a(3) = dec;

% now scale up to new duration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% it should be longer than the optimal time we computed
if (newDuration < sum(t(1:3)))
	disp(sprintf('Error: You tried to scale to a time shorter than the optimal!'));
	return;
end

% old cruising time
tcruise = t(2);
A = abs(w) * (newDuration - sum(t(1:3))) / aMax;
B = newDuration - sum(t(1:3)) + tcruise;

% compute time delta to steel from acc + decl phase
deltaT = -B/2. + sqrt (B*B/4. + A); % > 0

if (~bDoubleDeceleration && (t(1) - deltaT >= 0))
    t(1) = t(1) - deltaT;
    t(3) = t(3) - deltaT;
    t(2) = newDuration - t(1) - t(3);
    w = v + 2. * 0.5*acc * t(1);
else
	% compute time needed for full stop
    dir = -sign (v); % direction of acceleration to stop
    stopT = abs(v / aMax);
    % compute final position after full stop
    stop = cur + stopT * (v + dir * aMax/2. * stopT);

    % cruising speed:
    w = (goal - stop) / (newDuration - stopT);
    % turn acceleration into deceleration:
    if (~bDoubleDeceleration)
		a(1) = -a(1);
        bDoubleDeceleration = true;
    end
	
	% time to reach cruising speed:
    t(1) = abs(w - v) / aMax;
    t(3) = stopT-t(1);
    t(2) = newDuration - t(1) - t(3);
end

% display graph
plotaTracks(t, a, aMax, vMax, goal, v, cur, false);