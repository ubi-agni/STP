function [t, a] = calc3st(goal,aMax,vMax,v,cur)
% Berechnung des zeitoptimalen Bewegungsprofil mit Beschleunigungsimpulsen
% von a_max, bzw. -a_max. Zurückgegeben werden 3 Zeitintervalle mit den
% dazugehörigen acc-Werten.

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
    t(2) = vMax / aMax;

    % pos change from acceleration and deceleration only:
    deltaP = t(1) * (v + acc/2. * t(1));
    deltaP = deltaP + t(2) * (dir * vMax + dec/2. * t(2));

    % time in cruising phase:
    deltaT = (goal - cur - deltaP) / (dir * vMax);

    if (deltaT >= 0.0) % plan a complete (trapezoidal) profile:
        t(3) = t(1) + deltaT + t(2); % duration
        t(2) = t(3) - t(2);
        w = dir * vMax;
    else % plan an incomplete (triangular) profile:
        % w - speed at switching between acceleration and deceleration
        w = dir * sqrt (dir * aMax * (goal-cur) + v*v/2.);
        t(1) = (w - v) / acc;
        t(2) = t(1);
        t(3) = t(1) + abs (w / dec); % duration
    end
end

a(1) = acc;
a(2) = 0;
a(3) = dec;

% display graph
%plotaTracksNice(t, a, aMax, vMax, goal, v, cur);