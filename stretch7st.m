function [t_res,j_res] = stretch7st(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0, plotMe)
% This functions stretches a given profile [t,j] to a new duration T.

if (nargin < 11) plotMe=false; end

if (T <= sum(t))
    disp('New duration must be smaller than current one.');
    disp('I did nothing...');
    t_res = t; j_res = j;
else
    t = splitNoCruiseProfile(t,j,a0);

    [dummy, v3, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
    dir = sign(v3); % TODO: what happens when dir=0?

    % check whether we must use the double deceleration branch
    [bUseDoubleDeceleration, td,jd] = useDoubleDeceleration (t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0);
    if (bUseDoubleDeceleration)
        % double deceleration branch
        disp('Double Deceleration Case');
        [t, j] = findProfileDoubleDec (td,jd,T, a0,v0,p0, ptarget, jmax,amax);
        % extend simple profile computed in useDoubleDeceleration() to
        % wedge-shaped profile
       if (t(1) ~= 0 && t(3) == 0) t(3) = 1; end
       if (t(1) == 0 && t(3) ~= 0) t(1) = 1; end
    else
      % normal profile branch
       disp('Normal Case');
      [t, j] = findProfileNormal (t,j,T, a0,v0,p0, ptarget, jmax,amax);
    end
    % Calculate exact phase duration for choosen profile t, j
    % generate set of equations
    [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
    SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
    t_res = solveAll ([A V TEQ SUM_EQ P], TVARS);
    j_res = j;
end

% display graph
if (plotMe)
	[a_end, v_end, p_end] = plotjTracks(t_res,j_res,a0,v0,p0, true, jmax,amax,vmax,ptarget);
end

return

function [bDoubleDec, t,j] = useDoubleDeceleration(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0);
% Tests for a given profile, whether stretching it to the time T would
% nead a double deceleration profile.    
% TODO TODO TODO:
% folgenden einfachen algorithmus verwenden:
% Profil erzeugen: erst a auf null, dann v auf null
% ist dieses profil zu langsam --> double deceleration / normal

if (sign(j(3)) ~= sign (j(5)))
    % that was easy - it already is a double dec. profile :)
    bDoubleDec = true;
    return;
end

% If the velocity change within the deceleration part is larger in magnitude than
% the velocity change within acceleration part (starting from max velocity,
% i.e. at a=0, the cutting process in findProfileNormal may lead to the
% situation, where no more area can be cut from the acceleration part,
% although the desired duration T is not yet reached. In this case we have
% to switch to the double deceleration branch as well.
% We check whether we are still too early at the target, when employing a
% full stop profile, reducing acceleration to zero immidiately in the first
% phase. 

if (a0 == 0)
    j1 = j(5); % direction of double deceleration
    t1 = 0;
else
    % jerk to decrease acceleration to zero
    j1 = -sign(a0) * jmax; 
    % time needed to reach zero acceleration
    t1 = abs(a0/j1); 
end

% position and velocity reached after this time
[a1 v1 p1] = calcjTracks (t1, j1, a0,v0,p0); %(a1==0)
% profile to reach zero velocity, starting from v1
[tdec jdec] = calc3st(0,jmax,amax,a1,v1);

% If the a(t) profile in deceleration part has the same direction as before,
% we may need to switch to a double deceleration profile. 
if (sign(jdec(1)) == sign(j(5))) % we may need to switch
    if (j1 == jdec(3)) % 
        t = [0 0 t1  0  tdec]; j = [jdec  0  jdec];
    else
        t = [t1 0 0  0  tdec]; j = [jdec  0  jdec];
    end
    % insert cruising phase, such that the target is still reached
    t = adaptProfile (t,j, ptarget, a0,v0,p0);
    % figure;[a_end, v_end, p_end] = plotjTracks(t,j,a0,v0,p0, true, jmax,amax,vmax,ptarget);
    bDoubleDec = stillTooShort (t, T);
else
    bDoubleDec = false;
end
return

function t = adaptProfile (t, j, p_target, a0,v0,p0)
% Given a profile (t,j) where acceleration and deceleration phase was
% already changed (cutted or shifted) such that velocity v(3) is smaller
% in magnitude than before, this function extends the cruising phase (or
% inserts one), such that the target is reach again.
% This is a simple linear equation...
[dummy, dummy, p_reached] = calcjTracks(t,j, a0,v0,p0); 
[dummy, v3_new, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
% enlarge cruising time, such that area below velocity profile equals dp again
t(4) = t(4) + (p_target - p_reached) / v3_new;

% figure; plotjTracks(t,j, a0,v0,p0); set(gcf, 'Name', 'adapted');
return


function [t,j] = findProfileDoubleDec (t,j,T, a0,v0,p0, ptarget, jmax,amax)
% find correct double deceleration profile by shifting area from second
% deceleration to first deceleration part.
% We can get two type of deceleration profiles here:
% 1) The time-optimal profile was already double deceleraton, leading to a(3) ~= 0
% 2) all other profiles: a(3) == 0, there might be profile [0 0 t3] / [t1 0 0]
a3 = calcjTracks (t(1:3), j(1:3), a0,v0,p0);
if (~isZero(a3))
    % 1) Time-optiomal profile was double deceleration already
    %    In this case we must check, whether we can reach a3 = 0 (to insert
    %    a cruising phase
    tz = abs(a3/jmax); % time needed to reach zero acceleration
    % try to shorten deceleration profile, such that a3 reaches zero in between
    [isPossible, tdec] = removeArea([t(5)+tz t(6) t(7)], abs(a3*tz), jmax,amax);
    % adapt profile to reach target again
    tn = adaptProfile ([t(1:2) t(3)+tz  0  tdec],j, ptarget, a0,v0,p0);
    % if target is overshooted if trying to set a3=0, 
    % adaptProfile will return negative time tn(4) (and we must keep the current profile)
    if (tn(4) < 0 || ~stillTooShort(tn,T))
        return % keep existing profile
    end
    t = tn; % use adapted profile for further checks
end

% Compute current velocity decrease achieved during first and second part
[a3 curFirst dummy] = calcjTracks(t(1:3),j, a0,0,0); % a3 = 0
[a7 curLast dummy]  = calcjTracks(t(5:7),j, 0,0,0); % a7 = 0
curFirst = abs(curFirst); curLast = abs(curLast);

% We enlarge curFirst such, that it contains the area of the (full) triangle 
% which reaches peak acceleration a2
if (a0 ~=0 && sign(a0) == sign(j(7)))
    % time needed to reach zero acceleration
    t_0acc = abs(a0/j(1));
    % add area on the other side of x-axis 
    % (because it was substracted during integration)
    curFirst = curFirst + abs(a0)*t_0acc/2;
else
    % time needed to reach zero acceleration (now backwards)
    t_0acc = -abs(a0/j(1));
    % add (virtual) area of the missing triangle
    curFirst = curFirst + abs(a0*t_0acc/2);
end

wedgeMax = amax^2/jmax;

while (1)
    % area needed to extend first part to full wedge
    deltaFirst = wedgeMax - curFirst;
    if (t(2) == 0 && ~isZero(deltaFirst)) % first part is not yet full wedge 
        if (t(6) == 0) 
            deltaLast  = curLast; % area available in second part
            % if last part has no enough area to extend first one to full
            % triangle, the profile will keep WW shape
            if (deltaFirst >= deltaLast) return; end
        else
            deltaLast = t(6)*amax; % area below const-trapezoidal part
        end
        deltaV = min (deltaFirst, deltaLast);
    else
        if (t(2) == 0) t(2) = 1; end % allow const-part in trapez
        if (t(6) == 0) return; % profile will keep TW shape
        else deltaV = t(6)*amax; end % area below const-trapezoidal part
    end

    tacc = addArea (t_0acc, curFirst + deltaV - wedgeMax, jmax,amax);
    [isPossible, tdec] = removeArea (t(5:7), deltaV, jmax,amax);
    tn = adaptProfile ([tacc t(4) tdec],j, ptarget, a0,v0,p0);
    % if we overshoot in time, t contains the correct profile
    if (~stillTooShort(tn,T)) return; end
    % otherwise continue probing with adapted profile
    t = tn;
    curFirst = curFirst + deltaV;
    curLast  = curLast  - deltaV;
end
return

function [t_res] = addArea(deltaT1, deltaV, jmax,amax)
% Compute a profile [t1 t2 t3] such that its area is wedgeMax + deltaV.
% deltaT1 is extra time added to t1

tmax = amax/jmax;
if (deltaV >= 0) % full wedge + const trapezoidal part
    t_res = [tmax + deltaT1, deltaV/amax, tmax];
else
    deltaT = tmax - sqrt (tmax^2 + deltaV/jmax);
    t_res = [tmax + deltaT1 - deltaT, 0 , tmax - deltaT];
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% normal profile functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t,j,type] = findProfileNormal(t,j,T, a0,v0,p0, ptarget, jmax,amax)
% find correct profile by cutting pieces and descending to shorter profiles

if (t(2) ~= 0) % T? profile
    if (t(6) ~= 0) 
        type = 'TT'; % TT profile
    else
        type = 'TW'; % TW profile
    end
else
    if (t(6) ~= 0) 
        type = 'WT'; % WT profile
    else
        type = 'WW'; % WW profile
    end
end

% if profile type does not change, we return t_orig, but with cruising
% phase inserted
t_orig = t;
if(t(4) == 0) t_orig(4)=1; end
if (strcmp (type, 'TT')) 
    % cut out smaller a=const. part
    dt = min(t(2), t(6));
    t(2) = t(2) - dt;
    t(6) = t(6) - dt;
    t = adaptProfile (t,j, ptarget, a0,v0,p0);
    if (stillTooShort(t,T))
        % recursively calling this function even cuts further
        [t,j,type] = findProfileNormal (t,j,T, a0,v0,p0, ptarget, jmax,amax);
    else
        % now we stop after duration time T, hence profile stays TT
        t = t_orig; % allow for a cruising phase
    end
    return
end

if (strcmp (type, 'WW'))
    t = t_orig; % allow for a cruising phase
    return % nothing to do, WW stays WW anytime
end

if (strcmp (type, 'WT'))
    a1 = a0 + j(1)*t(1);
    dt_w = min(t(1),t(3));
    area_w_max = abs(dt_w * (2*a1 - dt_w*j(1)));
    area_t_max = t(6)*amax;
    if (area_w_max > area_t_max)
        % we will cut out the whole t(6) WT -> WW
        t(6) = 0;
        dt = (abs(a1)-sqrt(a1^2-area_t_max))/jmax;
        t(1) = t(1)-dt;
        t(3) = t(3)-dt;
        t = adaptProfile (t,j, ptarget, a0,v0,p0);

        if (stillTooShort(t,T))
            type = 'WW'; % type switches to WW
        else
            % now we stop after duration time T, hence profile stays WT
            t = t_orig; % allow for a cruising phase
        end
    else
        % nothing to cut out, stays at WT
        t = t_orig; % allow for a cruising phase
    end
    return
end

if (strcmp (type, 'TW'))
    a5 = j(5)*t(5);
    area_w_max = abs(t(5)*a5);
    area_t_max = t(2)*amax;
    if (area_w_max > area_t_max)
        % we will cut out the whole t(2)
        t(2) = 0;
        t(5) = sqrt((area_w_max-area_t_max)/abs(j(5)));
        t(7) = t(5);
        t = adaptProfile (t,j, ptarget, a0,v0,p0);

        if (stillTooShort(t,T))
            % TODO checkDoubleDecel
            type = 'WW'; % type switches to WW
        else
            % now we stop after duration time T, hence profile stays TW
            t = t_orig; % return input profile
        end
    else
       % nothing to cut out, stays at TW
       t = t_orig; % allow for a cruising phase
    end
    return
end
return

function bTooShort = stillTooShort(t,T)
    bTooShort = (sum(t) < T);
return

function [tsplit] = splitNoCruiseProfile(t,j,a0)
    % In case of a profile without cruising phase, the time intervalls
    % t(3) and t(5) might be joined together into one of them so the other
    % one is zero. This can only occour if j(3) and j(5) have the same
    % sign.
    % For this stretching algorithm, we need to split the time intervall up
    % so the acc-graph reaches zero after t(3).
    tsplit = t; jsplit = j;
    if (t(4) ~= 0) return; end
    if (j(3) ~= j(5)) return; end
    tsplit(3) = abs((a0 + j(1)*t(1)) / j(3));   % = -a2/j(3)
    tsplit(5) = t(3) + t(5) - tsplit(3); % ==>tsplit(3)+tsplit(5)=t(3)+t(5)
return