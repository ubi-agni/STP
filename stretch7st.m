function [t_res,j_res] = stretch7st(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0, plotMe)
% This functions stretches a given profile [t,j] to a new duration T.

if (nargin < 11) plotMe=false; end

if (T <= sum(t))
    disp('New duration must be smaller than current one.');
    disp('I did nothing...');
    t_res = t; j_res = j;
elseif (numel(t) == 3)
    disp('This is a fullstop-profile which can''t be stretched');
    t_res = [t 0 0 0 0]; j_res = [j 0 -j];
else
    t = splitNoCruiseProfile(t,j,a0);
    if (j(1) == 0)
        j(1) = -sign(v0)*jmax;
        j(3) = -j(1);
    end

    [dummy, v3, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
    dir = sign(v3); % TODO: what happens when dir=0?

    % check whether we must use the double deceleration branch
    [bUseDoubleDeceleration, td,jd] = useDoubleDeceleration (t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0);
    if (bUseDoubleDeceleration)
        % double deceleration branch
        disp('Double Deceleration Case');
        [t, j] = findProfileDoubleDec (td,jd,T, dir,a0,v0,p0, ptarget, jmax,amax,vmax);
        % extend simple profile computed in useDoubleDeceleration() to wedge-shaped profile
        t(1) = 1; t(3) = 1;
    else
      % normal profile branch
       disp('Normal Case');
      [t, j] = findProfileNormal (t,j,T, a0,v0,p0, ptarget, jmax,amax);
    end
    % Calculate exact phase duration for choosen profile t, j
    % generate set of equations
    
    % TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    % this is a very ugly solution for the problem that we can't detect
    % cases in which a ddec profile with cruising phase switches into a
    % profile without cruising phase.
    % We will just set t(4) to zero and try.
    
    % The unlikely case we cant detect:
    if (bUseDoubleDeceleration) && (~isZero(t(4)))
        try % try the normal solution
            [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
            SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
            t_res = solveAll ([A V TEQ SUM_EQ P], TVARS);
            j_res = j;
            % display graph
            if (plotMe)
                [a_end, v_end, p_end] = plotjTracks(t_res,j_res,a0,v0,p0, true, jmax,amax,vmax,ptarget);
            end
            return;
        end
        % it didnt work --> so lets try with t(4) = 0
        t(4) = 0;
        [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
        SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
        t_res = solveAll ([A V TEQ SUM_EQ P], TVARS);
        j_res = j;
    else
        % the normal solution
        [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
        SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
        t_res = solveAll ([A V TEQ SUM_EQ P], TVARS);
        j_res = j;  
    end
end

% display graph
if (plotMe)
	[a_end, v_end, p_end] = plotjTracks(t_res,j_res,a0,v0,p0, true, jmax,amax,vmax,ptarget);
end

return

function [bDoubleDec, t,j] = useDoubleDeceleration(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0);
% Tests for a given profile, whether stretching it to the time T would
% nead a double deceleration profile.    
% folgenden einfachen algorithmus verwenden:
% Profil erzeugen: erst a auf null, dann v auf null
% ist dieses profil zu langsam --> double deceleration, sonst normal

if (sign(j(3)) ~= sign (j(5)))
    % that was easy - it already is a double dec. profile :)
    bDoubleDec = true;
    if (abs(a0) > amax) && (sign(a0) == sign(j(5)))
        j(1) = j(3);
    end
    return;
end

% If the velocity change within the deceleration part is larger in magnitude than
% the velocity change within acceleration part (starting from max velocity,
% i.e. at a=0), the cutting process in findProfileNormal may lead to the
% situation, where no more area can be cut from the acceleration part,
% although the desired duration T is not yet reached. In this case we have
% to switch to the double deceleration branch as well.

% We compute a profile, which immediately decreases acceleration to zero
% in the first (acceleration) phase and subsequently does a full stop to zero
% velocity in the second phase. In between an appropriate cruising phase is
% inserted to reach the final position. If this profile still is to short,
% we need to switch to a double deceleration profile.

if (a0 == 0)
    t1 = 0;
    % profile to reach zero velocity, starting from a0=0, v0
    [tdec jdec] = calc3st(0,jmax,amax,a0,v0);
else
    % jerk to decrease acceleration to zero
    j1 = -sign(a0) * jmax; 
    % time needed to reach zero acceleration
    t1 = abs(a0/jmax); 
    % position and velocity reached after this time
    [a1 v1 p1] = calcjTracks (t1, j1, a0,v0,p0); %(a1==0)
    % profile to reach zero velocity, starting from v1
    [tdec jdec] = calc3st(0,jmax,amax,a1,v1);
end

% If the a(t) profile in deceleration part has the same direction as before,
% we may need to switch to a double deceleration profile. 
% Otherwise, the velocity change in deceleration phase was smaller than
% in acceleration phase, hence no switch is neccessary.
if (sign(jdec(1)) == sign(j(5))) % we may need to switch
    if (sign(a0) == sign(j(5)))
        t = [0 0 t1  0  tdec]; j = [jdec  0  jdec];
    else
        t = [t1 0 0  0  tdec]; j = [jdec  0  jdec];
    end
    % for the case that we need to reduce the acceleration first, because
    % its over the limit, we need to rearrage the jerks a bit...
    if (abs(a0) > amax) && (sign(a0) == sign(j(5)))
        t1a = (abs(a0)-amax)/jmax;
        t = [t1a 0 t1-t1a  0  tdec]; j = [jdec  0  jdec]; j(1) = -j(1);
    end
    % insert cruising phase, such that the target is still reached
    t = adaptProfile (t,j, ptarget, a0,v0,p0);
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

%figure; plotjTracks(t,j, a0,v0,p0); set(gcf, 'Name', 'adapted');
return


function [t,j] = findProfileDoubleDec (t,j,T, dir, a0,v0,p0, ptarget, jmax,amax,vmax)
% find correct double deceleration profile by shifting area from second
% deceleration to first deceleration part.
% We can get two type of deceleration profiles here:
% 1) The time-optimal profile was already double deceleraton, leading to a(3) ~= 0
% 2) all other profiles: a(3) == 0, there might be profile [0 0 t3] / [t1 0 0]
a3 = calcjTracks (t(1:3), j(1:3), a0,v0,p0);
if (isZero(a3))
    % We need to differentiate between two cases:
    % Either j(1) and j(3) have different sign, in that case, the first part of
    % the profile will resemble a wedge or a trapezoid, respectively.
    % When they have the same sign (in case a0 > amax), it will have the form
    % of a slope or a stair, respectively.
    if (sign(j(1)) == sign(j(3))) && (j(1) ~= 0)
        if (t(6) == 0)
            t(2) = 1;
            return;
        end
        % move all the area from second trapezoid part to the first
        t(2) = t(2) + t(6);
        t(6) = 0;
        tn = adaptProfile (t,j,ptarget,a0,v0,p0);
        % if we overshoot in time, t contains the correct profile
        if (stillTooShort(tn,T)) 
            % we need to allow t(2) to be different from zero
            t(2) = 1;
            return;
        end
        % otherwise the second part will stay trapezoid
        t(6) = 1;
        t(2) = 1;
        return;
    end

    if (sign(a0) ~= sign(j(5)))
        % In the shifting process, we may only consider the velocity change *after*
        % reaching zero acceleration. Hence, we compute new initial conditions, reached
        % after this initial acceleration decrease.
        t0 = abs(a0/jmax);
    else
        % To ease computation during shifting, we extend the first phase
        % to an full profile, starting at zero acceleration
        t0 = -abs(a0/jmax);
    end
    % compute initial position at zero acceleration
    [a0_ v0_ p0_] = calcjTracks (t0, j(1), a0,v0,p0);
    t(1) = t(1) - t0;

    [t, j] = shiftDoubleDecArea (t,j,T-t0, a0_,v0_,p0_, ptarget, jmax,amax);
    t(1) = t(1) + t0;
    
    % if we got a valid result, a profile with cruising phase is possible.
    % Otherwise we have to enter the a3 <> 0 case.
    if (t(4) >= 0) return; end
end
    
    % We now have to stretch a double deceleration profile without a
    % cruising phase, which is the most complex possible case.
    % It turns out, that we can't distinguish between the 8 profile types
    % without actually trying to compute them and see whether we get a
    % correct solution or not.
    % However - since we will shift area under the acc-graph
    % from the second part of the movement to the first, the unstretched
    % profile type already limits the possible outcomes:
    % TT==>{TT,TW}, TW==>{TW}, WT==>{WT,WW,TW,TT}, WW==>{WW,TW}
    % In each of this cases, t4 could either be zero or not.
    
    % First put all profiles to test as columns into a matrix:
    clear Mt;
    if (t(2) ~= 0) && (t(6) ~= 0) % TT
        Mt{1} = [1 1 1 1 1 1 1]; Mt{2} = [1 1 1 0 1 1 1]; % TT
        Mt{3} = [1 1 1 1 1 0 1]; Mt{4} = [1 1 1 0 1 0 1]; % TW
    elseif (t(2) ~= 0) && (t(6) == 0) % TW
        Mt{1} = [1 1 1 1 1 0 1]; Mt{2} = [1 1 1 0 1 0 1]; % TW
    elseif (t(2) == 0) && (t(6) ~= 0) % WT
        Mt{1} = [1 1 1 1 1 1 1]; Mt{2} = [1 1 1 0 1 1 1]; % TT
        Mt{3} = [1 1 1 1 1 0 1]; Mt{4} = [1 1 1 0 1 0 1]; % TW
        Mt{5} = [1 0 1 1 1 1 1]; Mt{6} = [1 0 1 0 1 1 1]; % WT
        Mt{7} = [1 0 1 1 1 0 1]; Mt{8} = [1 0 1 0 1 0 1]; % WW
    elseif (t(2) == 0) && (t(6) == 0) % WW
        Mt{1} = [1 1 1 1 1 0 1]; Mt{2} = [1 1 1 0 1 0 1]; % TW
        Mt{3} = [1 0 1 1 1 0 1]; Mt{4} = [1 0 1 0 1 0 1]; % WW
    end
    
    % Now test all profiles in Mt until we found the right one:
    for ii=1:numel(Mt),
       try
           [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(Mt{ii}, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
           SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
           t = solveAll ([A V TEQ SUM_EQ P], TVARS);
           % in some cases we might get an additional solution that is
           % oszillating in the acceleration. So we need to check, whether
           % a3 has the same sign as -dir.
           a3 = a0 + t(1)*j(1) + t(2)*j(2) + t(3)*j(3);
           if (isZero(a3)) || (sign(a3) ~= sign(dir))
               % no oszillation, we found the correct profile
               return;
           end
           % oszillation, we need to continue the search
       end
    end
    disp('This should not happen: Found no solution when stretching double dec profile');
    return   

function [t,j] = shiftDoubleDecArea (t,j,T, a0,v0,p0, ptarget, jmax,amax)
% Compute current velocity decrease achieved during first and second part
    [a3 curFirst dummy] = calcjTracks(t(1:3),j, a0,0,0); % a3 = 0
    [a7 curLast dummy]  = calcjTracks(t(5:7),j, 0,0,0); % a7 = 0
    curFirst = abs(curFirst); curLast = abs(curLast);

    wedgeMax = amax^2/jmax;

    while (1)
        % area needed to extend first part to full wedge
        deltaFirst = wedgeMax - curFirst;
        if (t(2) == 0 && ~isZero(deltaFirst)) % first part is not yet full wedge 
            if (t(6) == 0) 
                deltaLast  = curLast; % area available in second part
                % if last part has no enough area to extend first one to full
                % triangle, the profile will keep WW shape
                if (deltaFirst >= deltaLast) return;
                end
            else
                deltaLast = t(6)*amax; % area below const-trapezoidal part
            end
            deltaV = min (deltaFirst, deltaLast);
        else
            if (t(2) == 0) t(2) = 1; end % allow const-part in trapez
            if (t(6) == 0) return; % profile will keep TW shape
            else deltaV = t(6)*amax; end % area below const-trapezoidal part
        end

        tacc = addArea (curFirst + deltaV - wedgeMax, jmax,amax);
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

function [t_res] = addArea(deltaV, jmax,amax)
% Compute a profile [t1 t2 t3] such that its area is wedgeMax + deltaV.

tmax = amax/jmax;
if (deltaV >= 0) % full wedge + const trapezoidal part
    t_res = [tmax, deltaV/amax, tmax];
else
    deltaT = tmax - sqrt (tmax^2 + deltaV/jmax);
    t_res = [tmax - deltaT, 0 , tmax - deltaT];
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
    %%%dt_w = min(t(1),t(3));
    
    %old ?wrong? version: area_w_max = abs(dt_w * (2*a1 - dt_w*j(1)));
    %neu version:
    
    %%%area_w_max = 0.5*dt_w*dt_w*jmax;
    %%%if (t(1) > t(3)) area_w_max = area_w_max + abs(2*a0*dt_w); end;
    
    area_w_max = t(3)*t(3)*jmax;
    if (t(1) < t(3))
        area_w_max = area_w_max - 0.5*a0*a0/jmax;
    end
    
    area_t_max = t(6)*amax;
    if (area_w_max > area_t_max)
        % we will cut out the whole t(6) WT -> WW
        t(6) = 0;
        dt = (abs(a1)-sqrt(a1^2-area_t_max*jmax))/jmax;
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
        
        % for the case the t area and the second wedge are exactly same,
        % the result is a fullstop. In this case we stay a TW profile.
        if (isZero(t(7)))
            t = t_orig;
        else
            t = adaptProfile (t,j, ptarget, a0,v0,p0);
            % t(4) might get smaller than zero, when due to the area
            % switching, the direction flag of the motion changes. In
            % that case, we stay a TW profile.
            if (t(4) >= 0) && (stillTooShort(t,T))
                % TODO checkDoubleDecel
                type = 'WW'; % type switches to WW
            else
                % now we stop after duration time T, hence profile stays TW
                t = t_orig; % return input profile
            end
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