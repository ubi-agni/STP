function [t_res,j_res] = stretch7st(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0, plotMe)
% This functions stretches a given profile [t,j] to a new duration T.

if (nargin < 11) plotMe=false; end

[dummy, v3, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
dir = sign(v3); % TODO: what happens when dir=0?

% check if we must use the double deceleration branch
if (sign(j(3)) ~= sign (j(5))) bDoubleDec = true;
else bDoubleDec = false; end

if (bDoubleDec)
    % double deceleration branch
    [t, j] = findProfileDoubleDec (t,j);
else
    % This is a usual profile with acceleration and deceleration part.
    % If the area below the deceleration part is larger than the area
    % below the acceleration part (without negative parts) we may have to
    % switch to a double deceleration branch (for very large slow down).
    % This extends the decision tree of the no-cruise case by some
    % more cases... 8-(
    
    % normal profile branch
    [t, j] = findProfileNormal (t,j,T, a0,v0,p0, ptarget, jmax,amax);
end
% Calculate exact phase duration for choosen profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
SUM_EQ = sym(strcat('t1+t2+t3+t4+t5+t6+t7=', char(sym(T))));
t_res = solveAll ([A V TEQ SUM_EQ P], TVARS);
j_res = j;

% display graph
if (plotMe)
	[a_end, v_end, p_end] = plotjTracks(t_res,j_res,a0,v0,p0, true, jmax,amax,vmax,ptarget);
end

return

function [t, j] = adaptProfile (t, j, dp, a0,v0,p0)
% Given a profile (t,j) where acceleration and deceleration phase was
% already changed (cutted or shifted) such that velocity v(3) is smaller
% in magnitude than before, this function extends the cruising phase (or
% inserts one), such that the target is reach again.
% This is a simple linear equation...
[dummy, dummy, p_reached] = calcjTracks(t,j, a0,v0,p0); 
dp_new = p_reached - p0; % compute position delta during profile evolution
[dummy, v3_new, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
% enlarge cruising time, such that area below velocity profile equals dp again
t(4) = t(4) + (dp - dp_new) / v3_new;

% figure; plotjTracks(t,j, a0,v0,p0); set(gcf, 'Name', 'adapted');
return


% TODO
function [t,j,type] = findProfileDoubleDec (t,j,T)
return

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
    
t_orig = t; j_orig = j;
if (strcmp (type, 'TT')) 
    % cut out smaller a=const. part
    dt = min(t(2), t(6));
    t(2) = t(2) - dt;
    t(6) = t(6) - dt;
    [t,j] = adaptProfile (t,j, ptarget-p0, a0,v0,p0);

    if (stillTooShort(t,T))
        % recursively calling this function even cuts further
        [t,j,type] = findProfileNormal (t,j,T, a0,v0,p0, ptarget, jmax,amax);
    else
        % now we stop after duration time T, hence profile stays TT
        t = t_orig; % return input profile
    end
    return
end

if (strcmp (type, 'WW'))
    % allow for a cruising phase
    t(4) = 1;
    % TODO checkDoubleDecel
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
        [t,j] = adaptProfile (t,j, ptarget-p0, a0,v0,p0);

        if (stillTooShort(t,T))
            % TODO checkDoubleDecel
            type = 'WW'; % type switches to WW
        else
            % now we stop after duration time T, hence profile stays WT
            t = t_orig; % return input profile
        end
    else
        % TODO checkDoubleDecel
        % nothing to cut out, stays at WT
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
        [t,j] = adaptProfile (t,j, ptarget-p0, a0,v0,p0);

        if (stillTooShort(t,T))
            % TODO checkDoubleDecel
            type = 'WW'; % type switches to WW
        else
            % now we stop after duration time T, hence profile stays TW
            t = t_orig; % return input profile
        end
    else
        % TODO checkDoubleDecel
        % nothing to cut out, stays at TW
    end
    return
end
return

function [t,j] = checkDoubleDecel (t,j, type)
    % If the area below the deceleration part is larger than the area
    % below the acceleration part (without negative parts???) we may have to
    % switch to a double deceleration branch (for very large slow down).
    
    % TODO: to be implemented
return        

function bTooShort = stillTooShort(t,T)
    bTooShort = (sum(t) < T);
return
