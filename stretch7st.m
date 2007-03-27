function [t_res,j_res] = stretch7st(t,j,T, ptarget,jmax,amax,vmax, a0,v0,p0)
% This functions stretches a given profile [t,j] to a new duration T.

% check if we have must use tje double deceleration branch
if (sign(j(3)) ~= sign (j(5))) bDoubleDec = true;
else
    % calc full-stop profile
    [t_stop a_stop] = calc3st(0,jmax,amax,a0,v0);
    % if full stop deceleration profile crosses zero we must cut there
    % otherwise ???
    [tn, jn] = adaptProfile ([t_stop 0 0 0 0], [a_stop 0 0 0 0], ptarget-p0);
    % if still too fast
    if (sum(tn(1:7)) < T)
        bDoubleDec = true;
        [t, j] = [tn, jn];
    else
        bDoubleDec = false;
    end
end

if (bDoubleDec)
    % double deceleration branch
    [t, j] = findProfileDoubleDec (t,j);
else
    % normal profile branch
    [t, j] = findProfileNormal (t,j);
end
[t_res, j_res] = solveProfile (t,j);
return

function [t, j] = adaptProfile (t, j, dp)
% Given a profile (t,j) where acceleration and deceleration phase was
% already changed (cutted or shifted) such that velocity v(3) is smaller
% in magnitude than before, this function extends the cruising phase (or
% inserts one), such that the target is reach again.
% This is a simple linear equation...
[dummy, dummy, dp_new] = calcjTracks(t,j, a0,v0,p0);
[dummy, v3_new, dummy] = calcjTracks(t(1:3),j, a0,v0,p0);
% enlarge cruising time, such that area below velocity profile equals dp again
t(4) = t(4) + (dp - dp_new) / v3_new;
return


function [t,j,type] = findProfileDoubleDec (t,j, type)
return

function [t,j,type] = findProfileNormal (t,j, a0,v0,p0, ptarget, type)
if (nargin < 7) % initial call, type not yet known, simply find initial profile
    if (t(2) ~= 0) % T? profile
        if (t(6) ~= 0) 
            type = 'TT' % TT profile
        else
            type = 'TW' % TW profile
        end
    else
        if (t(6) ~= 0) 
            type = 'WT' % WT profile
        else
            type = 'WW' % WW profile
        end
    end
end
    
disp ('profile: %s', type);

t_orig = t;
if (strcmp (type, 'TT')) 
    % cut out smaller a=const. part
    dt = min(t(2), t(6));
    t(2) = t(2) - dt;
    t(6) = t(6) - dt;
    if (stillOvershoots(t,j,a0,v0,p0,ptarget))
        % recursively calling this function even cuts further
        [t,j,type] = findProfileNormal (t,j, a0,v0,p0, ptarget, type);
    else
        % now we stop before the target, hence profile stays TT
        t = t_orig; % return input profile
    return
end

if (strcmp (type, 'WW'))
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

        if (stillOvershoots(t,j,a0,v0,p0,ptarget))
            type = 'WW'; % type switches to WW
        else
            % now we stop before the target, hence profile stays WT
            t = t_orig; % return input profile
        end
    else
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

        if (stillOvershoots(t,j,a0,v0,p0,ptarget))
            type = 'WW'; % type switches to WW
        else
            % now we stop before the target, hence profile stays TW
            t = t_orig; % return input profile
        end
    else
        % nothing to cut out, stays at TW
    end
    return
end
return

function bOverShoot = stillOvershoots(t,j,a0,v0,p0,ptarget)
    [a_end, v_end, p_end] = calcjTracks(t,j, a0,v0,p0);
    bOverShoot = (sign(p_end - ptarget)*dir == 1);
return
