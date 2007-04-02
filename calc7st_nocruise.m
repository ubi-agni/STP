function [t_res,j_res] = calc7st_nocruise(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0)

% The function must be called with a valid 7-phases profile, which is
% overshooting the target. It will then first do a case distinction
% to check, which kind of profile we currently have.
% Next step is to cut out / shift parts of the acc-profile to shorten the
% profile until it is not overshooting the target anymore. At this time we
% know which profile-type the final solution will have and can call the
% appropriate function to find the final solution.

% (0) Check whether a normal profile has to switch into double deceleration
% (1) Check whether we have a double deceleration profile.
% (2) Case distinction: TT / TW / WT / WW

bPlotAll=true;
if (bPlotAll) 
    h = figure;
    set(h,'Name','Zero-Cruise-Profile');
    plotjTracks(t,j, a0, v0, p0, true, jmax, amax, vmax, ptarget);
end

% (0)
if (sign(j(3)) == sign(j(5)) && t(3) < t(1))
    tacc = [t(3) t(2) t(3)]; tdec = t(5:7); 
    [dummy deltaAcc dummy] = calcjTracks(tacc, j(1:3), 0,0,0);
    [dummy deltaDec dummy] = calcjTracks(tdec, j(5:7), 0,0,0);
    if (deltaAcc < deltaDec)
        tacc = [0 0 t(1)-t(3)];
        [ok, tdec] = removeArea (tdec, deltaAcc, jmax,amax);
        tn = [tacc 0 tdec];
        jn = [j(3) 0 j(1)  0  j(5:7)];
        % if we still overshoot, the profile becomes double deceleration
        if (stillOvershoots(tn,jn, dir, a0,v0,p0,ptarget))
            t = tn;
            j = jn;
        end
    end
end

% (1)
% check if we have a double deceleration profile
if (sign(j(3)) ~= sign (j(5)))
    bSecondTrapezoidal = false;
    if (t(6) == 0) % second part is currently wedge, may become trapez
        % calculate maximal shift from first to second deceleration phase
        % in order to reach the W-T-border case
        [a2, v2, p2] = calcjTracks(t(1:2),j(1:2), a0, v0, p0);
        [DeltaT, T5, T7] = calc7st_opt_shift(t,j,dir, jmax,amax, a2,v2);
        if (DeltaT < 0) error ('This should not happen!'); end
        if (DeltaT < t(3))
            % adapt profile by shortening t(3)
            t_new = [t(1:2), t(3)-DeltaT, t(4), T5, 0, T7];

if (bPlotAll) 
    h = figure;
    set(h,'Name','Double Deceleration: border case ?W -> ?T');
    plotjTracks(t_new,j, a0, v0, p0, true, jmax, amax, vmax, ptarget);
end
            % if we still overshoot, the profile becomes trapezoidal
            if (stillOvershoots(t_new,j, dir, a0,v0,p0,ptarget))
                bSecondTrapezoidal = true;
                t = t_new;
            end
        else
            % velocity delta in phase 3 is not enough to extend
            % wedge-shaped second decleration phase to trapezoidal shape
            % so we stay at a triangular profile
        end
    end

    if (t(6) == 0 && ~bSecondTrapezoidal) % second part will be wedge-shaped (W)
        disp('double dec: ?-W');
    else % second part will be trapezoidal (T)
        t(6)=1; % allow trapez in second part when generating formulas
        disp('double dec: ?-T');
    end

    % Calculate exact phase duration from given profile t, j
    % generate generic set of equations
    [A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);

    % compute last part of profile, keeping first one fixed
    % in order to keep first part fixed we add further equations
    % t(1) and t(2) are fixed to their current values
    TEQ(1) = sym(strcat('t1=', char(sym(t(1)))));
    TEQ(2) = sym(strcat('t2=', char(sym(t(2)))));
    
    t_res = solveAll ([A V TEQ P], TVARS);
    j_res = j;
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% we don't have double deceleration --> cut out instead of merging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find correct profile by cutting pieces and descending to shorter profiles
[t,j,type] = findProfile(t,j,dir, a0,v0,p0, ptarget, jmax,amax);
disp (sprintf ('profile: %s', type));

% Calculate exact phase duration for choosen profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = solveAll ([A V TEQ P], TVARS);
%t_res = split35 (t_res, a0, j);
j_res = j;
return

function [t,j,type] = findProfile(t,j,dir, a0,v0,p0, ptarget, jmax,amax)
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
    
t_orig = t;
if (strcmp (type, 'TT')) 
    % cut out smaller a=const. part
    dt = min(t(2), t(6));
    t(2) = t(2) - dt;
    t(6) = t(6) - dt;
    if (stillOvershoots(t,j,dir,a0,v0,p0,ptarget))
        % recursively calling this function even cuts further
        [t,j,type] = findProfile (t,j,dir, a0,v0,p0, ptarget, jmax,amax);
    else
        % now we stop before the target, hence profile stays TT
        t = t_orig; % return input profile
    end
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

        if (stillOvershoots(t,j,dir,a0,v0,p0,ptarget))
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

        if (stillOvershoots(t,j,dir,a0,v0,p0,ptarget))
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

function bOverShoot = stillOvershoots(t,j,dir,a0,v0,p0,ptarget)
    [a_end, v_end, p_end] = calcjTracks(t,j, a0,v0,p0);
    bOverShoot = (sign(p_end - ptarget)*dir == 1);
return

function t = split35 (t, a0, j)
% Split phases 3 and 5 from each other, according to their zero-crossing
a1 = a0+j(1)*t(1); % == a2
t3 = -a1/j(3);
t(5) = t(3) - t3;
t(3) = t3;
return
