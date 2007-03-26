function [t_res,j_res] = calc7st_nocruise(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0)

% The function must be called with a valid 7-phases profile, which is
% overshooting the target. It will then first do a case distinction
% to check, which kind of profile we currently have.
% Next step is to cut out / shift parts of the acc-profile to shorten the
% profile until it is not overshooting the target anymore. At this time we
% know which profile-type the final solution will have and can call the
% appropriate function to find the final solution.
    % (1) Check whether we have a double deceleration profile.
    % (2) Case distinction: TT / TW / WT / WW
    
plotjTracks(t, j, ptarget, jmax, amax, vmax, a0, v0, p0, true); set(gcf, 'Name', 'zero-cruise'); figure;

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
            plotjTracks(t_new, j, ptarget, jmax, amax, vmax, a0, v0, p0, true); set(gcf, 'Name', 'W-T-Grenzfall'); figure;
            % compute reached position
            [a_end,v_end,p_end] =  calcjTracks(t_new, j, a0, v0, p0);
            % if we still overshoot, the profile becomes trapezoidal
            if (sign(p_end - ptarget)*dir == 1)
                bSecondTrapezoidal = true;
                t = t_new;
            end
        else
            % velocity delta in phase 3 is not enough to extend
            % wedge-shaped second decleration phase to trapezoidal shape
            % so we stay at a triangular profile
        end
    end
    % compute last part of profile, keeping first one fixed
    % in order to keep first part fixed we add further equations
    if (t(6) == 0 && ~bSecondTrapezoidal) % second part will be wedge-shaped (W)
        [t_res, j_res] = calc7st_nocruise_decW (t,j,dir, ptarget, jmax,amax,vmax, a0,v0,p0);
    else % second part will be trapezoidal (T)
        t(6)=1; % allow trapez in second part
        [t_res, j_res] = calc7st_nocruise_decT (t,j,dir, ptarget, jmax,amax,vmax, a2,v2,p2);
    end
    return;
end

%h = figure; set(h, 'Name', 'Before cut out'); plotjTracks(t, j, ptarget, jmax, amax, vmax, a0, v0, p0, true);

% we don't have double deceleration --> cut out instead of merging
% (2)
% case distinction
if (~isZero(t(2)) && ~isZero(t(6)))
    % TT profile
    disp(sprintf('[cutter] TT-Profile...'));
    % cut out smaller a=const. part
    dt = min(t(2), t(6));
    t(2) = t(2) - dt;
    t(6) = t(6) - dt;
    % test whether we still overshoot
    [a_end,v_end,p_end] =  calcjTracks(t, j, a0, v0, p0);
   % h = figure; set(h, 'Name', 'Cutted out'); plotjTracks(t, j, ptarget, jmax, amax, vmax, a0, v0, p0, true);
    if (sign(p_end - ptarget)*dir == -1)
        % Success! We stop before the target now!
        % Call the appropriate function to solve:
        t(2) = t(2) + dt;
        t(6) = t(6) + dt;
        [t_res, j_res] = calc7st_nocruise_tt(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        return;
    else
        % The resulting profile is still stopping behind the target.
        % We recursively call this function to do some further cutting:
        disp(sprintf('[cutter] recursive call...'));
        [t_res, j_res] = calc7st_nocruise(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0);
    end
    
elseif (isZero(t(2)) && isZero(t(6)))
    % WW profile
    % if we cut out anymore, we will reach the full stop profile. However,
    % we already know, that the full stop profile is stopping before the
    % target. So we know we have the WW case for sure!
    disp(sprintf('[cutter] WW-Profile...'));
    [t_res, j_res] = calc7st_nocruise_ww(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
    return;
    
elseif (isZero(t(2)))
    % WT profile
    disp(sprintf('[cutter] WT-Profile...'));
    a1 = a0 + j(1)*t(1);
    dt_w = min(t(1),t(3));
    area_w_max = abs(dt_w * (2*a1 - dt_w*j(1)));
    area_t_max = t(6)*amax;
    if (area_w_max > area_t_max)
        % we will cut out the whole t(6)
        t(6) = 0;
        dt = (abs(a1)-sqrt(a1^2-area_t_max))/jmax;
        t(1) = t(1)-dt;
        t(3) = t(3)-dt;
        % test whether we still overshoot
        [a_end,v_end,p_end] =  calcjTracks(t,j, a0, v0, p0);
        if (sign(p_end - ptarget)*dir == -1)
            % Success! We stop before the target now!
            % Call the appropriate function to solve:
            [t_res, j_res] = calc7st_nocruise_wt(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        else
            % The resulting profile is still stopping behind the target.
            % So it must be the WW-profile:
            disp(sprintf('[cutter] WW-Profile...'));
            [t_res, j_res] = calc7st_nocruise_ww(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        end
    else
        % not: (area_w_max > area_t_max)
        % so we would have to cut out the wedge completely - which would
        % lead to a full stop profile --> we have the td case
        [t_res, j_res] = calc7st_nocruise_wt(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        return;
    end
    
else
    % TW profile
    disp(sprintf('[cutter] TW-Profile...'));
    t
    j
    a5 = j(5)*t(5);
    area_w_max = abs(t(5)*a5);
    area_t_max = t(2)*amax;
    if (area_w_max > area_t_max)
        % we will cut out the whole t(2)
        t(2) = 0;
        t(5) = sqrt((area_w_max-area_t_max)/abs(j(5)));
        t(7) = t(5);
        %h = figure; set(h, 'Name', 'After cut out'); plotjTracks(t, j, ptarget, jmax, amax, vmax, a0, v0, p0, true);
        % test whether we still overshoot
        [a_end,v_end,p_end] =  calcjTracks(t,j, a0, v0, p0);
        if (sign(p_end - ptarget)*dir == -1)
            % Success! We stop before the target now!
            % Call the appropriate function to solve:
            disp(sprintf('[cutter] TW-Profile1...'));
            [t_res, j_res] = calc7st_nocruise_tw(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        else
            % The resulting profile is still stopping behind the target.
            % So it must be the WW-profile:
            disp(sprintf('[cutter] WW-Profile...'));
            [t_res, j_res] = calc7st_nocruise_ww(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        end
    else
        % not: (area_w_max > area_t_max)
        % so we would have to cut out the wedge completely - which would
        % lead to a full stop profile --> we have the td case
        disp(sprintf('[cutter] TW-Profile2...'));
        [t_res, j_res] = calc7st_nocruise_tw(t,j,dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        return;
    end
end
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% solve normal profiles                                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [t_res,j_res] = calc7st_nocruise_tt(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = split35 (stp7_solve ([A V TEQ], P, TVARS, sym('t6', 'positive')), a0, dir*jmax);
j_res = j;
return

function [t_res,j_res] = calc7st_nocruise_tw(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = split35 (stp7_solve ([A V TEQ], P, TVARS, sym('t7', 'positive')), a0, dir*jmax);
j_res = j;
return

function [t_res,j_res] = calc7st_nocruise_wt(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = split35 (stp7_solve ([A V TEQ], P, TVARS, sym('t1', 'positive')), a0, dir*jmax);
j_res = j;
return

function [t_res,j_res] = calc7st_nocruise_ww(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = split35 (stp7_solve ([A V TEQ], P, TVARS, sym('t7', 'positive')), a0, dir*jmax);
j_res = j;
return

function t = split35 (t, a0, j)
% Split phases 3 and 5 from each other, according to their zero-crossing
a1 = a0+j*t(1); % == a2
t3 = abs(a1/j);
t(5) = t(3) - t3;
t(3) = t3;
return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% these functions are used for double deceleration profiles %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [t_res,j_res] = calc7st_nocruise_decW(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = stp7_solve ([A V TEQ], P, TVARS, sym('t5', 'positive'));
j_res = j;
return

function [t_res,j_res] = calc7st_nocruise_decT(t,j, dir,ptarget,jmax,amax,vmax, a0,v0,p0)
% Calculate exact phase duration from given profile t, j
% generate set of equations
[A, V, P, TEQ, TVARS, VARS] = stp7_formulas(t, j, false, dir,ptarget,jmax,amax,vmax, a0,v0,p0);
t_res = stp7_solve ([A V TEQ], P, TVARS, sym('t5', 'positive'));
j_res = j;
return