function [t_res,j_res] = calc7st_nocruise_cutter(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0)

% The function must be called with a valid 7-phases profile, which is
% overshooting the target. It will then first first do a case distinction
% to check, which kind of profile we currently have.
% Next step is to cut out / merge parts of the acc-profile to reduce the
% profile until it is not overshooting the target anymore. At this time we
% know which profile-type the final solution will have and can call the
% appropriate function to get it.
    % (1) Check whether we have a double deceleration profile.
    
    % (2) Case distinction: TT / TW / WT / WW
    
% (1)
% check if we have a double deceleration profile
[a2, v2, p2] = calcjTracks(t(1:2),j(1:2), a0, v0, p0);
[a6, v6, p6] = calcjTracks(t(1:6),j(1:6), a0, v0, p0);
if (sign(a2) == sign(a6))
    bDoubleDec = true
    disp(sprintf('We can''t handle this case yet!'));
    t_res = zeros(1,7);
    j_res = zeros(1,7);
    return;
else
    bDoubleDec = false    
end

%h = figure; set(h, 'Name', 'Before cut out'); plotjTracksNice(t, j, ptarget, jmax, amax, vmax, a0, v0, p0);

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
   % h = figure; set(h, 'Name', 'Cutted out'); plotjTracksNice(t, j, ptarget, jmax, amax, vmax, a0, v0, p0); figure;
    if (sign(p_end - ptarget)*dir == -1)
        % Success! We stop before the target now!
        % Call the appropriate function to solve:
        t(2) = t(2) + dt;
        t(6) = t(6) + dt;
        [t_res, j_res] = calc7st_nocruise_tt(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0);
        return;
    else
        % The resulting profile is still stopping behind the target.
        % We recursively call this function to do some further cutting:
        disp(sprintf('[cutter] recursive call...'));
        [t_res, j_res] = calc7st_nocruise_cutter(t,j,dir,dp,ptarget,jmax,amax,vmax,a0,v0,p0)
    end
    
elseif (isZero(t(2)) && isZero(t(6)))
    % WW profile
    % if we cut out anymore, we will reach the full stop profile. However,
    % we already know, that the full stop profile is stopping before the
    % target. So we know we have the WW case for sure!
    disp(sprintf('[cutter] WW-Profile...'));
    [t_res, j_res] = calc7st_nocruise_ww(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
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
            [t_res, j_res] = calc7st_nocruise_wt(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        else
            % The resulting profile is still stopping behind the target.
            % So it must be the WW-profile:
            disp(sprintf('[cutter] WW-Profile...'));
            [t_res, j_res] = calc7st_nocruise_ww(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        end
    else
        % not: (area_w_max > area_t_max)
        % so we would have to cut out the wedge completely - which would
        % lead to a full stop profile --> we have the td case
        [t_res, j_res] = calc7st_nocruise_wt(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
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
        %h = figure; set(h, 'Name', 'After cut out'); plotjTracksNice(t, j, ptarget, jmax, amax, vmax, a0, v0, p0); figure;
        % test whether we still overshoot
        [a_end,v_end,p_end] =  calcjTracks(t,j, a0, v0, p0);
        if (sign(p_end - ptarget)*dir == -1)
            % Success! We stop before the target now!
            % Call the appropriate function to solve:
            disp(sprintf('[cutter] TW-Profile1...'));
            [t_res, j_res] = calc7st_nocruise_tw(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        else
            % The resulting profile is still stopping behind the target.
            % So it must be the WW-profile:
            disp(sprintf('[cutter] WW-Profile...'));
            [t_res, j_res] = calc7st_nocruise_ww(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
            return;
        end
    else
        % not: (area_w_max > area_t_max)
        % so we would have to cut out the wedge completely - which would
        % lead to a full stop profile --> we have the td case
        disp(sprintf('[cutter] TW-Profile2...'));
        [t_res, j_res] = calc7st_nocruise_tw(dir,ptarget,jmax,amax,vmax,a0,v0,p0);
        return;
    end
end