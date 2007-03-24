function [t,j] = calc7st(p_target,jmax,amax,vmax,a0,v0,p0,plotMe,bOverWrite)

% Calculates the time optimal third-order trajectory to reach the target
% position with the given start conditions and according to the limitations
% for jerk, acc and vel.
% There are two vectors given back: The vector describing the jerk impulses
% and the one describing the time intervalls for these impulses. Both have a
% fixed length of 7 entries, however, several entries in the t-vector might
% be zero - indicating that this paricular phase is not needed in the profil.
% The elements of the jerk vector are the jerk values (NOT: either of the
% three values -1, 0, 1. NOT: Multiplication with the max-jerk gives the
% actual jerk-value.)

% Algorithm:
    % (1) Calculation of the direction flag (direction of potential cruising phase)
    % by comparing the position we reach at an immideate halt to the desired
    % target position.
	
	% (2) Use 'calc3st' to calculate a zero-cruise profile (t4=0,v4=d*vmax) and
    % decide whether we have a profile with or without cruising phase.
	
	% (3) In case of cruising phase, insert it and give back the solution,
    % otherwise reapetingly call 'cutProfile' to find out, which case we have.
    % After finding out, use the appropriate algorithm to calculate that
    % profile.
    
% fill in missing arguments
if (nargin < 9) bOverWrite=true; end
if (nargin < 8) plotMe=false; end

% (1)
% calculate the dir-flag by testing if we over-shoot the target
% at imidiate halt
[t_stop a_stop] = calc3st(0,jmax,amax,a0,v0);
[ah vh p_stop] = calcjTracks(t_stop,a_stop,a0,v0,p0);
% get direction
dir = sign(p_target-p_stop);
p_fullstop = p_stop;

% (2)
% position change just from acc and dec phase:
[t_acc a_acc] = calc3st(dir*vmax,jmax,amax,a0,v0); % acc. part (before cruising)
[t_dec a_dec] = calc3st(0,jmax,amax,0,dir*vmax); % dec. part (after cruising)
% postion change:
t_zeroCruise = [t_acc 0 t_dec];
j_zeroCruise= [a_acc 0 a_dec];
[ah vh p_stop] = calcjTracks(t_zeroCruise,j_zeroCruise, a0, v0, p0);
%h = figure;set(h,'Name','Zero-Cruise-Profile');plotjTracksNice(t_zeroCruise,j_zeroCruise, p_target, jmax, amax, vmax, a0, v0, p0);
% distance we need to go in cruising phase:
p_delta = (p_target-p_stop);
t_delta = p_delta / (dir*vmax);

disp (sprintf ('full stop at: %f  zero-cruise: %f', p_fullstop, p_stop));

% (3)
% case differentiation: Do we have a cruising phase?
if (t_delta >= 0)
	% with cruising phase
    t = [t_acc t_delta t_dec];
    j = [a_acc, 0, a_dec];
    % display graph
    if (plotMe)
	    [a_end, v_end, p_end] = plotjTracksNice(t,j,jmax,amax,vmax,p_target,a0,v0,p0, bOverWrite);
    end
    return;
end

% without cruising phase
[t,j] = calc7st_nocruise_cutter(t_zeroCruise, j_zeroCruise,dir,p_delta,p_target,jmax,amax,vmax,a0,v0,p0);

% display graph
if (plotMe)
	[a_end, v_end, p_end] = plotjTracksNice(t,j,jmax,amax,vmax,p_target,a0,v0,p0, bOverWrite)
end
