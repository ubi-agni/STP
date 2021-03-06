% t = (t1 t2 ... tn+1) ... start time, times for the n phases
% j = (j1 j2 ... jn) ... jerks for the n phases
function [isCorrect, reason] = testjTracks(t,j,a0,v0,p0,jmax,amax,vmax,ptarget)

if (numel(t) == 7) 
    output = 'Profile: ';
    if (j(1) == j(5)) output = [output, 'ddec ']; else output = [output, 'can ']; end
    if (t(2) == 0) output = [output, 'W']; else output = [output, 'T']; end
    if (t(4) ~= 0) output = [output, 'c']; end
    if (t(6) == 0) output = [output, 'W']; else output = [output, 'T']; end
    disp(output);
else
    disp('Expected 7 elements in t vector...');
    t
end

derr = 0.000001;

a=a0;
v=v0;
p=p0;

for ii=1:length(t),
	if (~isreal(t(ii)) || (t(ii) < 0))
		isCorrect = false;
		reason = sprintf('Negative or unreal time intervalls!! (t(%d) = %f)', ii, t(ii));
		return;
	end
    if (t(ii) > 0)
        [a,v,p] = calcjTrack(t(ii),j(ii),a,v,p);
		if (abs(j(ii))-jmax > derr)
			isCorrect = false;
			reason = sprintf('j is too high!! (j = %f but jmax = %f)', j(ii), jmax);
			return;
		end
		if (abs(a)-amax > derr)
			isCorrect = false;
			reason = sprintf('a is too high!! (a = %f but amax = %f)', a, amax);
			return;
		end
		if ((abs(v)-vmax > derr) && ((ii>1) || (abs(a0) <= amax)))
			[a2 v2 p2] = calcjTrack(t(ii+1),j(ii+1),a,v,p);
			if (~isZero(abs(v2)-vmax) && (a*sign(v) > 0))
				isCorrect = false;
				reason = sprintf('v is too high!! (v = %f but vmax = %f)', v, vmax);
				return;
			end
		end
    end
end

if (abs(p-ptarget) > derr)
	isCorrect = false;
	reason = sprintf('Didn''t reach the target!! (ptarget = %f but p = %f)', ptarget, p);
	return;
end
if (abs(a) > derr)
	isCorrect = false;
	reason = sprintf('Didn''t finish with a = 0!! (a = %f)', a);
	return;
end
if (abs(v) > derr)
	isCorrect = false;
	reason = sprintf('Didn''t finish with v = 0!! (v = %f)', v);
	return;
end

isCorrect = true;
reason = '';