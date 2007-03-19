% t = (t1 t2 ... tn+1) ... start time, times for the n phases
% j = (j1 j2 ... jn) ... jerks for the n phases
function [a,v,p] = calcjTracks(t,j,a0,v0,p0)

a=a0;
v=v0;
p=p0;

for i=1:length(t)-1,
    if (t(i+1) > t(i))
        [a,v,p] = calcjTrack(t(i),t(i+1),j(i),a,v,p);
    end
end