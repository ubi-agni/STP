% t = (t1 t2 ... tn) ... time-intervalls for the n phases
% j = (j1 j2 ... jn) ... jerks for the n phases
function [a,v,p] = calcjTracks(t,j,a0,v0,p0)

a=a0;
v=v0;
p=p0;

for i=1:length(t)
    if (t(i) > 0)
        [a,v,p] = calcjTrack(t(i),j(i),a,v,p);
    end
end