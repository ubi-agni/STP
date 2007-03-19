% t = (t1 t2 t3 t4) ... start time, times for the 3 phases
% a = (a1 a2 a3) ... acc for the 3 phases
function [v,p] = plotaTracks(t,a,v0,p0)
v = v0;
p = p0;
for i=1:length(t)-1,
    if (t(i+1) > t(i))
        [v,p] = plotaTrack(t(i),t(i+1),a(i),v,p);
    end
end

axis auto;