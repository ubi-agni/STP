% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases
% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases
function [a,v,p] = plot7Track(t,j,a0,v0,p0)

a=a0;
v=v0;
p=p0;

for i=1:length(t)-1,
    if (t(i+1) > t(i)+0.00001)
        [a,v,p] = plotjTrack(t(i),t(i+1),j(i),a,v,p);
    end
end

axis auto;