% t = (t1 t2 t3 t4 t5 t6 t7 t8) ... start time, times for the 7 phases

% j = (j1 j2 j3 j4 j5 j6 j7) ... jerks for the 7 phases

function [a,v,p] = plot7Track(t,j,jmax,amax,vmax,ptarget,a0,v0,p0)

[t,j]= shrink_t_j(t,j);
t = [0 t];



clf;
hold on;

tend = t(length(t));

plot([0,tend],[0,0],'k-');

plot([0,tend],[jmax,jmax],'m:');

plot([0,tend],[-jmax,-jmax],'m:');

plot([0,tend],[amax,amax],'b:');

plot([0,tend],[-amax,-amax],'b:');

plot([0,tend],[vmax,vmax],'g:');

plot([0,tend],[-vmax,-vmax],'g:');

%plot([0,tend],[p0,p0],'r:');

%plot([0,tend],[ptarget,ptarget],'r:');



a=a0;

v=v0;

p=p0;



for i=1:length(t)-1,

    if (t(i+1) > t(i))

        if (i<length(t)-1)

            [a,v,p] = plotjTrack(t(i),t(i+1),j(i),j(i+1),a,v,p);

        else

            [a,v,p] = plotjTrack(t(i),t(i+1),j(i),0,a,v,p);

        end

    end

end



axis tight;
hold off;
