function [isPossible, t_res] = remArea3(t,deltaV,jmax,amax)
% Takes a vector of three time intervalls and then
% deletes the passed deltaV to the area under the acceleration graph.

% We only decrease the area...
deltaV = abs(deltaV);
A_now = t(1)^2*jmax+amax*t(2);
if (A_now < deltaV)
    isPossible = false;
    t_res = [0,0,0];
    return;
end
isPossible = true;
Aw_max = amax^2/jmax;
        
if (isZero(t(2)) || (Aw_max > A_now - deltaV))
    % result wedge shaped
    t_res(1) = sqrt(A_now/jmax - deltaV/jmax);
    t_res(3) = t_res(1);
    t_res(2) = 0;
else
    % result trapezoid shaped
    t_res = t;
    t_res(2) = t_res(2) - deltaV/amax;
end