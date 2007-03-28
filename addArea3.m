function [t_res] = addArea3(t,deltaV,jmax,amax)
% Takes a vector of three time intervalls and then
% adds the passed deltaV to the area under the acceleration graph.

% We only increase the area...
deltaV = abs(deltaV);

if (isZero(t(2)))
    % source wedge shaped
    A_now = t(1)^2*jmax;
    Aw_max = amax^2/jmax;
    if (Aw_max < A_now + deltaV)
        % result trapezoid shaped
        t_res(1) = amax/jmax;
        t_res(3) = t_res(1);
        t_res(2) = (A_now + deltaV - Aw_max)/amax;
    else
        % result wedge shaped
        t_res(1) = sqrt(t(1)^2 + deltaV/jmax);
        t_res(3) = t_res(1);
        t_res(2) = 0;
    end
else
    % source trapezoid shaped
    t_res = t;
    t_res(2) = t_res(2) + deltaV/amax;
end