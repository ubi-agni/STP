function [t_res, j_res] = shrink_t_j(t, j)

% find first time > 0
start_index = 1;
while ((t(start_index) < 0) || (isZero(t(start_index))))
    start_index = start_index+1;
end

% remove t- and j-doubles
t_res(1) = t(start_index);
j_res(1) = j(start_index);
indx = 1;
for ii=start_index+1:length(t)
	if (isZero(j(ii-1)-j(ii)))
		t_res(indx) = t(ii);
	elseif (~isZero(t(ii-1)-t(ii)))
		indx = indx+1;
		t_res(indx) = t(ii);
		j_res(indx) = j(ii);
	end
end
