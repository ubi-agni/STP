function [t_res, j_res] = shrink_t_j(t, j)

% remove zero indices
nonzero = find(t ~= 0);
t = t(nonzero);
j = j(nonzero);

% merge identical phases
for i=1:length(t)-1
    if (j(i) == j(i+1))
        % merge
        t(i) = t(i)+t(i+1)
    end
end

% remove zero indices again
nonzero = find(t ~= 0);
t_res = t(nonzero);
j_res = j(nonzero);
