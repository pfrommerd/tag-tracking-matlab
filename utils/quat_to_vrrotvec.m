function OMEGA = quat_to_vrrotvec(Q)
%
% OMEGA = quat_to_vrrotvec(Q)
%
%% ------ input -------
% Q      n x 4 array of quaternions [w, x, y, z]
%% ------ output ------
% OMEGA  n x 4 array of rotation vectors, matlab style
%
    n             = size(Q, 1);
    ahalf         = acos(Q(:,1));
    sang           = sin(ahalf);
    OMEGA         = [ones(n, 1), zeros(n, 3)]; % x is non-zero, arbitrarily
    idx           = find(abs(sang) > 10 * eps);
    if length(idx) > 0
        OMEGA(idx, :) = [bsxfun(@rdivide, Q(idx, 2:4), sang(idx)), 2 * ...
                         ahalf(idx)];
    end
end

