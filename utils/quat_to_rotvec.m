function OMEGA = quat_to_rotvec(Q)
%
% OMEGA = quat_to_rotvec(Q)
%
%% ------ input -------
% Q      n x 4 array of quaternions [w, x, y, z]
%% ------ output ------
% OMEGA  n x 3 array of rotation vectors
%
    n             = size(Q, 1);
    ahalf         = acos(Q(:,1));
    ang           = sin(ahalf);
    OMEGA         = zeros(n, 3);
    idx           = find(ang > 10 * eps);
    
    if length(idx) > 0
        OMEGA(idx, :) = ahalf * 2 * Q(idx, 2:4) / ang(idx);
    end
end

