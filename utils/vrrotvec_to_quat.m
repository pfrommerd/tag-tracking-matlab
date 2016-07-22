function Q = vrrotvec_to_quat(OMEGA)
%
% Q = rotvec_to_quat(OMEGA)
%
% converts rotation around phi to quaternion
%% ------ input -------
% OMEGA  n x 4 array of rotation vectors
%% ------ output ------
% Q      n x 4 array of quaternions [w, x, y, z]
%
    n     = size(OMEGA, 1);  % number of vectors to convert
    Q     = [ones(n, 1), zeros(n, 3)];
    ang   = OMEGA(:, 4); % length of angle of rotation
    ahalf = ang * 0.5; % half of angle of rotation
    idx   = find(abs(ang) > 10 * eps);
    if (length(idx) > 0)
        Q(idx, :) = [cos(ahalf(idx)), bsxfun(@times, OMEGA(idx,1:3), ...
                                             sin(ahalf(idx)))];
    end
end
