function D = qdist(P, Q)
%
% D = qdist(P, Q)
%
% distance between quaternions:  D = norm(P-Q)
%
%% ------ input -------
% P      n x 4 array of quaternions [w, x, y, z]
% Q      n x 4 array of quaternions [w, x, y, z]
%% ------ output -------
% D      n x 1 vector of distances

    dq = P-Q;
    D  = sqrt(sum((dq.*dq)')');
end

