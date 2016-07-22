function QN = qnorm(Q)
%
% QN = qnorm(Q)
%
% calculates norm of q
%% ------ input -------
% Q      n x 4 array of quaternions [w, x, y, z]
%% ------ output ------
% QN      n x 1 vector of lengths of q
%
    QN = sqrt(sum((Q.*Q)')');
end

