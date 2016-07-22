function QI = qinterpolate(Q1, Q2, H)
%
% QI = qinterpolate(Q1, Q2, H)
%
% quaternion SLERP interpolation:
% QI = Q1 * (Q1^-1 * Q2)^h
%% ------ input -------
% Q1      n x 4 array of quaternions [w, x, y, z]
% Q2      n x 4 array of quaternions [w, x, y, z]
% H       interpolation weights
%% ------ output ------
% QI      n x 4 array of interpolated quaternions
%
    n             = size(Q1, 1);
    q1r = qregularize(Q1);
    q2r = qregularize(Q2);
    QI            = qmult(q1r, qpow(qmult(qinv(q1r), q2r), H));
end