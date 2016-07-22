function QI = qinv(Q)
%
% QI = qinv(Q)
%
% computes inverse of quaternion [w, x, y, z]
%
%% ------ input -------
% Q      n x 4 array of quaternions [w, x, y, z]
%% ------ output -------
% QI     n x 4 array of inverse of quaternions

    qnorm = sqrt(sum((Q.*Q)'))'; % norm of quats
    QI = bsxfun(@rdivide, [Q(:,1),-Q(:, 2),-Q(:,3),-Q(:,4)], qnorm);
end