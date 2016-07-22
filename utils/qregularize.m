function P = qregularize(Q)
%
% P = qregularize(Q)
%
% make vector of quaternions regular such that there is minimum
% distance between each quaternion.
%% ------ input -------
% Q       n x 4 array of quaternions [w, x, y, z]
%% ------ output ------
% P       n x 4 array of quaternions [w, x, y, z]
    P = Q;
    for i = 2:size(Q,1)
        if qdist(Q(i,:),P(i-1,:)) > qdist(-Q(i,:),P(i-1))
            P(i,:) = -Q(i,:);
        end
    end
end

