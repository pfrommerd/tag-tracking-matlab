function P = qpow(Q, H)
%
% P = qpow(Q, H)
%
% computes power of unit quaternion: P = Q^H
%
    n     = size(Q, 1);
    theta = acos(Q(:,1));
    P     = [ones(n,1), zeros(n, 3)];
    idx   = find(abs(theta) > 10 * eps);
    P(idx,:) = [cos(theta(idx).*H(idx)), ...
                bsxfun(@times, bsxfun(@rdivide, Q(idx,2:4), sin(theta(idx))), ...
                       sin(theta(idx)).*H(idx))];
end

