function [ nx, nw ] = resample_particles(n, x, w)
    %RESAMPLE_PARTICLES Resamples n particles from 
    % the previous particles x with weights w
    L = cumsum(w);

    nx = zeros([size(x, 1) n]);
    for i = 1:n
        nx(:, i) = x(:, find(rand <= L,1));
    end
    
    nw = 1 / n .* ones([1, n]);
end
