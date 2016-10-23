function [ nx, nw ] = resample_particles(n, x, w)
    %RESAMPLE_PARTICLES Resamples n particles from 
    % the previous particles x with weights w
    L = cumsum(w);
        
    fflush(stdout);

    nx = zeros([size(x, 1) n]);
    for i = 1:n
        idx = find(rand <= L, 1);
        nx(:, i) = x(:, idx);
    end
    
    nw = 1 / n .* ones([1, n]);

    sleep(1);
    
    fflush(stdout);
end
