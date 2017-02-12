% Use squared error
%{
function [ err ] = measure_patch_error(patchA, patchB)
    if ((size(patchA, 1) ~= size(patchB,1)) || ...
        (size(patchA, 2) ~= size(patchB,2)) || ...            
        (size(patchA,1) == 0 || size(patchA,2) == 0))
        err = 1;
        return;
    else
        [M, N] = size(patchA);
        diff = double(patchA) - double(patchB);
        %err = sum(sum(diff) .* diff)) / (M * N);        
        % Divide by 255^2 to get the error from 0-1
        err = sum(sum(diff .* diff)) / (M * N * 255 * 255);
        return;
    end
end
%}

% Use correlation
%%{
function [ err ] = measure_patch_error(patchA, patchB, default)
    if (size(patchA) ~= size(patchB))
        err = default;
        return;
    else
        a = double(patchA);
        b = double(patchB);

        correlation = min(max(corr2(a, b), 0.0001), 1);
        err = -log(correlation);

        if isnan(err) % Some crazy value, like -Inf, Inf, NaN
            err = default;
        end
    end
end
%}