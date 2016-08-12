function [ err ] = measure_patch_error(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        err = 1;
        return;
    else
        a = int16(patchA);
        b = int16(patchB);

        err = abs(corr2(a, b));
        err = 1 - err;
        if err < 0 || isnan(err) % Some crazy value, like -Inf
            err = 0;
        end

end

