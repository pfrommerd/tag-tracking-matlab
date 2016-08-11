function [ err ] = measure_patch_error(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        err = 1;
        return;
    else
        a = int16(patchA);
        b = int16(patchB);

        err = 1 - abs(corr2(a, b));
        if err < 0 || isnan(err) % Some crazy value, like -Inf
            err = 0;
        end

end

