function [ err ] = measure_patch_error(patchA, patchB)
    if (size(patchA) ~= size(patchB) || ...
        (size(patchA,1) == 0 || size(patchA,2) == 0) ||
        (size(patchA,1) == 0 || size(patchA,2) == 0) )
        err = 1;
        return;
    else
        a = patchA(:);
        b = patchB(:);
        
        diff = double(a - b) / 255;
        
        err = sum(diff.^2) / length(diff);
    end
end

