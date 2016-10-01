function [ err ] = measure_patch_error(patchA, patchB)
    if (size(patchA) ~= size(patchB))
        err = 1;
        return;
    else
        a = patchA(:);
        b = patchB(:);
        
        diff = double(a - b) / 255;
        
        err = sum(diff.^2) / length(diff);
        
        %err = 1 - err * err;
end

