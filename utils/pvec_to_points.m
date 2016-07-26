function [ points ] = pvec_to_points( x )
    points = reshape(x, [2, size(x, 1) * size(x, 2) / 2])';
end

