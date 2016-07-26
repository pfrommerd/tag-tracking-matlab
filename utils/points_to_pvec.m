function [ x ] = points_to_pvec( points, n)
    x = reshape(points', [n * 2, ...
                          size(points, 1) / n]); 
end

