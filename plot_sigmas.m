function [  ] = plot_sigmas(Y, Z, g_x, g_y, figure)
    set(0, 'CurrentFigure', figure)    
    clf();
    
    %{
    x = Y(1, :);
    y = Y(2, :);
    %z = Y(3, :);
    w = Z';
    scatter3(x, y, w, 'o');
    %}
    %%{
    w  = reshape(Z', [length(g_y) length(g_x)]);
    surf(g_x, g_y, w);
    %}
end

