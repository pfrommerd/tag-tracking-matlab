function [ M ] = algorithm(K, images, record)
    M = [];

    tagSize = 0.1635;
    patchTagSize = [0.1835 0.1835];
    patchSize = [64 64];
    
    num_particles = 2000;
    lambda = 8;
    k = 1;
    alpha = 1 - k;
    %alpha = 0.3;
    %{
    process_noise = [0.001 0.001 0.001 ...
                     0.001 0.001 0.001 ...
                     0 0 0 ...
                     0 0 0];
    %}

    %{
    process_noise = [0.1 0.1 0.1 ...
                     0.001 0.001 0.001 ...
                     0.002 0.002 0.002 ...
                     0.005 0.005 0.005];
    %}
    % Setup for seq1
    %{
    process_noise = [0.8 0.8 0.8 ...
                     0.3 0.3 0.3 ...
                     0.01 0.01 0.01 ...
                     0.01 0.01 0.01];
    %}
    % Setup for video.mp4
    %%{
    process_noise = [0.01 0.01 0.01 ...
                     0.05 0.05 0.05 ...
                     0.02 0.02 0.02 ...
                     0.01 0.01 0.01];
    %}
    %{
    process_noise = [0.03 0.03 0.03 ...
             0.08 0.08 0.08 ...
             0.0 0.0 0.0 ...
             0.00 0.00 0.00];
    %}  

    
    params(1).tagSize = tagSize;
    params(1).patchTagSize = patchTagSize;
    params(1).patchSize = patchSize;
    params(1).K = K;
    
    params(1).process_noise = process_noise;
    params(1).num_particles = num_particles;
    params(1).k = k;
    params(1).alpha = alpha;
    params(1).lambda = lambda;
                 
    tagSource = AprilTrack(params);
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    fig5 = figure(5);
    
    figure(1);
    
    while images.hasImage()
        det_img = images.readImage();
        img = det_img;
        %img = imgaussfilt(det_img, 10);
        tags = tagSource.process(img, det_img);
        
        set(0, 'CurrentFigure', fig1)

        clf(fig1);
        colormap(fig1, gray(256));
        caxis([0,1])
        image(img);
        hold on;
        drawTags(tags);

        tagSource.debug_plot(fig2, fig3, fig4, fig5, img);
        
        drawnow;        
        
        if record
            M = [ M getframe(fig1) ];
        end
    end
end

function drawTags(tags)
    for i=1:length(tags)
        drawTag(tags{i});
    end
end

% Draws a tag (the corner points, in column vector form)
function drawTag(tag)
    if max(size(tag)) < 1
        return
    end
    color = tag.color;
    points = tag.corners';
    x = points(1, :);
    y = points(2, :);
    
    plot([x x(1)], [y y(1)], '.-', 'Color', color);
    
    %{
    plot(x(1), y(1), 'x', 'Color', color);
    plot(x(2), y(2), 'o', 'Color', color);
    plot(x(3), y(3), '*', 'Color', color);
    plot(x(4:end), y(4:end), 's', 'Color', color);
    %}
end