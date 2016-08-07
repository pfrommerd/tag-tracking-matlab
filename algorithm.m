function [ M ] = algorithm(K, images)
    M = [];

    tagSize = 0.1635;

    tagSource = AprilTrack(K, tagSize, [0.1835 0.1935], [128 128]);
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    
    figure(1);
    
    while images.hasImage()
        img = images.readImage();

        tags = tagSource.process(img);
        
        
        set(0, 'CurrentFigure', fig1)

        clf(fig1);
        colormap(fig1, gray(256));
        caxis([0,1])
        image(img);
        hold on;
        drawTags(tags);
        
        tagSource.debug_plot(fig2, fig3, fig4, img);
        
        drawnow;
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