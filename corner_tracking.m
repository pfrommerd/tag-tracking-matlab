function [ M ] = corner_tracking(K, images)
    tagSize = 0.1635;

    tagSource = FusedTagSource(tagSize, K, 4);

    M = [];
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    figure(1);
    
    while images.hasImage()
        img = images.readImage();
        
        set(0, 'CurrentFigure', fig1)

        tags = tagSource.process(img);

        clf(fig1);
        imshow(img);
        hold on;

        drawTags(tags);
        drawnow
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
    
    plot(x(1), y(1), 'x', 'Color', color);
    plot(x(2), y(2), 'o', 'Color', color);
    plot(x(3), y(3), '*', 'Color', color);
    plot(x(4:end), y(4:end), 's', 'Color', color);
end