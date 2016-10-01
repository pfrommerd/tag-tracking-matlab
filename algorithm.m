function [ M ] = algorithm(tracker, images, record)
    M = [];
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    %fig5 = figure(5);
    
    figure(1);
    
    while images.hasImage()
        img = images.readImage();
        tags = tracker.process(img);
        set(0, 'CurrentFigure', fig1)
        clf(fig1);
        colormap(fig1, gray(256));
        caxis([0,1])

        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(tracker.params.K, tags);
        
        drawTags(tags);
        
        set(0, 'CurrentFigure', fig2);
        tracker.debug(fig2, fig3, fig4);
        
        drawnow;        
        %break;
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

% Draws a (projected!) tag
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