function [ M ] = algorithm(tracker, images, record)
    M = [];
    
    disp('Initializing figures');
    fflush(stdout);
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    
    figure(1);
    
    disp('Entering main loop');
    fflush(stdout);
    
    while images.hasImage()
        disp('------------------------------');        
        disp('Reading image');
        tic();
        fflush(stdout);
        img = images.readImage();
        disp('-- '), disp(toc());
        disp('Processing image');
        fflush(stdout);
        
        tags = tracker.process(img);
                
        disp('-- '), disp(toc());
        disp('Clearing figures');
        fflush(stdout);

        clf(fig1);
        clf(fig2);
        clf(fig3);
        clf(fig4);
        
        disp('-- '), disp(toc());
        disp('Displaying figures');
        fflush(stdout);

        figure(1);
        colormap(gray(255));
        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(tracker.params.K, tags);
        
        drawTags(tags);
        
        figure(2);
        
        disp('-- '), disp(toc());
        disp('Rendering debug output');
        fflush(stdout);
        
        tracker.debug(fig2, fig3, fig4);
        figure(1);

        disp('-- '), disp(toc());
        fflush(stdout);

        if record
            disp('Getting frame');
            fflush(stdout);

            M = [ M getframe(fig1) ];

            disp('-- '), disp(toc());
            fflush(stdout);
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