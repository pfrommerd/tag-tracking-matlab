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
        printf('------------------------------\n');      
        printf(':: Reading image\n'); fflush(stdout);
        tic();
        img = images.readImage();
        printf('// Took %f\n', toc());
	
        disp(':: Processing image'); fflush(stdout);
        tags = tracker.process(img);
                
        printf(':: Clearing figures\n'); fflush(stdout);
        tic();

        clf(fig1);
        clf(fig2);
        clf(fig3);
        clf(fig4);
        
        printf('// Took %f\n', toc());
        printf(':: Displaying result\n'); fflush(stdout);
        tic();

        figure(1);
        colormap(gray(255));
        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(tracker.params.K, tags);
        
        drawTags(tags);
        
        figure(2);
        
        printf('// Took %f\n', toc());
        printf(':: Displaying debug stuff\n'); fflush(stdout);
        tic();

        tracker.debug(fig2, fig3, fig4);
        figure(1);

        printf('// Took %f\n', toc()); fflush(stdout);

        if record
            tic();
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
end
