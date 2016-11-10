function [ M ] = algorithm(tracker, detector, images, record)
    M = [];
    
    disp('Initializing figures');
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    
    figure(1);
    
    disp('Entering main loop');
    
    while images.hasImage()
        fprintf('------------------------------\n');      
        fprintf(':: Reading image\n'); 
        tic();
        img = images.readImage();
        fprintf('// Took %f\n', toc());
	
        fprintf(':: Processing image'); 
        detector_tags = detector.process(img);
        tags = tracker.process(img, detector_tags);
                
        fprintf(':: Clearing figures\n');
        tic();

        %{
        clf(fig1);
        clf(fig2);
        clf(fig3);
        clf(fig4);
        %}
        
        fprintf('// Took %f\n', toc());
        fprintf(':: Displaying result\n');
        tic();

        figure(1);
        colormap(gray(255));
        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(tracker.params.K, tags);
        
        drawTags(tags);
        drawnow;
        
        figure(2);
        
        fprintf('// Took %f\n', toc());
        fprintf(':: Displaying debug stuff\n');
        tic();

        drawnow;
        tracker.debug(fig2, fig3, fig4);
        figure(1);

        fprintf('// Took %f\n', toc());

        
        
        if record
            tic();
            fprintf('Getting frame');

            M = [ M getframe(fig1) ];

            disp('-- '), disp(toc());
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
