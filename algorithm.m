function [ imgs, corners, pos, rot ] = algorithm(tracker, detector, images, record)
    imgs = [];
    corners = [];
    pos = [];
    rot = [];
    
    disp('Initializing figures');
    
    fig1 = sfigure(1);
    fig2 = sfigure(2);
    fig3 = sfigure(3);
    fig4 = sfigure(4);
    fig5 = sfigure(5);
    
    figure(1);
    
    disp('Entering main loop');
    
    counter = 0;
    
    while images.hasImage()
        
        counter = counter + 1;
        if mod(counter, 10) == 0
            clc;
        end
        
        fprintf('------------------------------\n');      
        fprintf(':: Reading image\n'); 
        tic();
        img = images.readImage();
        fprintf('// Took %f\n', toc());
	
        fprintf(':: Processing image'); 
        [detector_tags] = detector.process(img);
        tags = tracker.process(img, detector_tags);
                
        fprintf(':: Clearing figures\n');
        tic();

        clf(fig1);
        clf(fig2);
        clf(fig3);
        clf(fig4);
        clf(fig5);
        
        fprintf('// Took %f\n', toc());
        fprintf(':: Displaying result\n');
        tic();

        sfigure(1);
        colormap(gray(255));
        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(tracker.tagParams.K, tags);
        
        pos = [pos; tags{1}.state'];
        corners = [corners; reshape(tags{1}.corners', [1, 8])];
        
        %drawTags(detector_tags, 'symbol', 'x');
        %drawTags(reproj_tags, 'symbol', 'o');
        drawTags(tags);
        
        sfigure(2);
        
        fprintf('// Took %f\n', toc());
        fprintf(':: Displaying debug stuff\n');
        tic();

        tracker.debug(fig2, fig3, fig4, fig5);
        sfigure(1);

        fprintf('// Took %f\n', toc());
        
        drawnow;
        if record
            tic();
            fprintf('Getting frame');

            imgs = [ imgs getframe(fig1) ];

            disp('-- '), disp(toc());
        end
    end
end

function drawTags(tags, varargin)
    for i=1:length(tags)
        drawTag(tags{i}, varargin);
    end
end

% Draws a (projected!) tag
function drawTag(tag, vars)
    symbol = '.-';
    for i=1:length(vars)
        if strcmp(vars{i},'symbol')
            symbol = vars{i + 1};
        end
    end
    
    if max(size(tag)) < 1
        return
    end
    color = tag.color;
    points = tag.corners';
    x = points(1, :);
    y = points(2, :);
    
    plot([x x(1)], [y y(1)], symbol, 'Color', color);
end
