function algorithm(tracker, detector, images, initial_skip, skip_rate, save_images, save_poses)
    disp('Initializing figures');
    
    fig1 = sfigure(1);
    fig2 = sfigure(2);
    fig3 = sfigure(3);
    fig4 = sfigure(4);
    fig5 = sfigure(5);
    fig6 = sfigure(6);
    
    figure(1);
    
    disp('Entering main loop');
    
    counter = initial_skip;
    
    pose_history = [];
    while images.hasImage()
        if mod(counter, 10) == 0
            %clc;
        end
        
        fprintf('------------------------------\n');      
        fprintf(':: Reading image\n'); 
        tic();
        img = images.readImage();
        fprintf('// Took %f\n', toc());
	
        fprintf(':: Processing image'); 
        [detector_tags] = detector.process(img);
        [tags, x] = tracker.process(img, detector_tags);
        x
        
        fprintf(':: Clearing figures\n');
        tic();

        clf(fig1);
        clf(fig2);
        clf(fig3);
        clf(fig4);
        clf(fig5);
        clf(fig6);
        
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
        
        
        %drawTags(detector_tags, 'symbol', 'x');
        %drawTags(reproj_tags, 'symbol', 'o');
        drawTags(tags);
        
        sfigure(2);
        
        fprintf('// Took %f\n', toc());
        fprintf(':: Displaying debug stuff\n');
        tic();

        tracker.debug(img, x, fig2, fig3, fig4, fig5, fig6);
        
        % Draw the pose history
        sfigure(7);
        pose_history = [pose_history x];
        visualize_poses(pose_history);
        
        sfigure(1);

        fprintf('// Took %f\n', toc());
        
        drawnow;
        if ~(counter == initial_skip && initial_skip > 0)
            if save_images 
                img = getframe(fig1);

                file = sprintf('../tmp/images/frame_%04d.png', counter);
                fprintf('Saving image to file %s\n', file);
                imwrite(img.cdata, file);
            end

            if save_poses
                % Save the poses
                file = sprintf('../tmp/poses/poses_%04d.mat', counter);
                fprintf('Saving pose to file %s\n', file);

                save(file, 'x');
            end
        end
        counter = counter + skip_rate;        
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
