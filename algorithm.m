function [ M ] = algorithm(K, images, record)
    M = [];

    tagSize = [0.1635 0.1635];
    patchTagSize = [0.1835 0.1835];
    patchSize = [64 64];
    
    params(1).tagSize = tagSize;
    params(1).patchTagSize = patchTagSize;
    params(1).patchSize = patchSize;
    params(1).K = K;
    tagSource = AprilTrack(params);

    % Motion model stuff
    function state = transform(state, x)
        state(1:3) = state(1:3) + x(1:3);
        state(4:7) = x(4:7);
    end
    function x = invtransform(state, transState)
        x = state;
        x(1:3) = transState(1:3) - state(1:3);
        x(4:7) = state(4:7);
    end
    
    mmParams(1).num_particles = 1000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.01 0.01 0.01 ...
                                 0 0 0 ...
                                 0 0 0];
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;
    mmParams(1).lambda = 8;
    
    
    model = MotionModel(mmParams, @transform, @invtransform);
    % Add the tags
    t(1).id = 0;
    t(1).color = 'r';
    t(1).state = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    
    model.addTag(t);
    model.setTagWeight(0, 1);
    
    tagSource.addMotionModel(model);
    
    
    fig1 = figure(1);
    fig2 = figure(2);
    fig3 = figure(3);
    fig4 = figure(4);
    fig5 = figure(5);
    
    figure(1);
    
    while images.hasImage()
        img = images.readImage();

        set(0, 'CurrentFigure', fig2)
        clf(fig2);

        tags = tagSource.process(img);

        set(0, 'CurrentFigure', fig1)
        clf(fig1);
        colormap(fig1, gray(256));
        caxis([0,1])

        image(img);
        hold on;
        
        % project the tags, will be stored in the
        % tags array
        tags = project_tags(K, tagSize, tags);
        
        drawTags(tags);
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

% Draws a (projected!) tag (the corner points, in column vector form)
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