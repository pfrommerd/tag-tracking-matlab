function [tracker, detector, frames] = kumar_c2_config(initial_skip, skip_rate)
    frames = ImagesSource('../data/kumar/c2/images', initial_skip, skip_rate);

    % Image parameters
    K = [568.8885   0           959.1503;
         0          784.9941    539.7587;
         0          0           1.0000];

    % Setup some detector stuff
    detector = TagDetector(K);
    trackParams(1).K = K;
    
    % Patch size
    trackParams(1).patchSize = [32 32];
    
    % Create the tracker
    tracker = AprilTrack(trackParams);

    % Add a motion model
    % 0 = don't discard, + = error cutoff, - = discard if out of screen
    mmParams(1).err_discard_threshold = -1; % Discard only if out of screen
    %mmParams(1).num_particles = 6000;
    %mmParams(1).process_noise = [0.01 0.01 0.01 ...
    %                             0.005 0.005 0 ... % pitch, yaw, roll
    %                             0 0 0 ...
    %                             0 0 0];
    mmParams(1).num_particles = 20000;
    mmParams(1).process_noise = [0.02 0.02 0.02 ...
                                 0.01 0.01 0.01 ...
                                 0.005 0.005 0.005 ...
                                 0.001 0.001 0.001];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 8;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/kumar/tags.txt');
     
    % Self-minimized:
    initial = [-0.022422; -0.2402; 1.2511;
                0.99989; -0.012514; 0.00029752; -0.0073524; 
                0; 0; 0; 0; 0; 0];
    % If the initial_skip is not 0, read from the temporary poses
    % to resume tracking
    if initial_skip > 0
        file = sprintf('../tmp/poses/poses_%04d.mat', initial_skip);
        contents = load(file);
        initial = contents.x;
    end
    
    % From dataset:
    % initial = [-0.049631; -0.25185; 1.244; 0.99997; -0.0049944; 0.0013402; -0.0054943;
    %               0; 0; 0; 0; 0; 0]


    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end