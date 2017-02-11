function [tracker, detector, frames] = setup_hard_seq(skip_rate)
    frames = ImagesSource('../data/kumar/c2/images', skip_rate);

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
    mmParams(1).err_discard_threshold = 2; % Discard only if out of screen
    mmParams(1).num_particles = 3000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.04 0.04 0.04 ...
                                 0.005 0.005 0.005 ...
                                 0 0 0];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 14;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/kumar/tags.txt');
     
    % Self-minimized:
    initial = [-0.022422; -0.2402; 1.2511;
                0.99989; -0.012514; 0.00029752; -0.0073524; 
                0; 0; 0; 0; 0; 0];
    
    
    % From dataset:
    % initial = [-0.049631; -0.25185; 1.244; 0.99997; -0.0049944; 0.0013402; -0.0054943;
    %               0; 0; 0; 0; 0; 0]


    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end