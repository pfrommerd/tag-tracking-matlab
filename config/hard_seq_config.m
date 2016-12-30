function [tracker, detector, frames] = setup_hard_seq(skip_rate)
    frames = ImagesSource('../data/hard_seq/images', skip_rate);

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
    mmParams(1).err_discard_threshold = 1;
    mmParams(1).num_particles = 5000;
    mmParams(1).process_noise = [0.04 0.04 0.04 ...
                                 0.04 0.04 0.04 ...
                                 0 0 0 ...
                                 0 0 0];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/hard_seq/tags.txt');

    initial = [1.1029; 0.2798; -0.3398;  0.499; 0.499; -0.501; 0.501; 0; 0; 0; 0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end