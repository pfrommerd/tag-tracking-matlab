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
    mmParams(1).num_particles = 2000;
    mmParams(1).process_noise = [0.02 0.02 0.02 ...
                                 0.01 0.01 0.01 ...
                                 0 0 0 ...
                                 0 0 0];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/hard_seq/tags.txt');

    %initial = [1.1029; 0.2798; -0.3398;  0.707; 0.707; 0; 0; 0; 0; 0; 0; 0; 0];
    %initial = [1.4845; 0.0898; -1.1796;  0.5676; 0.6329; -0.4496; -0.2735; 0; 0; 0; 0; 0; 0];
    initial = [0; 0.2798; -0.3398;
                1; 0; 0; 0; 
                0; 0; 0; 0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end