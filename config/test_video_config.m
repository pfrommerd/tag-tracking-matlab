function [tracker, detector, frames] = test_video_config()
    frames = ImagesSource('../data/test/images');

    % Image parameters
    K = [1002.6   0           540.9;
         0          1007.0    355.4;
         0          0         1.0];

    % Setup some detector stuff
    detector = TagDetector(K);
    trackParams(1).K = K;
    
    % Patch size
    trackParams(1).patchSize = [64 64];
    
    % Create the tracker
    tracker = AprilTrack(trackParams);

    % Add a motion model
    mmParams(1).err_discard_threshold = 0.9;
    mmParams(1).num_particles = 3000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.04 0.04 0.04 ...
                                 0.02 0.02 0.02 ...
                                 0 0 0];

    % noise = 1 / (k + alpha * w) * process_noise * randn
    % Allows for particles of higher/lower weight to have
    % more noise in the propagation step

    % ATM, we resample after every pass, so don't bother...
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 9;

    model = MotionModel(mmParams, @tag_transform);

    model.loadTags('../data/test/tags.txt');
    
    initial = [0.1862224; -0.0067017; 0.696942; ...
               0.991367; -0.124779; -0.014736; 0.037472;...
               0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end