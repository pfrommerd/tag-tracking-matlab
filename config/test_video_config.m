function [tracker, detector, frames] = test_video_config(initial_skip, skip_rate)
    frames = ImagesSource('../data/test/images', initial_skip, skip_rate);

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
    mmParams(1).num_particles = 6000;
    mmParams(1).process_noise = [0.005 0.005 0.005 ...
                                 0.03 0.03 0.03 ...
                                 0.01 0.01 0.01 ...
                                 0.03 0.03 0.03];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @transform_tag);

    model.loadTags('../data/test/tags.txt');
    
    initial = [0.1862224; -0.0067017; 0.696942; ...
               0.991367; -0.124779; -0.014736; 0.037472;...
               0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end