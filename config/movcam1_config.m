function [tracker, detector, frames] = movcam1_config(skip_rate)
    frames = ImagesSource('../data/movcam1/images', skip_rate);

    % Image parameters
    K = [1885         0         946.8;
         0            1881.3    540.8;
         0            0         1.0];


    % Setup some detector stuff
    detector = TagDetector(K);
    trackParams(1).K = K;
    
    % Patch size
    trackParams(1).patchSize = [32 32];
    
    % Create the tracker
    tracker = AprilTrack(trackParams);

    % Add a motion model
    mmParams(1).err_discard_threshold = 1.1;
    mmParams(1).num_particles = 5000;
    mmParams(1).process_noise = [0.0 0.0 0.0 ...
                                 0.1 0.1 0.1 ...
                                 0.0 0.0 0.0 ...
                                 0.0 0.0 0.0];
%    mmParams(1).process_noise = [0.02 0.02 0.02 ...
%                                 0.02 0.02 0.02 ...
%                                 0.01 0.01 0.01 ...
%                                 0.01 0.01 0.01];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/movcam_tags.txt');
    
    initial = [-0.6899; -0.1881; 3.4893; ...
               0.1540;  -0.0023; 0.9792; 0.1319; ...
               0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end
