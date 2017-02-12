function [tracker, detector, frames] = movcam1_config(initial_skip, skip_rate)
    frames = ImagesSource('../data/movcam1/images', initial_skip, skip_rate);

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
    mmParams(1).num_particles = 6000;
%    mmParams(1).process_noise = [0.05 0.05 0.05 ...
%                                 0.05 0.05 0.05 ...
%                                 0.02 0.02 0.02 ...
%                                 0.02 0.02 0.02];
    mmParams(1).process_noise = [0.02 0.02 0.02 ...
                                 0.02 0.02 0.02 ...
                                 0.01 0.01 0.01 ...
                                 0.01 0.01 0.01];

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @transform_ego);

    model.loadTags('../data/movcam_tags.txt');
    
    %initial = [0.424; 0.7353; -3.4589; ...
    %             0.9919; -0.0018; 0.1246; 0.0231; ...
    %               0; 0; 0;  0; 0; 0];
    
    initial = [0.4673; 0.4104; -3.6941; ...
                0.9986; -0.0519; 0.0000; -0.0028; ...
                0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end
