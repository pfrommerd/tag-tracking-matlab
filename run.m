function [  ] = run(config, shouldRecord)
    tracker = [];
    detector = [];
    frames = [];
    M = [];
    
    switch config
        case 'hard_seq'
            [tracker, detector, frames] = setup_hard_seq();
        case 'test_video'
            [tracker, detector, frames] = setup_test_video();
        case 'movcam1'
            [tracker, detector, frames] = setup_movcam1();
        otherwise
            return;
    end
    
    if (exist('shouldRecord', 'var') && shouldRecord)
        M = algorithm(tracker, detector, frames, true);
    else
        algorithm(tracker, detector, frames, false);
    end
end

function [tracker, detector, frames] = setup_hard_seq()
    frames = ImagesSource('../data/hard_seq/images');

    % Image parameters
    K = [568.8885   0           959.1503;
         0          784.9941    539.7587;
         0          0           1.0000];

    % Setup some detector stuff
    detector = TagDetector(K);
    trackParams(1).K = K;
    
    % Patch size
    trackParams(1).patchSize = [64 64];
    
    % Create the tracker
    tracker = AprilTrack(trackParams);

    % Add a motion model
    mmParams(1).err_discard_threshold = 0.9;
    mmParams(1).num_particles = 1000;
    mmParams(1).process_noise = [0.03 0.03 0.03 ...
                                 0.01 0.01 0.01 ...
                                 0 0 0 ...
                                 0 0 0];

    % noise = 1 / (k + alpha * w)
    % Allows for particles of higher/lower weight to have
    % more noise
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 10;

    model = MotionModel(mmParams, @ego_transform, @ego_invtransform, ...
                        @ego_invtransform2);
    model.loadTags('../data/hard_seq/tags.txt');

    initial_pos = [1.1029; 0.2798; -0.3398;  0.499; 0.499; -0.501; 0.501; 0; 0; 0; 0; 0; 0];
    initial_random = [[0.1; 0.1; 0.1];  [0.05; 0.05; 0.05]; [0; 0; 0]; [0; 0; 0]];

    model.initializeParticlesRandom(initial_pos, initial_random);

    tracker.addMotionModel(model);
end


function [tracker, detector, frames] = setup_test_video()
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
    mmParams(1).num_particles = 4000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.01 0.01 0.01 ...
                                 0.02 0.02 0.02 ...
                                 0.01 0.01 0.01];

    % noise = 1 / (k + alpha * w) * process_noise * randn
    % Allows for particles of higher/lower weight to have
    % more noise in the propagation step

    % ATM, we resample after every pass, so don't bother...
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 6;

    model = MotionModel(mmParams, @tag_transform);

    model.loadTags('../data/test/tags.txt');
    
    initial = [0.1862224; -0.0067017; 0.696942; ...
               0.991367; -0.124779; -0.014736; 0.037472;...
               0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end

function [tracker, detector, frames] = setup_movcam1()
    frames = ImagesSource('../data/movcam1/images');

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
    mmParams(1).err_discard_threshold = 0.01;
    mmParams(1).num_particles = 10000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.02 0.02 0.02 ...
                                 0.005 0.005 0.005 ...
                                 0.005 0.005 0.005];
                             
    % noise = 1 / (k + alpha * w) * process_noise * randn
    % Allows for particles of higher/lower weight to have
    % more noise in the propagation step

    % ATM, we resample after every pass, so don't bother...
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 6;

    model = MotionModel(mmParams, @transform);

    model.loadTags('../data/movcam_tags.txt');
    
    initial = [-0.424; -0.7353; 3.4589; ...
               0.9791; 0.1296; -0.1559; -0.0135; ...
               0; 0; 0;  0; 0; 0];
    
    model.initializeParticlesTo(initial);
    
    tracker.addMotionModel(model);
end


% The motion model stuff

% For tag-motion

function transState = tag_transform(state, x)
    transState = state;
    transState(1:3) =  x(1:3) + state(1:3);
    transState(4:7) = qmult(x(4:7)', state(4:7)')';
end

% For ego-motion

function newPose = transform(tagPose, x)
    % Local -> World transform
    WL = [[quat_to_rotm(tagPose(4:7)') [tagPose(1); tagPose(2); tagPose(3)]]; [0 0 0 1]];
    % World -> Camera transform
    CW = [[quat_to_rotm(x(4:7)') [x(1); x(2); x(3)]]; [0 0 0 1]];
    T = CW * WL;
    newPose = tagPose;
    newPose(1:3) = [T(1, 4) T(2, 4) T(3, 4)];
    newPose(4:7) = rotm_to_quat(T(1:3, 1:3));
end