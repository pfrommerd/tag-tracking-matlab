function [  ] = run(config)
    tracker = [];
    frames = [];

    switch config
        case 'seq1'
            [tracker, frames] = setup_seq1();
        case 'test1'
            [tracker, frames] = setup_test1();
        otherwise
            return;
    end

    algorithm(tracker, frames, false);
end

function [tracker, frames] = setup_seq1()
    frames = ImagesSource('../seq1_still');

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
    tracker = AprilTrack(detector, trackParams);

    % Add a motion model
    mmParams(1).err_discard_threshold = 0.9;
    mmParams(1).num_particles = 1000;
    mmParams(1).process_noise = [0.03 0.03 0.03 ...
                                 0.01 0.01 0.01 ...
                                 0 0 0 ...
                                 0 0 0];
    % Weight of the tag corner error
    % error = rho * sqr(corner error)
    mmParams(1).rho = 1e-10;

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
    model.loadTags('../tags_seq1.txt');

    initial_pos = [1.1029; 0.2798; -0.3398;  0.499; 0.499; -0.501; 0.501; 0; 0; 0; 0; 0; 0];
    initial_random = [[0.1; 0.1; 0.1];  [0.05; 0.05; 0.05]; [0; 0; 0]; [0; 0; 0]];

    model.initializeParticlesRandom(initial_pos, initial_random);

    tracker.addMotionModel(model);
end


function [tracker, frames] = setup_test1()
    frames = ImagesSource('../test_video');

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
    tracker = AprilTrack(detector, trackParams);

    % Add a motion model
    mmParams(1).err_discard_threshold = 0.9;
    mmParams(1).num_particles = 2000;
    mmParams(1).process_noise = [0.01 0.01 0.01 ...
                                 0.05 0.05 0.05 ...
                                 0 0 0 ...
                                 0 0 0];
    % Weight of the tag corner error
    % error = rho * sqr(corner error)
    mmParams(1).rho = 1e-10;

    % noise = 1 / (k + alpha * w)
    % Allows for particles of higher/lower weight to have
    % more noise
    mmParams(1).k = 1;
    mmParams(1).alpha = 0;

    % For measurement error --> weight conversion
    % where weight = e^(-lambda * measurement)
    mmParams(1).lambda = 8;

    model = MotionModel(mmParams, @tag_transform, @tag_invtransform, ...
                        @tag_invtransform2);

    model.loadTags('../tags_test1.txt');
    
    initial = [0.1862224; -0.0067017; 0.696942; ...
               0.991367; -0.124779; -0.014736; 0.037472;...
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

function x = tag_invtransform(state, transState)
    x = state;
    x(1:3) = transState(1:3) - state(1:3);
    x(4:7) = qmult(transState(4:7)',qinv(state)')';
end

function state = tag_invtransform2(x, transState)
    state = x;
    state(1:3) = state(1:3) - transState(1:3);
    state(1:4) = transState(1:4);
end

% For ego-motion

function transState = ego_transform(state, x)
    R = [[quat_to_rotm(x(4:7)') [0; 0; 0]]; [0 0 0 1]];
    T = [1 0 0 -x(1); 0 1 0 -x(2); 0 0 1 -x(3); 0 0 0 1];

    R2 = [[quat_to_rotm(state(4:7)') [0; 0; 0]]; [0 0 0 1]];
    T2 = [1 0 0 state(1); 0 1 0 state(2); 0 0 1 state(3); 0 0 0 1];

    C = R * T * T2 * R2;

    transState = state;
    transState(1:3) = [C(1, 4) C(2, 4) C(3, 4)];
    transState(4:7) = rotm_to_quat(C(1:3, 1:3));
end

function x = ego_invtransform(state, transState)
    C = [[quat_to_rotm(transState(4:7)') ...
          [transState(1); transState(2); transState(3)]]; [0 0 0 1]];

    R2 = [[quat_to_rotm(state(4:7)') [0; 0; 0]]; [0 0 0 1]];
    T2 = [1 0 0 state(1); 0 1 0 state(2); 0 0 1 state(3); 0 0 0 1];
    C = C * inv(R2) * inv(T2);

    % Decompose C
    R_t = C(1:3, 1:3);
    T_t = R_t' * C(1:3, 4);
    x = state;
    x(1:3) = -T_t;
    x(4:7) = rotm_to_quat(R_t);
end

function state = ego_invtransform2(transState, x)
    C = [[quat_to_rotm(transState(4:7)') ...
          [transState(1); transState(2); transState(3)]]; [0 0 0 1]];

    R = [[quat_to_rotm(x(4:7)') [0; 0; 0]]; [0 0 0 1]];
    T = [1 0 0 -x(1); 0 1 0 -x(2); 0 0 1 -x(3); 0 0 0 1];

    C = inv(R) * inv(T) * C;

    R_t = C(1:3, 1:3);
    T_t = C(1:3, 4);

    state = zeros(13, 1);
    state(1:3) = T_t;
    state(4:7) = rotm_to_quat(R_t);
end