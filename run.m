function [ M ] = run(config, shouldRecord)
    tracker = [];
    detector = [];
    frames = [];
    M = [];
    
    
    % The parameter argument represents the number of frames to skip
    % so zero means always stay on the first frame
    
    switch config
        case 'hard_seq'
            [tracker, detector, frames] = hard_seq_config(1);
        case 'test_video'
            [tracker, detector, frames] = test_video_config();
        case 'movcam1'
            [tracker, detector, frames] = movcam1_config(1);
        case 'movcam1-still'
            [tracker, detector, frames] = movcam1_config(0);
        otherwise
            return;
    end
    
    if (exist('shouldRecord', 'var') && shouldRecord)
        M = algorithm(tracker, detector, frames, true);
    else
        algorithm(tracker, detector, frames, false);
    end
end