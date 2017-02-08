function [ imgs, corners, poses, rot ] = run(config, shouldRecord)
    tracker = [];
    detector = [];
    frames = [];
    imgs = {};
    corners = [];
    poses = [];
    
    % The parameter argument represents the number of frames to skip
    % so zero means always stay on the first frame
    
    switch config
        case 'hard_seq'
            [tracker, detector, frames] = hard_seq_config(1);
        case 'hard_seq-still'
            [tracker, detector, frames] = hard_seq_config(0);
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
        [imgs, corners, poses] = algorithm(tracker, detector, frames, true);
    else
        [imgs, corners, poses] = algorithm(tracker, detector, frames, false);
    end
end