function [ imgs, poses] = run(config, shouldRecord)
    tracker = [];
    detector = [];
    frames = [];
    imgs = {};
    poses = [];
    
    % The parameter argument represents the number of frames to skip
    % so zero means always stay on the first frame
    
    switch config
        case 'kumar_c2'
            [tracker, detector, frames] = kumar_c2_config(1);
        case 'kumar_c2-still'
            [tracker, detector, frames] = kumar_c2_config(0);
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
        [imgs, poses] = algorithm(tracker, detector, frames, true);
    else
        [imgs, poses] = algorithm(tracker, detector, frames, false);
    end
end