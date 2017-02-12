function [ imgs, poses] = run(config, initial_skip, skip_rate, save_images, save_poses)
    tracker = [];
    detector = [];
    frames = [];
    imgs = {};
    poses = [];
    
    % The parameter argument represents the number of frames to skip
    % so zero means always stay on the first frame
    
    switch config
        case 'kumar_c2'
            [tracker, detector, frames] = kumar_c2_config(initial_skip, skip_rate);
        case 'test_video'
            [tracker, detector, frames] = test_video_config(initial_skip, skip_rate);
        case 'movcam1'
            [tracker, detector, frames] = movcam1_config(initial_skip, skip_rate);
        otherwise
            return;
    end
    
    algorithm(tracker, detector, frames, initial_skip, skip_rate, save_images, save_poses);
end