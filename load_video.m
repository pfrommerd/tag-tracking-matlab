function [ frames, num_frames ] = load_video( file, camParams )
    videoReader = VideoReader(file);
    frames = {};
    num_frames = 0;

    disp('Reading frames');

    while hasFrame(videoReader)
        num_frames =  num_frames + 1;

        imgRGB = readFrame(videoReader);
        imgGray = rgb2gray(imgRGB);
        % Undistort the image
        [img, center] = undistortImage(imgGray,camParams);
        
        frames{num_frames} = img;
    end

    disp('Done reading frames');
end