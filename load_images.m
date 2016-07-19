function [ frames, num_rames ] = load_images( directory )
    frames = {};
    num_frames = 0;

    imageFiles = dir(directory);

    for imageFile = imageFiles
        imageName = imageFile.name;
        if (imageName == '.') || (imageName == '..')
            continue;
        end
        
        imgRGB = imread(imageFile.name);
        imgGray = rgb2gray(imgRGB);
        img = imresize(imgGray, [480, 640]);

        num_frames =  num_frames + 1;
        frames{num_frames} = img;
    end
end