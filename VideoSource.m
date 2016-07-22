classdef VideoSource < ImageSource
    %VIDEOSOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        decoder
        camParams
    end
    
    methods
        function source = VideoSource(file, camParams)
            source.decoder = VideoReader(file);
            source.camParams = camParams;
        end
        
        function hasImage = hasImage(this)
            hasImage = hasFrame(this.decoder);
        end
        
        function img = readImage(this)
            imgRGB = readFrame(this.decoder);
            imgGray = rgb2gray(imgRGB);
            % Undistort the image
            img = undistortImage(imgGray, this.camParams);
        end
    end
    
end

