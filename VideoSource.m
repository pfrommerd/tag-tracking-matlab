classdef VideoSource < FrameSource
    %VIDEOSOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        decoder
        
        useUndistort
        camParams
    end
    
    methods
        function source = VideoSource(file, camParams)
            source.decoder = VideoReader(file);
            
            source.useUndistort = false;
            if exist('camParams', 'var')
                source.useUndistort = true;
                source.camParams = camParams;
            end
        end
        
        function hasImage = hasImage(this)
            hasImage = hasFrame(this.decoder);
        end
        
        function img = readImage(this)
            imgRGB = readFrame(this.decoder);
            img = rgb2gray(imgRGB);
            
            % Undistort the image
            if this.useUndistort
                img = undistortImage(img, this.camParams);
            end
        end
    end
    
end

