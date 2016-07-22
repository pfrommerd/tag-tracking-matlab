classdef CameraSource < ImageSource
    %CAMERASOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        camera
        camParams
    end
    
    methods
        function source = VideoSource(camera, camParams)
            source.camera = camera;
            source.camParams = camParams;
        end
        
        function hasImage = hasImage(this)
            hasImage = true;
        end
        
        function img = readImage(this)
            imgRGB = snapshot(this.decoder);
            imgGray = rgb2gray(imgRGB);
            % Undistort the image
            img = undistortImage(imgGray, this.camParams);
        end
    end
    
end

