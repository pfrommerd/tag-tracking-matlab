classdef FrameSource < handle
    %IMAGESOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function hasImage = hasImage(this)
            hasImage = false;
        end
        
        function img = readImage(this)
            img = [];
        end
    end
    
end

