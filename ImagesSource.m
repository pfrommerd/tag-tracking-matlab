classdef ImagesSource < FrameSource
    %IMAGESSOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dir
        files
        
        useUndistort
        camParams
    end
    
    methods
        function obj = ImagesSource(directory, camParams)
            files = dir(directory);
            
            obj.files = {};
            obj.dir = directory;
            i = 1;
            c = 1;
            while i < length(files);
                file = files(i).name;
                if file(1) == '.'
                else
                    obj.files{c} = file;
                    c = c + 1;
                end
                i = i + 1;
            end
            
            if exist('camParams', 'var')
                obj.useUndistort = true;
                obj.camParams = camParams;
            end            
        end
        
        function hasImage = hasImage(this)
            hasImage = (length(this.files) > 0);
        end
        
        function img = readImage(this)
            f = fullfile(this.dir, this.files{1});
            this.files(:, 1) = [];

            img = imread(f);
            if ndims(img) > 2
                img = rgb2gray(img);
            end
            
            if this.useUndistort
                img = undistortImage(img, this.camParams);
            end
        end
    end
    
end

