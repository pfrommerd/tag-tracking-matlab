classdef ImagesSource < FrameSource
    %IMAGESSOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        files
        index
        
        skip_rate
        
        useUndistort
        camParams
    end
    
    methods
        function obj = ImagesSource(directory, initial_skip, skip_rate, camParams)
            files = dir(directory);
            
            obj.files = {};
            obj.index = 1 + initial_skip;
            obj.skip_rate = skip_rate;
            
            
            i = 1;
            c = 1;
            while i <= length(files);
                file = files(i).name;
                if file(1) == '.'
                else
                    obj.files{c} = fullfile(directory, file);
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
            hasImage = (length(this.files) >= this.index);
        end
        
        function img = readImage(this)
            f = this.files{this.index};
            disp('Reading from:');
            disp(f);
            this.index = this.index + this.skip_rate;
            
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

