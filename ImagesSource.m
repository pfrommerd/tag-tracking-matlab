classdef ImagesSource < FrameSource
    %IMAGESSOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dir
        files
    end
    
    methods
        function obj = ImagesSource(directory)
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
        end
        
        function hasImage = hasImage(this)
            hasImage = (length(this.files) > 0);
        end
        
        function img = readImage(this)
            f = fullfile(this.dir, this.files{1});
            this.files(:, 1) = [];

            img = imread(f);
            img = rgb2gray(img);
        end
    end
    
end

