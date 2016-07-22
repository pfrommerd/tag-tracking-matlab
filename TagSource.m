classdef TagSource < handle
    %TAGSOURCE A source of tag positions to input into the filter
    % All subclasses should override the process() function
    
    properties
    end
    
    methods
        function tags = process(this, img)
            tags = [];
        end
    end
    
end

