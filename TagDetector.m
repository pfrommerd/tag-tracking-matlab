classdef TagDetector < TagSource
    properties
        tagSize
        params        
        K
    end
    
    methods
        function detector = TagDetector(K, tagSize)
            detector.tagSize = tagSize;
            detector.K = K;
            params = [K(1, 1) K(2, 2) ...
                      K(1, 3) K(2, 3)];            
            detector.params = params;
        end
        
        function tags = process(this, img)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = {};
            for i = 1:resultsSize(2)
                r = results(i);
                
                tag.id = r.id;
                tag.color = 'r';
                tag.corners = r.corners';
                tags{i} = tag;
            end
        end
    end
    
end

