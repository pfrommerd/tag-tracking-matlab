classdef TagDetector < TagSource
    properties
        num_pts

        tagSize
        camParams
        params
        
        K
    end
    
    methods
        function detector = TagDetector(tagSize, camParams, np)
            detector.tagSize = tagSize;
            detector.K = transpose(camParams.IntrinsicMatrix);
            params = [detector.K(1, 1) detector.K(2, 2) ...
                      detector.K(1, 3) detector.K(2, 3)];
            
            detector.camParams = camParams;
            detector.params = params;
            detector.num_pts = np;
        end
        
        function tags = process(this, img)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = [];
            for i = 1:resultsSize(2)
                r = results(i);
                corners = [r.corners];
                tag = reshape(corners, [size(corners, 2) * 2, 1]);
                tags = [tags tag];
            end
            
        end
    end
    
end

