classdef TagDetector < TagSource
    properties
        tagSize
        camParams
        params
        
        K
    end
    
    methods
        function detector = TagDetector(tagSize, camParams)
            detector.tagSize = tagSize;
            detector.K = transpose(camParams.IntrinsicMatrix);
            params = [detector.K(1, 1) detector.K(2, 2) ...
                      detector.K(1, 3) detector.K(2, 3)];
            
            detector.camParams = camParams;
            detector.params = params;
        end
        
        function tags = process(this, img)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = zeros(8, resultsSize(2));
            for i = 1:resultsSize(2)
                r = results(i);
                tag = reshape(r.corners, [8, 1]);
                tags(:,i) = tag;
            end
            
        end
    end
    
end

