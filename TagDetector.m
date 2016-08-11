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
            results = find_apriltags(img, this.tagSize(1), this.params);
            resultsSize = size(results);

            tags = cell(1, resultsSize(2));
            for i = 1:resultsSize(2)
                r = results(i);

                corners = r.corners';
                
                tagSize = this.tagSize/2;
                pin = [-tagSize(1), -tagSize(2); ...
                        tagSize(1), -tagSize(2); ...
                        tagSize(1),  tagSize(2);...
                       -tagSize(1),  tagSize(2)];

                H = homography_solve(pin, corners);
                % Extract the rotation and translation
                [R, T] = homography_extract_pose(this.K, H);

                % Comute a quaternion from a rot mat
                rot = rotm_to_quat(R)';
                state = [ T; rot; 0; 0; 0; 0; 0; 0];
                
                tag.id = r.id;
                tag.state = state;
                tag.color = 'r';
                
                tags{i} = tag;
            end
        end
    end
    
end

