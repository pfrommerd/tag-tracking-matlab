classdef TagDetector < TagSource
    properties
        tagSize
        params        
        K
    end
    
    methods
        function detector = TagDetector(K)
            detector.tagSize = [0.1635 0.1635];
            detector.K = K;
            params = [K(1, 1) K(2, 2) ...
                      K(1, 3) K(2, 3)];            
            detector.params = params;
        end
        
        function tags = process(this, img)
            printf('--> Detecting tags'); fflush(stdout);
            tic();

            results = find_apriltags(img, this.tagSize(1), this.params);

            

            tags = cell(1, length(results));
            for i = 1:length(tags);
                r = results(i);

                corners = r.corners';

                %{
                tagSize = this.tagSize/2;
                pin = [-tagSize(1), -tagSize(2); ...
                        tagSize(1), -tagSize(2); ...
                        tagSize(1),  tagSize(2);...
                       -tagSize(1),  tagSize(2)];

                H = homography_solve(pin, corners);
                [R, T] = homography_extract_pose(this.K, H);
                
                rot = rotm_to_quat(R)';

                T
                rot
                
                %}

                tag.id = r.id;
                tag.corners = corners;
                tag.color = 'r';
                tags{i} = tag;
            end

            printf('; Took: %f\n', toc()); fflush(stdout);
        end
    end
    
end

