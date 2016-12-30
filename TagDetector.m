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
        
        function [tags] = process(this, img)
            fprintf('--> Detecting tags');
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
                r.id
                
                H_0 =  [0.5417    0.0623    0.6447;
                        0.0138    0.5204    0.1269;
                        0.0001    0.0001    0.0009];

                pix_center = H(:,3) ./ H(3,3);
                
                T_rel = inv(H_0) * pix_center;
                T_rel = T_rel ./ T_rel(3);
                T_rel * 1000
                %rot = rotm_to_quat(R')';
                
                %}
                
                tag.id = r.id;
                tag.corners = corners;
                tag.color = 'r';
                tags{i} = tag;
            end

            fprintf('; Took: %f\n', toc());
        end
    end
    
end

