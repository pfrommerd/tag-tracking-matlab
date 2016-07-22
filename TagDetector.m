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
        
        function tags = process(this, img, lastTag)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = {};
            for i = 1:resultsSize(2)
                r = results(i);
                
                tagSize = this.tagSize / 2;
                
                % TL
                % TR
                % BL
                % BR
                pin = [-tagSize, tagSize; ...
                     tagSize tagSize; ...
                     tagSize -tagSize;...
                     -tagSize -tagSize]';
                
                %H = homography_solve(pin, r.corners');
                %[R, T] = homography_extract_pose(this.K, H);
                R = r.rotation';
                T = -r.translation;
                
                % Comute a quaternion from a rot mat
                rot = rotm_to_quat(R)';
                delta_q = qmult(qinv(lastTag(4:7)'), rot')';
                
                tags{i} = [ T; rot; T - lastTag(1:3); quat_to_rotvec(delta_q')'];
     %{
                tags{i}.translation = r.translation;
                tags{i}.rotation = transpose(r.rotation);
                tags{i}.rotation_quat = rotm_to_quat(transpose(r.rotation));
     %}
            end
            
        end
    end
    
end

