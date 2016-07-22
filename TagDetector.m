classdef TagDetector < TagSource
    properties
        tagSize
        camParams
        params
    end
    
    methods
        function detector = TagDetector(tagSize, camParams)
            detector.tagSize = tagSize;
            intrinsics = transpose(camParams.IntrinsicMatrix);
            params = [intrinsics(1, 1) intrinsics(2, 2) intrinsics(1, 3) intrinsics(2, 3)];
            
            detector.camParams = camParams;
            detector.params = params;
        end
        
        function tags = process(this, img)
            results = find_apriltags(img, this.tagSize, this.params);
            resultsSize = size(results);

            tags = {};
            for i = 1:resultsSize(2)
                r = results(i);
                % Comute a quaternion from a rot mat
                tags{i} = [ r.translation; rotm_to_quat(r.rotation')'; 0; 0; 0; 0; 0; 0];
    %{ 
                tags{i}.translation = r.translation;
                tags{i}.rotation = transpose(r.rotation);
                tags{i}.rotation_quat = rotm_to_quat(transpose(r.rotation));
     %}
            end
            
        end
    end
    
end

