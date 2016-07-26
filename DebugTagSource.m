classdef DebugTagSource < TagSource
    %DEBUGTAGSOURCE
    properties
        time
        
        H % The measurement transform function
    end
    
    methods
        function obj = DebugTagSource(H)
            obj.time = 0;
            obj.H = H;
        end
        
        function tags = process(this, img)
            % Update the time
            this.time = this.time + 0.1;
            time = this.time;
            
            x = zeros(13, 1);
            % Set the position
            x(1:3) = [time * 0.001; sin(0.1*time) * 0.1; 1];
            x(1:3) = [0; 0; 1];
            
            % Rotate the tag around the y axis
            rVec = [0 time * sin(0.01*time) 0];
            rVec = [0 0 0];
            
            % Set the rotation
            x(4:7) = rotvec_to_quat(rVec);

            % Now project the points
            tag = this.H(x);

            tags = tag + ( 0.1 * randn([8, 1]) );
        end    
    end
end

