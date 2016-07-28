classdef TagTracker < TagSource
    properties
        tracker
        initialImage
        initialized
        
        % The tags we are tracking
        % kept so that we know what
        % tags we are tracking
        tags
        idxMap % A map from tag id to index in tags
    end
    
    methods
        function obj = TagTracker(np)
            obj.tracker = vision.PointTracker();
            %obj.tracker.BlockSize = [63 63];
            obj.initialized = false;
            obj.initialImage = [];
            
            obj.tags = {};
        end
        
        % Setting an existing tag
        % Will just update it's position
        function set(this, tags)
            for i=1:length(tags)
                t = tags{i};
                
                % Check if we have it already in the tags
                % array
                idx = length(this.tags) + 1;
                for j=1:length(this.tags)
                    if this.tags{j}.id == t.id
                        idx = j;
                        break;
                    end                    
                end
                
                this.tags{idx} = t;
            end

            % Updates the points we are tracking
            this.update();
        end
        
        function remove(this, tags)
            % Do nothing for now
        end
        
        function removeIdx(this, idx)
            this.tags(:, idx) = [];
            this.update();
        end
        
        % Update the points we are tracking
        function update(this)
            % Construct a points matrix            
            points = zeros(length(this.tags) * 4, 2);
            for i=1:length(this.tags)
                points(4*i-3:4*i, :) = this.tags{i}.corners;
            end
            
            if ~this.initialized
                if max(size(this.initialImage)) > 0
                    this.tracker.initialize(points, this.initialImage);
                    this.initialized = true;
                end
                return;
            end
            
            this.tracker.setPoints(points);
        end
        
        function tags = process(this, img)
            tags = {};
            if ~this.initialized
                this.initialImage = img;
                return;
            end

            [points, validity] = this.tracker.step(img);    
            
            i = 1;
            p = 0;
            while i < length(this.tags)
                p = p + 1;
                
                v = validity(4*p-3:4*p);
                idx = find(v < 1);
                if length(idx > 0)
                    this.removeIdx(i);
                    continue;
                end
                
                corners = points(4*p-3:4*p, :);
                
                this.tags{i}.corners = corners;
                t.id = this.tags{i}.id;
                t.color = 'y';
                t.corners = this.tags{i}.corners;
                tags{i} = t;
                i = i + 1;
            end  
        end
    end
end
