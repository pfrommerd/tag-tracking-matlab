function x = projectTag(K, tagSize, tag)
    X = [[-tagSize/2; tagSize/2; 0; 1] ...
              [tagSize/2; tagSize/2; 0; 1] [tagSize/2; -tagSize/2; 0; 1] ...
              [-tagSize/2; -tagSize/2; 0; 1]];
    
    rot = tag(4:7);
    trans = tag(1:3);
          
    RT = [ quat_to_rotm(rot) trans];
    
    x = K * RT * X;
    x = bsxfun(@rdivide, x, x(3,:));
    x = x(1:2, :);
end