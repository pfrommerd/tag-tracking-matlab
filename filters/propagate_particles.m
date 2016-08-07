function y = propagate_particles(x, r, deltaT)
    % Propagates the particles from 
    % the previous particles x with weights w
    % and process noise r
    n = size(x, 2);
    
    y = zeros([size(x, 1) n]);
    
    for i=1:size(x, 2)
        
        xp = x(:, i);
        % Add the velocity
        % add the rotational velocity
        
        xp(1:3) = x(1:3, i) + deltaT * x(8:10, i); 
        % angular velocity
        xp(4:7) = qmult(x(4:7, i)', ...
                           rotvec_to_quat((x(11:13, i) * deltaT)')); 
        
        % Add some noise
        noise = (r .* randn(1, 12))';

        y(:, i) = xp(:) + [noise(1:6); 0; noise(7:12)];
        % Special case for the quaternion
        y(4:7, i) = qmult(xp(4:7)', rotvec_to_quat(noise(4:6)'))';
        %y(4:7, i) = [1 0 0 0];
        %y(3, i) = x(3, i);
    end
end

