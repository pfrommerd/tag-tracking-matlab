function y = q_smpl(x, w, deltaT, r, k, alpha)
    % Propagates the particles from 
    % the previous particles x with weights w
    % and process noise r
    n = size(x, 2);
    
    y = zeros([size(x, 1) n]);
    
    for i=1:size(x, 2)
        xp = x(:, i);
        % Add some noise        
        noise = 1 / (k + alpha * w(i)) * (r .* randn(1, 12))';
        xp(:) = xp(:) + [noise(1:6); 0; noise(7:12)];
        % Special case for the quaternion
        xp(4:7) = qmult(x(4:7, i)', rotvec_to_quat(noise(4:6)'))';
        
        % Add the velocity
        xp(1:3) = xp(1:3) + deltaT * xp(8:10); 
        % angular velocity
        xp(4:7) = qmult(xp(4:7)', ...
                      rotvec_to_quat((x(11:13, i) * deltaT)')); 
        
        y(:, i) = xp;
        %y(4:7, i) = [1 0 0 0];
        %y(3, i) = x(3, i);
    end
end

