classdef Filter < handle
    %FILTER A Unscented Kalman-Filter-based tag filter
    
    % The filter state variable is of length 13:
    % [pos(3), rot(4), vel(3), rot_vel(3)]
    
    properties
        process_noise
        measure_noise
        
        state
        state_covar
    end
    
    methods
        function obj = Filter()
            % Process noise is in
            % the 12-dimensional matrix space [pos, rot, vel, rot_vel]
            pnoise = 1e-3;
            %pnoise = 0;
            obj.process_noise = diag(pnoise * ones(1, 12));

            % Measurement noise is in the same
            % the 12-dimensional matrix space [pos, rot, vel, rot_vel]
            mnoise = 1e-4;
            obj.measure_noise = diag(mnoise * ones(1, 12));
            obj.state = zeros(13, 1);
            % Set initial rotation to 0
            obj.state(4:7) = [1; 0; 0; 0]; 
            
            % Covariance is 12 dimensional as quaternion is regarded
            % as 3 elements, not 4
            obj.state_covar = eye(12);        
        end
        
        function [x_out, P_out] = step(this, z, dt)
            % Setup variables
            x = this.state; % Our last state
            P = this.state_covar; % Our last covariance 
            Q = this.process_noise; % Our process noise
            R = this.measure_noise; % Our measurement noise
            
            % n is the dimensionality in the vector space
            n = size(P, 1);
            % x_n is the dimensionality in the vector-quaternion space
            x_n = length(x);
            
            % Process time step update
            S   = chol((P + Q));   % add process noise first (eq 35)
            W   = sqrt(n)*[S, -S]; % this is where the paper has sqrt(2n)
            % Our sigmas
            chi = add_x(W, x);
            
            % Project the sigmas using our process model
            Y = A(chi, dt);
            y_minus = calc_mean(Y);

            % W_p has the mean subtracted from Y and is in a
            % linear-friendly form (12xN)
            [P_yy, W_y] = calc_cov(Y, y_minus); % eqn (39, 64)
            
            % Since Z = Y
            Z = Y;
            z_minus = y_minus;
            P_zz = P_yy;
            W_z = W_y;
            
            % Calculate the nu
            % For all the regular elements we can subtract
            diff = (z - z_minus);
            % Do special quaternion subtracting
            delta_qz = qmult(qinv(z_minus(4:7)'), z(4:7)')';
            nu = [diff(1:3); quat_to_rotvec(delta_qz')'; diff(8:13)];
            
            P_xz       = P_zz;
            P_vv       = P_zz + R;       % eqn (45, 69)
            K          = P_xz * P_vv^-1; % eqn (72)

            x_out = add_x(K * nu, y_minus);
            P_out = P_yy - K * P_vv * K';
            
            % Set our variables again
            this.state = x_out;
            this.state_covar = P_out;
        end
    end
end

function new_x = A(x, deltaT)
    new_x = x;
    % Update the position and the orientation
    new_x(1:3) = x(1:3) + deltaT * x(8:10); % velocity
    new_x(4:7) = qmult(x(4:7), rotvec_to_quat(x(11:13) * deltaT)); % angular velocity
end

%{
function X = create_sigmas(prev_state, prev_state_covar, alpha, k)
    % N is a matrix which contains the offset for each
    % sigma point
    L = max(size(prev_state_covar));
    lamda = alpha * alpha * (L + k) - L;

    % Calculate our directional changes
    N = sqrtm((L + lamda) .* prev_state_covar);
    % Create the sigma points matrix
    % by disturbing the previous state by every column of N
    % and storing each resulting state vector in a column of X
    X = zeros(13, 25);
    
    X(:, 1) = prev_state;
    
    for i=1:12
        n = N(:,i);
        X(:, 2*i) = disturb(prev_state, n);
        X(:, 2*i + 1) = disturb(prev_state, -n);
    end
end
%}

function x_bar = calc_mean(X)
    x_bar = mean(X, 2);

    % now average the q component
    %
    lastq = zeros(4,1);
    qbar = X(4:7,1);    % pick the first q as starting point
    e    = zeros(3, size(X,2));
    cnt  = 0; maxcnt = 1000;
    while (norm(qbar -lastq) > 1e-8) & (cnt < maxcnt)
        lastq = qbar;
        for i = 1:size(X,2)
            eq = qmult(X(4:7,i)', qinv(qbar'));
            e(:, i) = quat_to_rotvec(eq)';
        end
        eavg = mean(e')';
        qbar = qmult(rotvec_to_quat(eavg'), qbar')';
        cnt  = cnt + 1;
    end
    assert(cnt < maxcnt);
    
    x_bar(4:7) = qbar;  % overwrite the q component
end

function [P, W] = calc_cov(X, x_bar)
    W = subtract_mean(X, x_bar);
    P = 1/size(W,2) * W*W';
end

function W = subtract_mean(chi, xbar)
    W = zeros(12, size(chi,2));
    qbarinv = qinv(xbar(4:7)')';
    for i = 1:size(chi, 2)
        r = qmult(qbarinv', chi(4:7, i)')';
        diff = chi(:, i) - xbar;
        W(:, i) = [diff(1:3); [0; 0; 0]; diff(8:13)];
        W(4:6, i) = quat_to_rotvec(r')';
    end
end

function chi = add_x(W, x)
    chi = zeros(size(x,1), size(W, 2));
    for i = 1:size(W,2)
        % Add a zero as our quaternion takes
        % up 1 more space.
        d = [W(1:6, i); 0; W(7:12, i)];
        chi(:, i) = d + x;
        % Handle the quaternion as a special case
        chi(4:7, i) = (qmult(x(4:7)',rotvec_to_quat(W(4:6,i)')))';
    end
end