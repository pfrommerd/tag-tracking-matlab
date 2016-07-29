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
            pos_pnoise = 1e-15;
            rot_pnoise = 1e-12;
            vel_pnoise = 1e-12;
            rot_vel_pnoise = 1e-12;
            %pnoise = 0;
            obj.process_noise = diag([pos_pnoise * [1 1 1] ...
                                      rot_pnoise * [1 1 1] ...
                                      vel_pnoise * [1 1 1] ...
                                      rot_vel_pnoise * [1 1 1]]);
            %obj.process_noise = diag(0 * ones([1, 12]));

            % Measurement noise is in the same
            % 12-dimensional matrix space [pos, rot, vel, rot_vel]
            mnoise = 1e-15;            
            obj.measure_noise = mnoise;
            
            obj.state = [0;0;0;1;0;0;0;0;0;0;0;0;0];
            
            % Covariance is 12 dimensional as quaternion is regarded
            % as 3 elements, not 4       
            obj.state_covar = 0.00001 * eye(12);
        end
        
        function setState(this, x)
            this.state = x;
        end
        
        function [x_out, y_out, z_out, zm, P_out, nu_out] = step(this, z, A, H)
            % Setup variables
            x = this.state; % Our last state
            P = this.state_covar; % Our last covariance 
            
            Q = this.process_noise; % Our process noise
            r = this.measure_noise; % Our measurement noise
            
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
            Y = A(chi);
            y_minus = calc_mean(Y);

            % W_p has the mean subtracted from Y and is in a
            % linear-friendly form (12xN)
            [P_yy, W_y] = calc_cov(Y, y_minus); % eqn (39, 64)
            
            % Transform to measurements
            Z = H(Y);

            
            %z_minus = mean(Z, 2);
            z_minus = H(y_minus);
            P_zz = cov(Z', 1);
            
            % Calculate the nu
            nu = z - z_minus;
            nu = filter_outliers(nu, 0.1);
            
            % Adjust the nu's to compensate for outliers
            %nu = filter_outliers(nu, 100);
            P_yz       = crosscov_nomean(W_y', (Z - z_minus)');
            P
            P_yz
            
            R = r * eye(size(P_zz));
            P_vv       = P_zz + R;       % eqn (45, 69)
            K          = P_yz * P_vv^-1; % eqn (72)

            x_out = add_x(K * nu, y_minus);
            y_out = Y;
            z_out = Z;
            zm = z_minus;
            P_out = P_yy - K * P_vv * K';

            % Debugging stuff
            nu_out = nu;
            
            % Set our variables again
            this.state = x_out;
            this.state_covar = P_out;
        end
    end
end

function y = filter_outliers(x, L)
    y = L * tanh(x / L);
end

%{
function y = filter_outliers(x, L)
    mag = [norm([x(1) x(2)]); norm([x(3) x(4)]); norm([x(5) x(6)]); norm([x(7) x(8)])];
    mult = [mag(1); mag(1); mag(2); mag(2); mag(3); mag(3); mag(4); mag(4)];
    y = x ./ mult; % Divide the x's by their magnitudes
    
    % Change the magnitudes
    mag = L * tanh(mag ./ L);
    
    mult = [mag(1); mag(1); mag(2); mag(2); mag(3); mag(3); mag(4); mag(4)];
    y = y .* mult; % Scale back the y's
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
    assert(cnt <= maxcnt);
    
    x_bar(4:7) = qbar;  % overwrite the q component
end

function [P, W] = calc_cov(X, x_bar)
    W = subtract_mean(X, x_bar);
    P = 1/size(W,2) * W*W';
end

function cc = crosscov(x, y)
    xbar = bsxfun(@minus,x,mean(x));
    ybar = bsxfun(@minus,y,mean(y));
    cc =   xbar' * ybar / size(x,1);
end

function cc = crosscov_nomean(x, y)
    cc =   x' * y / size(x,1);
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