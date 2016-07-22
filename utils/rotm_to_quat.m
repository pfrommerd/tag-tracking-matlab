function [ quat ] = rotm_to_quat( mat )
q1 = sqrt(1 + mat(1,1) + mat(2, 2) + mat(3, 3))/2;

quat = [q1, ...
        (mat(3, 2) - mat(2, 3))/( 4 *q1), ...
        (mat(1, 3) - mat(3, 1))/( 4 *q1), ...
        (mat(2, 1) - mat(1, 2))/( 4 *q1)];

end

