function [ R, T ] = homography_extract_pose( K, H )
%HOMOGRAPHY_EXTRACT_POSE Extracts the rotation and translation from a
% homography
H = inv(K) * H;

r1 = H(:, 1);
r2 = H(:, 2);
r3 = cross(r1, r2);

R_h = [ r1 r2 r3 ];

% Clean up R_h
[U, D, V] = svd(R_h);

R = U * [ 1 0 0; 0 1 0; 0 0 det(U * V') ] * V';
T = H(:, 3) / norm(H(:, 1));

end