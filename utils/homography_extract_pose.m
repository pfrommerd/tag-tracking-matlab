function [ R, T ] = homography_extract_pose( K, H )
%HOMOGRAPHY_EXTRACT_POSE Extracts the rotation and translation from a
% homography
H = inv(K) * H;

h1 = H(:, 1);
h2 = H(:, 2);

R_h = [ h1 h2 cross(h1, h2) ];

% Clean up R_h
[U, D, V] = svd(R_h);

R = U * [ 1 0 0; 0 1 0; 0 0 det(U * V') ] * V';
T = H(:, 3) / norm(H(:, 1));

end