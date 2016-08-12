%{
function v = homography_solve(pin, pout)
% HOMOGRAPHY_SOLVE finds a homography from point pairs
%   V = HOMOGRAPHY_SOLVE(PIN, POUT) takes a 2xN matrix of input vectors and
%   a 2xN matrix of output vectors, and returns the homogeneous
%   transformation matrix that maps the inputs to the outputs, to some
%   approximation if there is noise.
%
%   This uses the SVD method of
%   http://www.robots.ox.ac.uk/%7Evgg/presentations/bmvc97/criminispaper/node3.html
% David Young, University of Sussex, February 2008
pin = pin';
pout = pout';

if ~isequal(size(pin), size(pout))
    error('Points matrices different sizes');
end
if size(pin, 1) ~= 2
    error('Points matrices must have two rows');
end
n = size(pin, 2);
if n < 4
    error('Need at least 4 matching points');
end
% Solve equations using SVD
x = pout(1, :); y = pout(2,:); X = pin(1,:); Y = pin(2,:);
rows0 = zeros(3, n);
rowsXY = -[X; Y; ones(1,n)];
hx = [rowsXY; rows0; x.*X; x.*Y; x];
hy = [rows0; rowsXY; y.*X; y.*Y; y];
h = [hx hy];
if n == 4
    [U, ~, ~] = svd(h);
else
    [U, ~, ~] = svd(h, 'econ');
end
v = (reshape(U(:,9), 3, 3)).';
end
%}
%%{
function [ H ] = homography_solve(in_pts, out_pts)
    % est_homography estimates the homography to transform each of the
    % in_pts to out_pts
    % Inputs:
    %     in_pts
    %     out_pts
    % Outputs:
    %     H: a 3x3 homography matrix such that outpts ~ H*video_pts

    % Scale the out_pts to prevent problems with small numbers
    %{
    out_mean = mean(out_pts, 1);
    out_pts = out_pts - out_mean(ones(size(out_pts,1),1),:);
    out_scale = max(abs(out_pts(:)));
    out_pts = out_pts ./ out_scale;
    %}
    A = [];

    for p=1:size(in_pts, 1)
        i = in_pts(p,:);
        o = out_pts(p,:);
        a_x = [ -i(1) -i(2) -1 0 0 0 i(1) * o(1) i(2) * o(1) o(1) ];
        a_y = [ 0 0 0 -i(1) -i(2) -1 i(1) * o(2) i(2) * o(2) o(2) ];
        A = [A; a_x; a_y];
    end

    [U, S, V] = svd(A);

    H = V(:, end);

    H = transpose(reshape(H, 3, 3));
    
    % Redo the scaling that we did before
    %{
    S = [out_scale  0           out_mean(1); ...
         0          out_scale   out_mean(2);
         0          0           1];
     
    H = S * H;
    %}
    
    % H33 (Tz) must be positive
    % if it is negative, take the negative of the matrix
    % as H is only known up to a scale
    
    if H(3, 3) < 0
        H  = -1 * H;
    end
end
%%}