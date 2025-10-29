
function F = task3_5(do_visualize)

if nargin < 1, do_visualize = true; end

% Load camera parameters
S1 = load('Parameters_V1_1.mat'); 
S2 = load('Parameters_V2_1.mat');
P1 = S1.Parameters;
P2 = S2.Parameters;

K1 = P1.Kmat;   K2 = P2.Kmat;
R1 = P1.Rmat;   R2 = P2.Rmat;
C1 = P1.position(:);  C2 = P2.position(:);

% Relative pose (cam1 -> cam2)
R = R2 * R1';            % rotation bringing cam1 coords into cam2 coords
t = R2 * (C1 - C2);      % translation of cam1 origin, expressed in cam2

% Essential & Fundamental
E = skew_local(t) * R;
F = (inv(K2))' * E * inv(K1);   % F = K2^{-T} * E * K1^{-1}

% Enforce rank-2
[U,S,V] = svd(F);
S(3,3) = 0;
F = U * S * V';

% Save for report
save('Task3_5_F.mat','F');
try
    writematrix(F,'Task3_5_F.txt','Delimiter','\t');
catch
    writematrix(F,'Task3_5_F.txt','Delimiter','\t','FileType','text');
end

% Optional visual sanity check
if do_visualize
    M = load('mocapPoints3D.mat');
    if isfield(M,'pts3D')
        X = M.pts3D;
    else
        error('mocapPoints3D.mat must contain variable "pts3D".');
    end
    if size(X,1) == 3, X = X'; end  % 3xN -> Nx3
    
    % Build projection matrices from K, R, C
    Pmat1 = K1 * [R1, -R1*C1];
    Pmat2 = K2 * [R2, -R2*C2];
    
    % Project 3D to each view
    x1 = projectPoints_local(Pmat1, X);
    x2 = projectPoints_local(Pmat2, X);
    
    I1 = imread('im1corrected.jpg');
    I2 = imread('im2corrected.jpg');
    
    figure; imshow(I2); hold on;
    title('Epipolar lines in Image 2 (from points in Image 1)');
    drawEpilines_local(F, x1, size(I2));
    
    figure; imshow(I1); hold on;
    title('Epipolar lines in Image 1 (from points in Image 2)');
    drawEpilines_local(F', x2, size(I1));
end
end

% local helpers 
function S = skew_local(v)
v = v(:);
S = [   0   -v(3)  v(2);
      v(3)    0   -v(1);
     -v(2)  v(1)    0 ];
end

function x = projectPoints_local(P, X)
% X: Nx3 world points
Xh = [X, ones(size(X,1),1)];
xh = (P * Xh')';
x  = xh(:,1:2) ./ xh(:,3);
end

function drawEpilines_local(F, pts, imsz)
w = imsz(2);
idx = unique(round(linspace(1,size(pts,1),12)));
for i = idx
    p = [pts(i,:) 1]';
    l = F * p;  a = l(1); b = l(2); c = l(3);
    if abs(b) < eps, continue; end
    y0 = -(c + a*1)/b;  y1 = -(c + a*(w-1))/b;
    plot([1, w-1], [y0, y1], 'LineWidth', 1.5);
    plot(pts(i,1), pts(i,2), 'yo', 'MarkerSize', 6, 'MarkerFaceColor', 'y');
end
end
