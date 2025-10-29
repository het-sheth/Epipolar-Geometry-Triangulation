function extra1_crop_and_F

%% Load images
I1 = imread('im1corrected.jpg');
I2 = imread('im2corrected.jpg');
[h1,w1,~] = size(I1);
[h2,w2,~] = size(I2);

%% Load camera params and mocap points
S1 = load('Parameters_V1_1.mat'); S2 = load('Parameters_V2_1.mat');
P1 = S1.Parameters;  P2 = S2.Parameters;
K1 = P1.Kmat;  K2 = P2.Kmat;
R1 = P1.Rmat;  R2 = P2.Rmat;
C1 = P1.position(:); C2 = P2.position(:);

M = load('mocapPoints3D.mat');
X = M.pts3D; if size(X,1)==3, X = X.'; end   % make Nx3

%% Project to each image
Pmat1 = K1 * [R1, -R1*C1];
Pmat2 = K2 * [R2, -R2*C2];
x1 = proj_local(Pmat1, X);
x2 = proj_local(Pmat2, X);

%% Compute tight crops around the person (mocap bbox) with padding
pad1 = 40; pad2 = 40;   % pixels
bbox1 = [ floor(min(x1(:,1))-pad1), floor(min(x1(:,2))-pad1), ...
          ceil(max(x1(:,1))+pad1),  ceil(max(x1(:,2))+pad1) ];
bbox2 = [ floor(min(x2(:,1))-pad2), floor(min(x2(:,2))-pad2), ...
          ceil(max(x2(:,1))+pad2),  ceil(max(x2(:,2))+pad2) ];

% convert to [x y w h] and clip to image bounds
crop1 = clampRect([bbox1(1), bbox1(2), bbox1(3)-bbox1(1)+1, bbox1(4)-bbox1(2)+1], w1, h1);
crop2 = clampRect([bbox2(1), bbox2(2), bbox2(3)-bbox2(1)+1, bbox2(4)-bbox2(2)+1], w2, h2);

fprintf('EC1 using crops:\n  crop1 = [%d %d %d %d]\n  crop2 = [%d %d %d %d]\n', crop1, crop2);

%% Call the core (scale = 1, with visualization)
[Fprime, K1p, K2p, H1, H2] = extra1_core(crop1, crop2, 1, 1, true); %#ok<ASGLU>

disp('Saved: Extra1_Fprime.mat and Extra1_Fprime.txt');
end

%  CORE IMPLEMENTATION
function [Fprime, K1p, K2p, H1, H2] = extra1_core(crop1, crop2, scale1, scale2, do_visualize)
% Inputs:
%   crop1 = [x y w h] for image 1
%   crop2 = [x y w h] for image 2
%   scale1, scale2 = isotropic scales applied after crop
%   do_visualize = true/false

if nargin < 3 || isempty(scale1), scale1 = 1; end
if nargin < 4 || isempty(scale2), scale2 = 1; end
if nargin < 5, do_visualize = true; end

% Load camera parameters
S1 = load('Parameters_V1_1.mat'); S2 = load('Parameters_V2_1.mat');
P1 = S1.Parameters;  P2 = S2.Parameters;
K1 = P1.Kmat;  K2 = P2.Kmat;
R1 = P1.Rmat;  R2 = P2.Rmat;
C1 = P1.position(:); C2 = P2.position(:);

% Compute baseline F (uncropped) from calibration
R = R2 * R1';
t = R2 * (C1 - C2);
E = skew_local(t) * R;
F = (inv(K2))' * E * inv(K1);
[U,S,V] = svd(F); S(3,3) = 0; F = U*S*V';

% Build crop+scale homographies H1, H2 that map original pixel coords -> new cropped/scaled coords
H1 = cropScaleHomography(crop1, scale1);  % original -> cropped_scaled (img1)
H2 = cropScaleHomography(crop2, scale2);  % original -> cropped_scaled (img2)

% Update F by similarity transform (correct way under pixel homographies)
Fprime_sim = (H2') \ (F / H1);

% Alternatively, update intrinsics explicitly and recompute from E
[K1p, K2p] = deal(K1, K2);
% First adjust for crop (principal point shift), then scale (focal & principal)
u01 = crop1(1); v01 = crop1(2);
u02 = crop2(1); v02 = crop2(2);

K1p(1,3) = K1(1,3) - u01;
K1p(2,3) = K1(2,3) - v01;
K2p(1,3) = K2(1,3) - u02;
K2p(2,3) = K2(2,3) - v02;

K1p(1,1) = K1p(1,1) * scale1;
K1p(2,2) = K1p(2,2) * scale1;
K1p(1,3) = K1p(1,3) * scale1;
K1p(2,3) = K1p(2,3) * scale1;

K2p(1,1) = K2p(1,1) * scale2;
K2p(2,2) = K2p(2,2) * scale2;
K2p(1,3) = K2p(1,3) * scale2;
K2p(2,3) = K2p(2,3) * scale2;

Fprime_cal = (inv(K2p))' * E * inv(K1p);
[U2,S2,V2] = svd(Fprime_cal); S2(3,3) = 0; Fprime_cal = U2*S2*V2';

% Compare (up to scale) for sanity
alpha = Fprime_sim(1,1) / (Fprime_cal(1,1) + eps);
relerr = abs(norm(Fprime_sim - alpha*Fprime_cal, 'fro')/max(1e-9,norm(Fprime_sim,'fro')));
if relerr > 0.05
    warning('F'' from similarity and from recalibration differ by more than 5%% (up to scale). RelErr=%.3g', relerr);
end
Fprime = Fprime_sim;

% Visualization
if do_visualize
    I1 = imread('im1corrected.jpg'); I2 = imread('im2corrected.jpg');
    I1c = imcrop(I1, [crop1(1) crop1(2) crop1(3)-1 crop1(4)-1]);
    I2c = imcrop(I2, [crop2(1) crop2(2) crop2(3)-1 crop2(4)-1]);
    if scale1 ~= 1, I1c = imresize(I1c, scale1); end
    if scale2 ~= 1, I2c = imresize(I2c, scale2); end

    % Optional overlay of mocap points (projects in original pixels, then mapped by H)
    hasPts = exist('mocapPoints3D.mat','file')==2;
    if hasPts
        M = load('mocapPoints3D.mat'); X = M.pts3D; if size(X,1)==3, X=X'; end
        Pmat1 = K1 * [R1, -R1*C1];
        Pmat2 = K2 * [R2, -R2*C2];
        x1 = proj_local(Pmat1, X); x2 = proj_local(Pmat2, X);
        x1p = applyH(H1, x1); x2p = applyH(H2, x2);
    else
        x1p = []; x2p = [];
    end

    figure; imshow(I2c); title('EC1: Epilines in Cropped Image 2 from Cropped Image 1');
    hold on; drawEpilines_local(Fprime,  x1p, size(I2c));

    figure; imshow(I1c); title('EC1: Epilines in Cropped Image 1 from Cropped Image 2');
    hold on; drawEpilines_local(Fprime', x2p, size(I1c));
end

% Save outputs
save('Extra1_Fprime.mat','Fprime','K1p','K2p','H1','H2','crop1','crop2','scale1','scale2');
try
    writematrix(Fprime,'Extra1_Fprime.txt','Delimiter','\t','FileType','text');
catch
    fid = fopen('Extra1_Fprime.txt','w');
    fprintf(fid, '%.10f\t%.10f\t%.10f\n', Fprime.');
    fclose(fid);
end
end

% HELPERS

function S = skew_local(v)
v = v(:);
S = [  0   -v(3)  v(2);
      v(3)   0   -v(1);
     -v(2)  v(1)   0  ];
end

function H = cropScaleHomography(crop, s)
% crop = [x y w h]; translate by (-x,-y) then scale by s
x = crop(1); y = crop(2);
T = [1 0 -x; 0 1 -y; 0 0 1];
S = [s 0 0; 0 s 0; 0 0 1];
H = S * T; % original -> cropped+scaled
end

function x2 = applyH(H, x)
% x: Nx2, maps via H (right-multiply in homogeneous form)
xh = [x, ones(size(x,1),1)] * H.';
xh = xh ./ xh(:,3);
x2 = xh(:,1:2);
end

function x = proj_local(P, X)
% X: Nx3
Xh = [X, ones(size(X,1),1)];
xh = (P*Xh')';
x  = xh(:,1:2) ./ xh(:,3);
end

function drawEpilines_local(F, pts, imsz)
if isempty(pts), return; end
w = imsz(2);
idx = unique(round(linspace(1,size(pts,1),12)));
for i = idx
    p = [pts(i,:) 1]';
    l = F*p; a=l(1); b=l(2); c=l(3);
    if abs(b) < eps, continue; end
    y0 = -(c + a*1)/b; y1 = -(c + a*(w-1))/b;
    plot([1, w-1], [y0, y1], 'LineWidth', 1.5);
    plot(pts(i,1), pts(i,2), 'yo', 'MarkerSize', 6, 'MarkerFaceColor','y');
end
end

function rect = clampRect(rect, w, h)
x = max(1, rect(1)); y = max(1, rect(2));
W = rect(3); H = rect(4);
if x+W-1 > w, W = w - x + 1; end
if y+H-1 > h, H = h - y + 1; end
rect = [x y W H];
end
