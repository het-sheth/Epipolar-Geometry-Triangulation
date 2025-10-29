% --- extra_credit_2.m ---
%
% Extra Credit 2: Generate a top-down view of the floor plane
%
fprintf('Running Extra Credit 2...\n');

% --- 1. Load Data & Apply Fixes ---
S1 = load('Parameters_V1_1.mat'); % Load struct
im_source = imread('im1corrected.jpg');

% Get Kmat and Pmat (which is [R|t])
Kmat = S1.Parameters.Kmat;
Pmat_extrinsic = S1.Parameters.Pmat;

[im_h, im_w, ~] = size(im_source);

% --- 2. Compute Homography ---
% *** THIS IS THE FIX ***
% First, calculate the FULL projection matrix
P_full = Kmat * Pmat_extrinsic;

% Now, calculate the homography from the FULL matrix [cite: 171]
% H_3D = P_full * [X; Y; 0; 1] = P_full(:, [1, 2, 4]) * [X; Y; 1]
H_3D = P_full(:, [1, 2, 4]);

% We will use "inverse mapping". We loop through our output image,
% which represents the (X,Y) world plane, and for each (X,Y) coord,
% we use H_3D to find where it is in the source image.

% --- 3. Define Output View ---
% Define the 3D world coordinates (in mm) we want to see [cite: 84]
x_world_range = -4000:20:4000; % -4m to +4m
y_world_range = -3000:20:3000; % -3m to +3m

% Output image dimensions
out_h = length(y_world_range);
out_w = length(x_world_range);
im_top_down = uint8(zeros(out_h, out_w, 3));

% Create 2D grids of all (X, Y) world coordinates
[X_grid, Y_grid] = meshgrid(x_world_range, y_world_range);

% --- 4. Perform Inverse Warp ---
% Create a [3 x (W*H)] matrix of homogeneous world points
num_out_pixels = out_h * out_w;
world_points_hom = [X_grid(:)'; Y_grid(:)'; ones(1, num_out_pixels)];

% 2. Apply H_3D to all points at once: image_points = H_3D * world_points
image_points_hom = H_3D * world_points_hom;

% 3. Convert from homogeneous to 2D pixel coordinates
%    u = x/w, v = y/w
u = image_points_hom(1, :) ./ image_points_hom(3, :);
v = image_points_hom(2, :) ./ image_points_hom(3, :);

% 4. Sample colors from the source image
% We can use interp2 to do this quickly.
R_source = double(im_source(:,:,1));
G_source = double(im_source(:,:,2));
B_source = double(im_source(:,:,3));

% Use 'linear' interpolation and set 'extrapval' to 0 (black)
% for points that are outside the original image
R_warped = interp2(R_source, u, v, 'linear', 0);
G_warped = interp2(G_source, u, v, 'linear', 0);
B_warped = interp2(B_source, u, v, 'linear', 0);

% 5. Reshape the 1D color vectors back into images
im_top_down(:,:,1) = uint8(reshape(R_warped, out_h, out_w));
im_top_down(:,:,2) = uint8(reshape(G_warped, out_h, out_w));
im_top_down(:,:,3) = uint8(reshape(B_warped, out_h, out_w));

% --- 5. Display Result ---
figure(10);
imshow(im_top_down);
title('Extra Credit 2: Top-Down View of Z=0 Floor Plane');
axis on; % Show axes
xlabel('X (world, mm)');
ylabel('Y (world, mm)');

% Set the axis labels to match the world coordinates
x_ticks = 1:round(out_w/5):out_w;
y_ticks = 1:round(out_h/5):out_h;
set(gca, 'XTick', x_ticks, 'XTickLabel', x_world_range(x_ticks));
set(gca, 'YTick', y_ticks, 'YTickLabel', y_world_range(y_ticks));

fprintf('Top-down view generated.\n');
fprintf('In the report, explain what looks accurate (e.g., floor, person position) and what looks weird (e.g., things not on the floor are distorted)[cite: 172].\n');
fprintf('This could be useful for analyzing a person''s motion path, stance, or interaction with the 2D floor space[cite: 173].\n');