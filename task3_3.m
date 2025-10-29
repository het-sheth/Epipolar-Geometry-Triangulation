% --- task3_3.m ---
%
% Task 3.3: Triangulation to recover 3D mocap points from two views
%
fprintf('Running Task 3.3...\n');

% --- 1. Load All Necessary Data ---
% Load camera parameters for both views
S1 = load('Parameters_V1_1.mat'); % Loads a struct S1
S2 = load('Parameters_V2_1.mat'); % Loads a struct S2

% Load the 2D projected points (from your Task 3.2)
% We load these into structs to get the correct variable names
S_pts1 = load('mocapPoints2D_cam_1.mat'); % Loads 'projectedPoints2D_1'
S_pts2 = load('mocapPoints2D_cam_2.mat'); % Loads 'projectedPoints2D_2'

% Load the original "ground truth" 3D points
S3 = load('mocapPoints3D.mat'); % Loads 'pts3D'

% Get parameters for Camera 1
K1 = S1.Parameters.Kmat;
R1 = S1.Parameters.Rmat;
C1 = S1.Parameters.position'; % This is the camera center

% Get parameters for Camera 2
K2 = S2.Parameters.Kmat;
R2 = S2.Parameters.Rmat;
C2 = S2.Parameters.position';

% Get the 3D points (and transpose from 3x39 to 39x3)
mocapPoints3D_transposed = S3.pts3D';
num_points = size(mocapPoints3D_transposed, 1); % Should be 39

% Get the 2D points (which are 2x39)
pts2D_cam1 = S_pts1.projectedPoints2D_1;
pts2D_cam2 = S_pts2.projectedPoints2D_2;

% --- 2. Triangulate Points ---
% Create an array to store our reconstructed 3D points
reconstructedPoints3D = zeros(num_points, 3);

for i = 1:num_points
    % Get the 2D pixel coordinates for this point
    % Data is 2x39, so we access the i-th COLUMN (:, i)
    x1_pixel = [pts2D_cam1(:, i); 1]; % Homogeneous 2D point for cam 1
    x2_pixel = [pts2D_cam2(:, i); 1]; % Homogeneous 2D point for cam 2

    % Compute the 3D viewing ray for Camera 1
    v1_cam = K1 \ x1_pixel;
    v1_world = R1' * v1_cam;
    v1_world = v1_world / norm(v1_world);
    
    % Compute the 3D viewing ray for Camera 2
    v2_cam = K2 \ x2_pixel;
    v2_world = R2' * v2_cam;
    v2_world = v2_world / norm(v2_world);

    % Now we have two 3D rays (as per lecture):
    % Ray 1: P1(t) = C1 + t * v1_world
    % Ray 2: P2(s) = C2 + s * v2_world
    
    % Find the point of closest approach between these two lines
    w = C1 - C2;
    
    % Build the 2x2 matrix A
    a11 = dot(v1_world, v1_world);
    a12 = -dot(v1_world, v2_world);
    a21 = dot(v1_world, v2_world);
    a22 = -dot(v2_world, v2_world);
    A = [a11, a12; a21, a22];
    
    % Build the 2x1 vector b
    b1 = -dot(w, v1_world);
    b2 = -dot(w, v2_world);
    b = [b1; b2];
    
    % Solve for [t; s]
    params = A \ b;
    t = params(1);
    s = params(2);
    
    % Find the 3D points on each ray
    P1_on_ray = C1 + t * v1_world;
    P2_on_ray = C2 + s * v2_world;
    
    % The triangulated point is the midpoint of the shortest segment
    reconstructedPoints3D(i, :) = (P1_on_ray + P2_on_ray)' / 2;
end

% --- 3. Compute Quantitative Error ---
% Calculate the Mean Squared Error (MSE) 

% Calculate the squared distances
% We must subtract the 39x3 'reconstructedPoints3D'
% from the 39x3 'mocapPoints3D_transposed'
diff = reconstructedPoints3D - mocapPoints3D_transposed;
squared_distances = sum(diff.^2, 2);
mse = mean(squared_distances);

% Display the result
fprintf('Triangulation complete.\n');
fprintf('Mean Squared Error (MSE) between original and reconstructed 3D points: %f\n', mse);
fprintf('This error should be very small.\n');