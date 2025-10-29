% --- triangulatePoint.m ---
%
% This is a helper function that takes camera parameters and a pair of
% 2D pixel coordinates and returns a single triangulated 3D point.
%
% INPUTS:
%   K1, K2: 3x3 Kmat
%   R1, R2: 3x3 Rmat
%   C1_col, C2_col: 3x1 position vectors (columns)
%   p1_row, p2_row: 1x2 pixel coordinates [x, y]
%
% OUTPUT:
%   P_world: 1x3 triangulated world coordinate [X, Y, Z]
%
function P_world = triangulatePoint(K1, R1, C1_col, K2, R2, C2_col, p1_row, p2_row)
    
    % --- 1. Convert 2D pixel points to 3D viewing rays ---
    x1_pixel = [p1_row'; 1]; % Homogeneous 3x1 vector [x; y; 1]
    x2_pixel = [p2_row'; 1]; % Homogeneous 3x1 vector [x; y; 1]

    % Ray for Camera 1 (in world coords)
    v1_cam = K1 \ x1_pixel;
    v1_world = R1' * v1_cam;
    v1_world = v1_world / norm(v1_world); % 3x1 direction vector
    
    % Ray for Camera 2 (in world coords)
    v2_cam = K2 \ x2_pixel;
    v2_world = R2' * v2_cam;
    v2_world = v2_world / norm(v2_world); % 3x1 direction vector

    % --- 2. Find point of closest approach ---
    w = C1_col - C2_col; % 3x1 vector
    
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
    
    % --- 3. Compute 3D point ---
    % P1 and P2 are 3x1 column vectors
    P1_on_ray = C1_col + t * v1_world;
    P2_on_ray = C2_col + s * v2_world;
    
    % Midpoint is also a 3x1 column vector
    P_midpoint = (P1_on_ray + P2_on_ray) / 2; 
    
    % Transpose to a 1x3 row vector [X, Y, Z] for output
    P_world = P_midpoint'; 
end