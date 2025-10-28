% --- Task 3.7 ---
clear all;
close all;
clc;

% --- 1. Load the F-matrices from .mat files ---
disp('Loading F-matrices from .mat files...');

try
    % Load F matrix from Task 3.5
    s_f5 = load('Task3_5_F.mat');
    fields_f5 = fieldnames(s_f5);
    F_3_5 = s_f5.(fields_f5{1}); % Get the first variable in the file
    fprintf('Loaded F_3_5 from Task3_5_F.mat (variable: %s)\n', fields_f5{1});

    % Load F matrix from Task 3.6
    s_f6 = load('Task3_6_F.mat');
    fields_f6 = fieldnames(s_f6);
    F_3_6 = s_f6.(fields_f6{1}); % Get the first variable in the file
    fprintf('Loaded F_3_6 from Task3_6_F.mat (variable: %s)\n', fields_f6{1});

catch e
    disp('ERROR: Could not load the F-matrix .mat files.');
    disp(e.message);
    return;
end
       
% --- 2. Load the 2D point matches from Task 3.2 files ---
disp('Loading 2D point matches from Task 3.2 files...');

try
    % Load each .mat file into a struct
    s1 = load('mocapPoints2D_cam_1.mat');
    s2 = load('mocapPoints2D_cam_2.mat');
    
    % Get the variable names from the loaded structs
    fields1 = fieldnames(s1);
    fields2 = fieldnames(s2);
    
    pts1 = s1.(fields1{1});
    pts2 = s2.(fields2{1});
    
    fprintf('Loaded data from %s (as pts1) and %s (as pts2).\n', fields1{1}, fields2{1});
    
    pts1 = pts1';
    pts2 = pts2';

catch e
    disp('ERROR: Could not load the .mat files.');
    disp(e.message);
    return;
end

% --- 3. Calculate SED for both matrices ---
disp('Calculating SED error for both matrices...');

% 1. SED for F-matrix from calibration (Task 3.5)
sed_3_5 = calculate_sed(F_3_5, pts1, pts2);

% 2. SED for F-matrix from eight-point (Task 3.6)
sed_3_6 = calculate_sed(F_3_6, pts1, pts2);

% --- 4. Results ---
fprintf('\n--- Task 3.7 Results ---\n');
fprintf('Symmetric Epipolar Distance (SED) for F_3_5 (calibration): %f\n', sed_3_5);
fprintf('Symmetric Epipolar Distance (SED) for F_3_6 (eight-point): %f\n', sed_3_6);
fprintf('\n');

if sed_3_5 < sed_3_6
    fprintf('SUCCESS: The error for the F matrix from calibration (%f)\n', sed_3_5);
    fprintf('is smaller than the error for the 8-point F matrix (%f).\n', sed_3_6);
else
    fprintf('WARNING: The error for the 8-point F matrix (%f)\n', sed_3_6);
    fprintf('is smaller than the error for the calibration F matrix (%f).\n', sed_3_5);
end

function sed = calculate_sed(F, pts1, pts2)
% This function calculates the Symmetric Epipolar Distance (SED)
% F: The Fundamental Matrix to test
% pts1: The 39x2 matrix of 2D points from Image 1
% pts2: The 39x2 matrix of 2D points from Image 2

    num_points = size(pts1, 1);
    total_sq_dist = 0;

    % Convert points to homogeneous coordinates
    p1_h = [pts1, ones(num_points, 1)]; % Should be 39x3
    p2_h = [pts2, ones(num_points, 1)]; % Should be 39x3

    for i = 1:num_points
        p1 = p1_h(i, :)'; % [x1; y1; 1] (3x1 vector)
        p2 = p2_h(i, :)'; % [x2; y2; 1] (3x1 vector)

        % 1. Map p1 to line l2 in Image 2
        % l2 = [a; b; c] for line ax+by+c=0
        l2 = F * p1; 
        a = l2(1);
        b = l2(2);
        c = l2(3);
        
        % Calculate squared distance from p2 to l2
        % Formula from project: (ax+by+c)^2 / (a^2+b^2)
        dist_p2_to_l2_sq = (a*p2(1) + b*p2(2) + c)^2 / (a^2 + b^2);

        % 2. Map p2 to line l1 in Image 1
        l1 = F' * p2; % l1 = [a; b; c]
        a = l1(1);
        b = l1(2);
        c = l1(3);
        
        % Calculate squared distance from p1 to l1
        dist_p1_to_l1_sq = (a*p1(1) + b*p1(2) + c)^2 / (a^2 + b^2);

        % 3. Accumulate all squared distances
        total_sq_dist = total_sq_dist + dist_p2_to_l2_sq + dist_p1_to_l1_sq;
    end

    % SED is the mean of all these squared distances
    sed = total_sq_dist / (2 * num_points);

end