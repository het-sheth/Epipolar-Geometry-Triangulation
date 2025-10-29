    % --- task3_4.m ---
    %
    % Task 3.4: Triangulation to make measurements about the scene
    %
    fprintf('Running Task 3.4...\n');
    fprintf('This script is INTERACTIVE. Please click on the images when prompted.\n');

    % --- 1. Load Data & Apply All Fixes ---
    S1 = load('Parameters_V1_1.mat'); % Loads a struct S1
    S2 = load('Parameters_V2_1.mat'); % Loads a struct S2
    im1 = imread('im1corrected.jpg');
    im2 = imread('im2corrected.jpg');

    % Get camera parameters (with .position and TRANSPOSE fixes)
    K1 = S1.Parameters.Kmat; 
    R1 = S1.Parameters.Rmat; 
    C1 = S1.Parameters.position'; % 3x1 column vector
            
    K2 = S2.Parameters.Kmat; 
    R2 = S2.Parameters.Rmat; 
    C2 = S2.Parameters.position'; % 3x1 column vector

    % --- 2. Measure Floor Plane ---
    fprintf('\n--- Measuring Floor Plane ---\n');
    fprintf('Click 3 points on the FLOOR in Image 1...\n');
    figure(1); imshow(im1); title('Click 3 points on the FLOOR');
    [x1_floor, y1_floor] = ginput(3); % Get 3 points
    p1_floor = [x1_floor, y1_floor];  % 3x2 matrix

    fprintf('Click the SAME 3 points on the FLOOR in Image 2...\n');
    figure(2); imshow(im2); title('Click the SAME 3 points on the FLOOR');
    [x2_floor, y2_floor] = ginput(3); % Get 3 points
    p2_floor = [x2_floor, y2_floor];  % 3x2 matrix
            
    close(1);
    close(2);

    % Triangulate these points
    floor_points_3D = zeros(3, 3);
    for i = 1:3
        % Pass 1x2 row vectors [x, y]
        floor_points_3D(i, :) = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_floor(i, :), p2_floor(i, :));
    end
    fprintf('3D Floor Points:\n');
    disp(floor_points_3D);

    % Fit a plane (aX + bY + cZ + d = 0) to these 3 points
    % [X Y Z 1] * [a; b; c; d] = 0
    A_floor = [floor_points_3D, ones(3, 1)];
    [~, ~, V_floor] = svd(A_floor);
    plane_floor = V_floor(:, end); % [a; b; c; d]

    % Normalize so the Z component (c) is 1.0 for easier reading
    % This verifies that the plane is (roughly) Z=0
    if plane_floor(3) ~= 0
        plane_floor = plane_floor / plane_floor(3);
    end

    fprintf('Floor Plane Equation (aX+bY+Z+d=0):\n a=%.3f, b=%.3f, c=1.000, d=%.3f\n', ...
            plane_floor(1), plane_floor(2), plane_floor(4));
    fprintf('This should be (roughly) Z=0 (i.e., a, b, and d should be small)\n');


    % --- 3. Measure Wall Plane ---
    fprintf('\n--- Measuring Striped Wall Plane ---\n');
    fprintf('Click 3 points on the STRIPED WALL in Image 1...\n');
    figure(1); imshow(im1); title('Click 3 points on the STRIPED WALL');
    [x1_wall, y1_wall] = ginput(3);
    p1_wall = [x1_wall, y1_wall];

    fprintf('Click the SAME 3 points on the STRIPED WALL in Image 2...\n');
    figure(2); imshow(im2); title('Click the SAME 3 points on the STRIPED WALL');
    [x2_wall, y2_wall] = ginput(3);
    p2_wall = [x2_wall, y2_wall];
            
    close(1);
    close(2);

    % Triangulate these 3 points
    wall_points_3D = zeros(3, 3);
    for i = 1:3
        wall_points_3D(i, :) = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_wall(i, :), p2_wall(i, :));
    end
    fprintf('3D Wall Points:\n');
    disp(wall_points_3D);

    % Fit a plane
    A_wall = [wall_points_3D, ones(3, 1)];
    [~, ~, V_wall] = svd(A_wall);
    plane_wall = V_wall(:, end); % [a; b; c; d]

    % Normalize by the length of the normal vector [a, b, c]
    norm_vec = norm(plane_wall(1:3));
    if norm_vec ~= 0
        plane_wall = plane_wall / norm_vec;
    end

    fprintf('Wall Plane Equation (aX+bY+cZ+d=0):\n a=%.3f, b=%.3f, c=%.3f, d=%.3f\n', ...
    plane_wall(1), plane_wall(2), plane_wall(3), plane_wall(4));


    % --- 4. Measure Heights and Locations ---

    % Doorway Height
    fprintf('\n--- Measuring Doorway Height ---\n');
    figure(1); imshow(im1); title('Click BOTTOM of doorway (on floor)');
    [p1_door_bot(1), p1_door_bot(2)] = ginput(1);
    figure(2); imshow(im2); title('Click BOTTOM of doorway (on floor)');
    [p2_door_bot(1), p2_door_bot(2)] = ginput(1);
    P_door_bot = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_door_bot, p2_door_bot);

    figure(1); imshow(im1); title('Click TOP of doorway');
    [p1_door_top(1), p1_door_top(2)] = ginput(1);
    figure(2); imshow(im2); title('Click TOP of doorway');
    [p2_door_top(1), p2_door_top(2)] = ginput(1);
    P_door_top = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_door_top, p2_door_top);

    door_height = P_door_top(3) - P_door_bot(3);
    fprintf('Doorway Height (Z_top - Z_bottom): %.1f mm\n', door_height);

    % Person's Height
    fprintf('\n--- Measuring Person Height ---\n');
    figure(1); imshow(im1); title('Click BOTTOM of person (foot on floor)');
    [p1_p_bot(1), p1_p_bot(2)] = ginput(1);
    figure(2); imshow(im2); title('Click BOTTOM of person (foot on floor)');
    [p2_p_bot(1), p2_p_bot(2)] = ginput(1);
    P_p_bot = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_p_bot, p2_p_bot);

    figure(1); imshow(im1); title('Click TOP of person''s head');
    [p1_p_top(1), p1_p_top(2)] = ginput(1);
    figure(2); imshow(im2); title('Click TOP of person''s head');
    [p2_p_top(1), p2_p_top(2)] = ginput(1);
    P_p_top = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_p_top, p2_p_top);

    person_height = P_p_top(3) - P_p_bot(3);
    fprintf('Person Height (Z_top - Z_bottom): %.1f mm\n', person_height);

    % Tripod Camera Location
    fprintf('\n--- Measuring Tripod Location ---\n');
    figure(1); imshow(im1); title('Click center of tripod camera');
    [p1_tripod(1), p1_tripod(2)] = ginput(1);
    figure(2); imshow(im2); title('Click center of tripod camera');
    [p2_tripod(1), p2_tripod(2)] = ginput(1);
    P_tripod = triangulatePoint(K1, R1, C1, K2, R2, C2, p1_tripod, p2_tripod);

    fprintf('3D Location of Tripod Camera (X,Y,Z): [%.1f, %.1f, %.1f]\n', ...
    P_tripod(1), P_tripod(2), P_tripod(3));
            
    close(1);
    close(2);
    fprintf('\nTask 3.4 Complete.\n');