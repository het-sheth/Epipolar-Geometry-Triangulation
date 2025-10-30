    % --- task3_2.m ---
    %
    % This is the DEMO SCRIPT for Task 3.2, as required by the rubric.
    % It calls the project_points_function and plots the results.
    %
    fprintf('Running Task 3.2 Demo Script...\n');

    % --- 1. Load Data ---
    S1 = load('Parameters_V1_1.mat');
    S2 = load('Parameters_V2_1.mat');
    S3 = load('mocapPoints3D.mat');

    % --- 2. Call the Function ---
    % Call the renamed function to get the 2D points
    [pts1, pts2] = project_points_function(S1.Parameters, S2.Parameters, S3);

    % Save the output .mat files for Task 3.3
    projectedPoints2D_1 = pts1;
    projectedPoints2D_2 = pts2;
    save('mocapPoints2D_cam_1.mat', 'projectedPoints2D_1');
    save('mocapPoints2D_cam_2.mat', 'projectedPoints2D_2');

    fprintf('2D points calculated and saved.\n');

    % --- 3. Plot Figure for Camera 1 ---
    fprintf('Generating plot for Camera 1...\n');
    im1 = imread('im1corrected.jpg');

    figure(1);
    imshow(im1);
    hold on;
    plot(projectedPoints2D_1(1, :), projectedPoints2D_1(2, :), 'y+', 'MarkerSize', 10, 'LineWidth', 2);
    title('Task 3.2: Projected 2D Points on Camera 1');
    hold off;

    % Save the figure as a .png for your report
    saveas(figure(1), 'task3_2_plot1.png');
    fprintf('Saved plot as task3_2_plot1.png\n');

    % --- 4. Plot Figure for Camera 2 ---
    fprintf('Generating plot for Camera 2...\n');
    im2 = imread('im2corrected.jpg');

    figure(2);
    imshow(im2);
    hold on;
    plot(projectedPoints2D_2(1, :), projectedPoints2D_2(2, :), 'y+', 'MarkerSize', 10, 'LineWidth', 2);
    title('Task 3.2: Projected 2D Points on Camera 2');
    hold off;

    % Save the figure as a .png for your report
    saveas(figure(2), 'task3_2_plot2.png');
    fprintf('Saved plot as task3_2_plot2.png\n');

    fprintf('Task 3.2 Demo complete. Figures saved.\n');