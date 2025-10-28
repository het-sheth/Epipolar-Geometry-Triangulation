V1_raw = load("C:\Users\fwbla\Downloads\Project2DataFiles\Project2DataFiles\Parameters_V1_1.mat");
V2_raw = load("C:\Users\fwbla\Downloads\Project2DataFiles\Project2DataFiles\Parameters_V2_1.mat");
MoCap = load("C:\Users\fwbla\Downloads\Project2DataFiles\Project2DataFiles\mocapPoints3D.mat"); % load all relevant information related to computation

V1 = V1_raw.Parameters;
V2 = V2_raw.Parameters;

im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg'); % read images for later plotting

[projectedPoints2D_1, projectedPoints2D_2] = Task_3_2(V1, V2, MoCap); % call function

figure;
imshow(im1); % Display the image
hold on; % Hold the plot to overlay points

plot(projectedPoints2D_1(1, :), projectedPoints2D_1(2, :), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
title('MoCap Points Projected onto Camera 1');
hold off;

figure;
imshow(im2); % Display the image
hold on;

% Plot the projected points
plot(projectedPoints2D_2(1, :), projectedPoints2D_2(2, :), 'go', 'MarkerSize', 8, 'LineWidth', 2);
title('MoCap Points Projected onto Camera 2');
hold off;