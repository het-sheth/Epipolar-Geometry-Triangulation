% --- Script for Project 2, Task 3.6 ---

% Method eightpoint from eightpoint.m (provided sample code)

clear all;
close all;
clc;

disp('Loading project images...');
im1 = imread('im1corrected.jpg');
im2 = imread('im2corrected.jpg');

disp('Running eightpoint.m');
[F_eightpoint, clicked_pts1, clicked_pts2] = eightpoint(im1, im2);

disp('Task 3.6 complete.');
disp('Fundamental Matrix (F_eightpoint) is:');
disp(F_eightpoint);
