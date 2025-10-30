function [projectedPoints2D_1, projectedPoints2D_2] = project_points_function(V1, V2, MoCap)

    K1 = double(V1.Kmat); % load relevant computational parameters inside function
    P1 = double(V1.Pmat); % Pmat is not the real Pmat, rather the E or F matrix, E in this context because we know the intrinsic parameters

    K2 = double(V2.Kmat); % most importantly, convert information from structural to double for proper manipulation of data
    P2 = double(V2.Pmat);

    Pres1 = K1 * P1; % Resolved P function including intrinsic and extrinsic parameters
    Pres2 = K2 * P2;

    MoCap = double(MoCap.pts3D); % double conversion for mocap points from structural

    N = size(MoCap, 2); % manipulates the size of the motion capture points so that we can check their validity with our algorithm
    MoCap_world = [MoCap; ones(1, N)]; 

    MoCap_pixel_1 = Pres1 * MoCap_world; % convert motion capture coordinates from world to pixel coordinates
    MoCap_pixel_2 = Pres2 * MoCap_world;

    w_prime_1 = MoCap_pixel_1(3, :); % scaling factor with respect to converting 3D to 2D
    w_prime_2 = MoCap_pixel_2(3, :);

    u_1 = MoCap_pixel_1(1, :) ./ w_prime_1;
    v_1 = MoCap_pixel_1(2, :) ./ w_prime_1; % normalizing dimensions of pixel coordinates

    u_2 = MoCap_pixel_2(1, :) ./ w_prime_2;
    v_2 = MoCap_pixel_2(2, :) ./ w_prime_2; 

    projectedPoints2D_1 = [u_1; v_1]; % final product of points calculated to projection from 3D to the 2D corrected images we have
    projectedPoints2D_2 = [u_2; v_2];

end
