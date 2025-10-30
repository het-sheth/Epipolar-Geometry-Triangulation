V1_raw = load('Parameters_V1_1.mat');
V2_raw = load('Parameters_V2_1.mat');
MoCap = load('mocapPoints3D.mat');

V1 = V1_raw.Parameters;
V2 = V2_raw.Parameters;

R1 = double(V1.Rmat); % load relevant computational parameters inside function
P1 = double(V1.Pmat); % Pmat is not the real Pmat, rather the E or F matrix, E in this context because we know the intrinsic parameters
Point1 = double(V1.position);

R2 = double(V2.Rmat); % most importantly, convert information from structural to double for proper manipulation of data
P2 = double(V2.Pmat);
Point2 = double(V2.position);

t1 = - R1 * Point1';
t2 = - R2 * Point2';

P_exp_1 = [R1, t1];
P_exp_2 = [R2, t2];

P_result_1 = P_exp_1 - P1;
P_result_2 = P_exp_2 - P2;

fprintf('Camera 1 - Maximum Pmat difference: %.10e\n', max(abs(P_result_1(:))));
fprintf('Camera 2 - Maximum Pmat difference: %.10e\n', max(abs(P_result_2(:))));