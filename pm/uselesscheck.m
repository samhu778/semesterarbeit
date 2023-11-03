clc
theta_true = [0.05; 0.02; 0.03];
q0 = deg2rad([0;0]);
qe = deg2rad([45; 90]);
dq0 = zeros(size(q0));
zero_matrix = zeros(size(q0));
x0 = cat(1, cat(1, q0, zero_matrix), zero_matrix);
alpha = 1;
theta0= [0;0;0];
theta= [0;0;0];
MaxTol = 2*10^(-4) ;
N_max = 80;
[t_set, y_mes_set] = SignalPreparation(theta_true);%y_mes_set you wen ti 
Q_epsilon= [0.5,0;0,0.5];%Qw=[0.5,0;0,0.5];
 t_kalman_step=0.01;
[x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter_Initialization(x0, theta0, t_kalman_step, Q_epsilon);
%this is true
theta_set = reshape(theta0, [], 1);
for tNr = 2:length(t_set)
    theta = theta_set(:, end);
    [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta); 
    [x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter(t_set(1:tNr), y_mes_set(:, 1:tNr), x_est_set, Pk_set, y_pre_set, Wk_set, Q_epsilon, BodyParameter, FrameParameter, IMUParameter);
%function [x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter(t_set, (((((y_mes_set))))), x_est_set, Pk_set, y_pre_set, Wk_set, Qw, BodyParameter, FrameParameter, IMUParameter)
end


% t_kalman_step=0.01;

% theta0=[0;0;0];
%Qw t kalman from py
% x0= [0;0;0;0;0;0];
% 
% t_kalman_step= 0.01;
% 
% [Q] = get_ProcessErrorCovariance(Qw, t_kalman_step);
%[R] = get_IMU_NoiseCovariance(IMUParameter);
% n=2;
% [F] = get_StateTransmissionMatrix(n, t_kalman_step);
% 
% 
% [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta);
% 
% 
% theta_true = [0.05;0.02; 0.03];
% 
% 
% [x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter(t_set, y_mes_set, x_est_set, Pk_set, y_pre_set, ...
%     Wk_set, Qw, BodyParameter, FrameParameter, IMUParameter);
