function [x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter_Initialization(x0, theta0, t_kalman_step, Qw)%duide
    
    [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta0);
    n = length(BodyParameter); 
    m = 6 * length(IMUParameter);
    Q = get_ProcessErrorCovariance(Qw, t_kalman_step); % process noise
    R = get_IMU_NoiseCovariance(IMUParameter); % measurement noise
    [yk_pre, Hk, Vk] = get_System_IMUMeasurement(x0(1:n), x0((n + 1):(2 * n)), x0((2 * n + 1):(3 * n)), BodyParameter, FrameParameter, IMUParameter, zeros(m, 1));
    
    x_est_set = x0; 
    Pk_k_1 = 10 * Q; 
    Pk_set = reshape(Pk_k_1, [3 * n, 3 * n, 1]);
    y_pre_set = yk_pre; 
    Wk = Hk * Pk_k_1 * Hk' + Vk * R * Vk'; 
    Wk_set = reshape(Wk, [m, m, 1]);
end