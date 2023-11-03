function [x_est_set, Pk_set, y_pre_set, Wk_set] = KalmanFilter(t_set, y_mes_set, x_est_set, Pk_set, y_pre_set, Wk_set, Qw, BodyParameter, FrameParameter, IMUParameter)
    
    n = floor(size(x_est_set, 1) / 3); % numbers of degrees of freedom
    m = size(y_mes_set, 1); % dimension of measurement
    t_kalman_step = t_set(end) - t_set(end - 1); % time step
    F = get_StateTransmissionMatrix(n, t_kalman_step);
    Q = get_ProcessErrorCovariance(Qw, t_kalman_step);

    xk_1_est = x_est_set(:, end);%wen ti shi x_est_set
    Pk_1 = Pk_set(:, :, end);
    
    % Prior or Prediction
    xk_pre = F * xk_1_est;
    qk_pre = xk_pre(1:n);
    dqk_pre = xk_pre((n + 1):(2 * n));
    ddqk_pre = xk_pre((2 * n + 1):(3 * n));
    
    % Output Prediction
    Rk = get_IMU_NoiseCovariance(IMUParameter);
    [yk_pre, Hk, Vk] = get_System_IMUMeasurement(qk_pre, dqk_pre, ddqk_pre, BodyParameter, FrameParameter, IMUParameter, zeros(m, 1));
    yk_mes = y_mes_set(:, end);%y_mes_set bu dui
    Pk_k_1 = F * Pk_1 * F' + Q;
    Wk = Hk * Pk_k_1 * Hk' + Vk * Rk * Vk';
    Kk = Pk_k_1 * Hk' / Wk;
    xk_est = xk_pre + Kk * (yk_mes - yk_pre);%zhe liyk_mesbudui
    Pk = (eye(3 * n) - Kk * Hk) * Pk_k_1;

    x_est_set = [x_est_set, xk_est];
    Pk_set = cat(3, Pk_set, Pk);
    y_pre_set = [y_pre_set, yk_pre];
    Wk_set = cat(3, Wk_set, Wk);
end