function Q = get_ProcessErrorCovariance(Qw, t_kalman_step)%duide
    % get the process noise covariance, which depends on the covariance of jerk and time step
    % Input:    covariance of jerk - Qw
    %           time step - t_kalman_step
    n = size(Qw, 1); % numbers of degrees of freedoms

    Q = [(1/20) * (t_kalman_step^5) * eye(n), (1/8) * (t_kalman_step^4) * eye(n), (1/6) * (t_kalman_step^3) * eye(n);
         (1/8) * (t_kalman_step^4) * eye(n), (1/3) * (t_kalman_step^3) * eye(n), (1/2) * (t_kalman_step^2) * eye(n);
         (1/6) * (t_kalman_step^3) * eye(n), (1/2) * (t_kalman_step^2) * eye(n), t_kalman_step * eye(n)];

    Q = Q * kron(eye(3), Qw);
end



