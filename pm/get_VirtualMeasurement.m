function [y_mes_set, q_set, dq_set, ddq_set] = get_VirtualMeasurement(t_set, q0, qe, BodyParameter, FrameParameter, IMUParameter)
    t_start = t_set(1); 
    t_end = t_set(end); 
    BodyQuantity = length(BodyParameter); % number of bodies
    IMU_Quantity = length(IMUParameter); % number of IMUs
    n = BodyQuantity; % number of degrees of freedoms = number of bodies
    m = 6 * IMU_Quantity; % dimension of the output / measurement, with 6 outputs per IMU
    
    % IMU Measurement Covariance Matrix
    R = get_IMU_NoiseCovariance(IMUParameter);
    Measurement_Noise = sqrt(R) * randn(m, 1);
    
    q_set = [];
    dq_set = [];
    ddq_set = [];
    y_mes_set = [];
    
    % For each interested time point
    for tNr = 1:length(t_set)
        t = t_set(tNr);
        
        % The actual state based on predefined trajectory function at time t
        if t <= t_end * 0.8
            [q, dq, ddq] = Polynimial_FirstSecondOrder0_Function(t, t_start, t_end*0.8, q0, qe);
        else
            q = qe;
            dq = zeros(n, 1);
            ddq = zeros(n, 1);
        end
        
        % Measurement and its Jacobian matrix at time t
        [yk_mes, Hk, Vk] = get_System_IMUMeasurement(q, dq, ddq, BodyParameter, FrameParameter, IMUParameter, Measurement_Noise);
        
        % zhe li yk_mes bu dui l  
        if tNr == 1
            q_set = q;
            dq_set = dq;
            ddq_set = ddq;
            y_mes_set = yk_mes;
        else
            q_set = [q_set, q];
            dq_set = [dq_set, dq];
            ddq_set = [ddq_set, ddq];
            y_mes_set = [y_mes_set, yk_mes];
        end
    end
    
    % Return the results
    y_mes_set = y_mes_set';
    q_set = q_set';
    dq_set = dq_set';
    ddq_set = ddq_set';
end