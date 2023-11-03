function [t_set, y_mes_set] = SignalPreparation(theta_true)
    q0 = deg2rad([0;0]);
    qe = deg2rad([45; 90]);
    [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta_true);
 
    t_start = 0;
    t_step = 0.01;
    t_end = 1;
    t_set = (t_start:t_step:t_end)';
  
    [y_mes_set, q_set, dq_set, ddq_set] = get_VirtualMeasurement(t_set, q0, qe, BodyParameter, FrameParameter, IMUParameter);
    %y_mes_set you wen ti 
    y_mes_set = y_mes_set';
end