clc;
clear;
                                    %syms tvalue t
close all ;                         %arxiv参考一下怎么写，template模板cope过来改一下
x_est_0=0.1*[1;1;1;1;1;1];
x_est_current = x_est_0;            % 在overleaf上写一个text 一个kalmanfilter的流程，一边写一边转成code，一一对应
for t=1:0.1:2
    x_est_previous = x_est_current;
                                    % 1 obtain measurement
                                    %     if use_real_sensor
                                    %         y_current = load_IMU;%代入计算
                                    %     elseif use_simulated_sensor
    [y_current,x_exact] = FD(t);    %暂时没有imu 用这个模拟imu
                                    %     end
                                    % 2 Estimation x
    x_est_current = KF(x_est_previous, y_current);
end
fprintf('x_exact=%d\n',x_exact);
fprintf('x_est_current=%d\n',x_est_current);
                                    %x_exact，x_est_current 












