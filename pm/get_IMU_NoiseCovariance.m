function R = get_IMU_NoiseCovariance(IMUParameter)%duide 
    
    IMU_Quantity = length(IMUParameter);
    R = []; % initialize the measurement noise covariance matrix
    for IMU_Nr = 1:IMU_Quantity
        Ra = IMUParameter(IMU_Nr).Ra; % noise covariance matrix for accelerometer for single IMU
        Rw = IMUParameter(IMU_Nr).Rw; % noise covariance matrix for gyroscope for single IMU
        % connect them in block diagonal matrix
        if IMU_Nr == 1
            R = blkdiag(Ra, Rw);
        else
            R = blkdiag(R, Ra, Rw);
        end
    end
    R = R * 1; 
end