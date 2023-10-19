function [BodyParameter, IMUParameter, FrameParameter] = get_SystemParameter(theta)
    %BodyParameter =[ BodyParameterClass(0,1, 'y', 1),BodyParameterClass(0,1, 'y', 1)];
    BodyParameter(1) = BodyParameterClass(0, 1, 'y', 1);
    BodyParameter(1).set_Joint_Parameter([0, 0, 0], [0, 0, 0]);
    BodyParameter(1).set_Joint_Parameter([0.2, 0, 0], [0, 0, 0]);

    BodyParameter(2) = BodyParameterClass(1, 2, 'y', 2);
    BodyParameter(2).set_Joint_Parameter([0, 0, 0], [0, 0, 0]);
    BodyParameter(2).set_Joint_Parameter([0.2, 0, 0], [0, 0, 0]);

    % IMU Parameter
    Ra = 0.005;
    Rw = deg2rad(0.002);
    IMUParameter(1) = IMUParameterClass(1, [0.1, 0, 0.05], [0, 180, 0], Ra, Rw);
    IMUParameter(2) = IMUParameterClass(2, [0.1, 0, 0.05], [0, 180, 0], Ra, Rw);

    % Frame Parameter
    FrameParameter = FrameParameterClass();
    FrameParameter.set_Joint_Parameter([0, 0, 0], [0, 0, 0]);

    % theta
    BodyParameter(1).Joint(2).r = BodyParameter(1).Joint(2).r + theta(:);
end