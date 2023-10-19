function [r02, R2, Tr2, dTr2, Tphi2, dTphi2, dddr02dtdt_dq, dddr02dtdt_ddq] = set_IMU_Joint(dq, IMUParameter, Body)
    % Get the kinematics properties of the corresponding body
    r01 = Body.r; % Position of the origin point
    R1 = Body.R; % Rotation matrix
    Tr1 = Body.Tr; % Jacobian matrix between dr/dt and q of the body
    Tphi1 = Body.Tphi; % Jacobian matrix between omega and q of the body
    dTr1 = Body.dTr; % Jacobian matrix between ddr/dtdt and q of the body
    dTphi1 = Body.dTphi; % Jacobian matrix between domega/dt and q of the body
    dddr01dtdt_dq = Body.dddrdtdt_dq; % Jacobian matrix between ddr/dtdt and dq of the body
    dddr01dtdt_ddq = Body.dddrdtdt_ddq; % Jacobian matrix between ddr/dtdt and q of the body
    ddomega1dt_dq = Body.ddomegadt_dq; % Jacobian matrix between ddomega/dtdt and dq of the body
    ddomega1dt_ddq = Body.ddomegadt_ddq; % Jacobian matrix between ddomega/dtdt and q of the body

    % Jacobian vector of qk [1x1] and q [nx1]
    BodyQuantity = size(Tr1, 2); % n
    Talpha = zeros(1, BodyQuantity); % t^T
    dTalpha = zeros(1, BodyQuantity); % dt^T

    % Relative position and pose between the IMU and corresponding body
    r_1_12 = IMUParameter.r; % Relative position
    phi_1_12 = IMUParameter.phi; % Relative pose

    % Get the rotation axis from phi_1_12 [3x1]
    alpha = norm(phi_1_12);
    if alpha ~= 0
        RotationAxis = phi_1_12 / alpha;
    else
        RotationAxis = [1; 0; 0];
    end

    % Get the kinematics properties of the IMU
    [r02, R2, Tr2, Tphi2, dTr2, dTphi2, dddr02dtdt_dq, dddr02dtdt_ddq] = FixedRotation_ForwardKinematics(...
        RotationAxis, alpha, Talpha, dTalpha, r01, R1, Tr1, Tphi1, dTr1, dTphi1, ...
        dddr01dtdt_dq, dddr01dtdt_ddq, ddomega1dt_dq, ddomega1dt_ddq, ...
        dq, zeros(size(dq)), r_1_12);
end