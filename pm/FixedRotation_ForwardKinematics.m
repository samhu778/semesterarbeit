function [r02, R2, Tr2, Tphi2, dTr2, dTphi2, dddr02dtdt_dq, dddr02dtdt_ddq, ddomega2dt_dq, ddomega2dt_ddq, dr02dt, omega2, ddr02dtdt, domega2dt] = FixedRotation_ForwardKinematics(RotationAxis, alpha, Talpha, dTalpha, r01, R1, Tr1, Tphi1, dTr1, dTphi1, dddr01dtdt_dq, dddr01dtdt_ddq, ddomega1dt_dq, ddomega1dt_ddq, dq, ddq, r_1_12)
    gi = RotationAxis;
    if strcmp(RotationAxis, 'x')
        gi = [1; 0; 0];
    end
    if strcmp(RotationAxis, 'y')
        gi = [0; 1; 0];
    end
    if strcmp(RotationAxis, 'z')
        gi = [0; 0; 1];
    end
    
    % Position and Posture
    Ri = get_R(gi * alpha); % joint0相对frame的转移矩阵
    trans_Ri = Ri';
    R2 = R1 * Ri;
    r02 = r01 + R1 * r_1_12; % 惯性坐标系上joint0的位置，绝对位置
    
    % Translational and Angular Velocity
    skew_r_1_12 = skew(r_1_12);
    Tr2 = Tr1 - R1 * skew_r_1_12 * Tphi1;
    Tphi2 = trans_Ri * Tphi1 + gi * Talpha;
    
    dr02dt = Tr2 * dq; % joint的线速度
    omega2 = Tphi2 * dq; % joint的角速度
    skew_omega2 = skew(omega2);
    
    % Translational and Angular Acceleration
    omega1 = Tr1 * dq;
    skew_omega1 = skew(omega1);
    dTr2 = dTr1 - R1 * skew_omega1 * skew_r_1_12 * Tphi1 - R1 * skew_r_1_12 * dTphi1;
    dTphi2 = trans_Ri * dTphi1 + skew_omega2 * gi * Talpha + gi * dTalpha;
    
    ddr02dtdt = Tr2 * ddq + dTr2 * dq;
    domega2dt = Tphi2 * ddq + dTphi2 * dq;
    
    dalphadt = Talpha * dq;
    skew_gi = skew(gi);
    
    ddomega2dt_ddq = trans_Ri * ddomega1dt_ddq + trans_Ri * skew_omega1 * Tphi1 - skew_omega2 * trans_Ri * Tphi1 - dalphadt * skew_gi * trans_Ri * Tphi1 + skew_omega2 * gi * Talpha;
    
    dddr02dtdt_ddq = dddr01dtdt_ddq - R1 * skew_r_1_12 * ddomega1dt_ddq - R1 * skew(skew_omega1 * r_1_12) * Tphi1 - 2 * R1 * skew_omega1 * skew_r_1_12 * Tphi1;
    
    ddomega2dt_dq = trans_Ri * ddomega1dt_dq + trans_Ri * skew_omega1 * dTphi1 - skew_omega2 * trans_Ri * dTphi1 - dalphadt * skew_gi * trans_Ri * dTphi1 + dalphadt * skew_gi * skew_omega2 * gi * Talpha;
    
    dddr02dtdt_dq = dddr01dtdt_dq + R1 * skew_omega1 * skew_omega1 * skew_r_1_12 * Tphi1 - R1 * skew(skew_omega1 * r_1_12) * dTphi1 - 2 * R1 * skew_omega1 * skew_r_1_12 * dTphi1 - R1 * skew_r_1_12 * ddomega1dt_dq;
end