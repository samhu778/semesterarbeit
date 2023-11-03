classdef FrameClass
    properties
        r
        R
        Tr
        Tphi
        dTr
        dTphi
        dddrdtdt_dq
        dddrdtdt_ddq
        ddomegadt_dq
        ddomegadt_ddq
        n
        Parameter
        Joint
    end

    methods
        function obj = FrameClass(n, Parameter)
            obj.r = [0; 0; 0]; 
            obj.R = eye(3); 
            obj.Tr = zeros(3, n);
            obj.Tphi = zeros(3, n);
            obj.dTr = zeros(3, n);
            obj.dTphi = zeros(3, n);
            obj.dddrdtdt_dq = zeros(3, n);
            obj.dddrdtdt_ddq = zeros(3, n);
            obj.ddomegadt_dq = zeros(3, n);
            obj.ddomegadt_ddq = zeros(3, n);
            obj.n = n;
            obj.Parameter = Parameter;
        end

        function obj=set_Joint(obj, dq)
            obj.Joint = {};
            FrameParameter = obj.Parameter;
            JointQuantity = length(FrameParameter.Joint);
            for JointNr = 1:JointQuantity
                r_1_12 = FrameParameter.Joint{JointNr}.r;
                phi_1_12 = FrameParameter.Joint{JointNr}.phi;
                alpha = norm(phi_1_12);
                if alpha ~= 0
                    RotationAxis = phi_1_12 / alpha;
                else
                    RotationAxis = [1; 0; 0];
                end
                Talpha = zeros(1, obj.n);
                dTalpha = zeros(1, obj.n);

                [r02, R2, Tr2, Tphi2, dTr2, dTphi2, dddr02dtdt_dq, dddr02dtdt_ddq, ddomega2dt_dq, ddomega2dt_ddq, ...
                 dr02dt, omega2, ddr02dtdt, domega2dt] = FixedRotation_ForwardKinematics(...
                     RotationAxis, alpha, Talpha, dTalpha, ...
                     obj.r, obj.R, obj.Tr, obj.Tphi, obj.dTr, obj.dTphi, ...
                     obj.dddrdtdt_dq, obj.dddrdtdt_ddq, ...
                     obj.ddomegadt_dq, obj.ddomegadt_ddq, ...
                     dq, zeros(size(dq)), r_1_12);

                obj.Joint{JointNr} = JointClass(r02, R2, Tr2, Tphi2, dTr2, dTphi2, ...
                                                dddr02dtdt_dq, dddr02dtdt_ddq, ddomega2dt_dq, ddomega2dt_ddq);
            end
        end
    end
end