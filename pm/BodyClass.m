classdef BodyClass
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
        Joint
    end
    
    methods
        function obj = BodyClass(q, dq, BodyParameter, Frame, Body)
            ConnectedBodyNr = BodyParameter.Connection.Body;
            if ConnectedBodyNr == 0
                ConnectedBody = Frame;
            else
                ConnectedBody = Body{ConnectedBodyNr};
            end
            ConnectedJointNr = BodyParameter.Connection.Joint;
            ConnectedJoint = ConnectedBody.Joint{ConnectedJointNr};
            r01 = ConnectedJoint.r;
            R1 = ConnectedJoint.R;
            Tr1 = ConnectedJoint.Tr;
            Tphi1 = ConnectedJoint.Tphi;
            dTr1 = ConnectedJoint.dTr;
            dTphi1 = ConnectedJoint.dTphi;

            dddr01dtdt_dq = ConnectedJoint.dddrdtdt_dq;
            dddr01dtdt_ddq = ConnectedJoint.dddrdtdt_ddq;
            ddomega1dt_dq = ConnectedJoint.ddomegadt_dq;
            ddomega1dt_ddq = ConnectedJoint.ddomegadt_ddq;

            RotationAxis = BodyParameter.Connection.RotationAxis;
            RotationAngle = BodyParameter.Connection.RotationAngle;
            n = numel(q);
            if ischar(RotationAngle)
                RotationAngle = str2double(RotationAngle);
                alpha = q(RotationAngle);
                Talpha = zeros(1, n);
                Talpha(1, RotationAngle) = 1;
                dTalpha = zeros(1, n);
            elseif isnumeric(RotationAngle)
                alpha = RotationAngle;
                Talpha = zeros(1, n);
                dTalpha = zeros(1, n);
            end
            r_1_12 = zeros(3, 1);

            [r0, R, Tr, Tphi, dTr, dTphi, dddr0dtdt_dq, dddr0dtdt_ddq, ddomegadt_dq, ddomegadt_ddq, ...
                dr02dt, omega2, ddr02dtdt, domega2dt] = FixedRotation_ForwardKinematics(...
                RotationAxis, alpha, Talpha, dTalpha, r01, R1, Tr1, Tphi1, dTr1, dTphi1, ...
                dddr01dtdt_dq, dddr01dtdt_ddq, ddomega1dt_dq, ddomega1dt_ddq, ...
                dq, zeros(size(dq)), r_1_12);

            obj.r = r0;
            obj.R = R;
            obj.Tr = Tr;
            obj.Tphi = Tphi;
            obj.dTr = dTr;
            obj.dTphi = dTphi;
            obj.dddrdtdt_dq = dddr0dtdt_dq;
            obj.dddrdtdt_ddq = dddr0dtdt_ddq;
            obj.ddomegadt_dq = ddomegadt_dq;
            obj.ddomegadt_ddq = ddomegadt_ddq;
        end

        function obj=set_Joint(obj, dq, BodyParameter)
            r01 = obj.r;
            R1 = obj.R;
            Tr1 = obj.Tr;
            Tphi1 = obj.Tphi;
            dTr1 = obj.dTr;
            dTphi1 = obj.dTphi;

            dddr01dtdt_dq = obj.dddrdtdt_dq;
            dddr01dtdt_ddq = obj.dddrdtdt_ddq;
            ddomega1dt_dq = obj.ddomegadt_dq;
            ddomega1dt_ddq = obj.ddomegadt_ddq;

            BodyQuantity = size(Tr1, 2);

            JointQuantity = numel(BodyParameter.Joint);
            %obj.Joint = cell(1, JointQuantity);
            obj.Joint = {};
            for JointNr = 1:JointQuantity
                 r_1_12 =BodyParameter.Joint{JointNr}.r;
                phi_1_12 = BodyParameter.Joint{JointNr}.phi;
                alpha = norm(phi_1_12);
                if alpha ~= 0
                    RotationAxis = phi_1_12 / alpha;
                else
                    RotationAxis = [1; 0; 0];
                end
                Talpha = zeros(1, BodyQuantity);
                dTalpha = zeros(1, BodyQuantity);
                [r02, R2, Tr2, Tphi2, dTr2, dTphi2, dddr02dtdt_dq, dddr02dtdt_ddq, ddomega2dt_dq, ddomega2dt_ddq, ...
                    dr02dt, omega2, ddr02dtdt, domega2dt] = FixedRotation_ForwardKinematics(RotationAxis, ...
                    alpha, Talpha, dTalpha, r01, R1, Tr1, Tphi1, dTr1, dTphi1, ...
                    dddr01dtdt_dq, dddr01dtdt_ddq, ddomega1dt_dq, ddomega1dt_ddq, ...
                    dq, zeros(size(dq)), r_1_12);
                obj.Joint{JointNr} = JointClass(r02, R2, Tr2, Tphi2, dTr2, dTphi2, ...
                    dddr02dtdt_dq, dddr02dtdt_ddq, ddomega2dt_dq, ddomega2dt_ddq);
            end
        end
    end
end