classdef JointClass
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
    end
    
    methods
        function obj = JointClass(r, R, Tr, Tphi, dTr, dTphi, dddrdtdt_dq, dddrdtdt_ddq, ddomegadt_dq, ddomegadt_ddq)
            obj.r = r;
            obj.R = R;
            obj.Tr = Tr;
            obj.Tphi = Tphi;
            obj.dTr = dTr;
            obj.dTphi = dTphi;
            obj.dddrdtdt_dq = dddrdtdt_dq;
            obj.dddrdtdt_ddq = dddrdtdt_ddq;
            obj.ddomegadt_dq = ddomegadt_dq;
            obj.ddomegadt_ddq = ddomegadt_ddq;
        end
    end
end