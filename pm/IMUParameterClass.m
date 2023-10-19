classdef IMUParameterClass
    properties
        Body
        r
        phi
        Ra
        Rw
    end

    methods
         function obj = IMUParameterClass(BodyNr, r, phi, Ra, Rw)
            obj.Body = BodyNr;
            obj.r = r;
            obj.phi = deg2rad(phi);
            obj.Ra = Ra * eye(3);
            obj.Rw = Rw * eye(3);
         end

        
    end
end





