classdef JointParameterClass
    properties
        r
        phi
    end
    
    methods
        function obj = JointParameterClass(r, phi)
            r = r(1:3);
            phi = phi(1:3);
            obj.r = reshape(r, 3, 1);
            obj.phi = reshape(phi, 3, 1);
        end
    end
end