classdef BodyParameterClass
    properties
        Joint
        Connection
    end
    
    methods
        function obj = BodyParameterClass(BodyNr, JointNr, RotationAxis, RotationAngle)
            obj.Joint = {};
            obj.Connection = BodyConnectionParameterClass(BodyNr, JointNr, RotationAxis, RotationAngle);
        end
        
        function set_Joint_Parameter(obj, r, phi)
            obj.Joint{end+1} = JointParameterClass(r, phi);
        end
    end
end





