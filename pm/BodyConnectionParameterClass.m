classdef BodyConnectionParameterClass
    properties
        Body
        Joint
        RotationAxis
        RotationAngle
    end
    
    methods
        function obj = BodyConnectionParameterClass(BodyNr, JointNr, RotationAxis, RotationAngle)
            obj.Body = BodyNr;
            obj.Joint = JointNr;
            obj.RotationAxis = RotationAxis;
            obj.RotationAngle = RotationAngle;
        end
    end
end