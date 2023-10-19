classdef FrameParameterClass
    properties
        Joint
    end
    
    methods
        function obj = FrameParameterClass()
            obj.Joint = {};
        end
        
        function set_Joint_Parameter(obj, r, phi)
            jointParam = JointParameterClass(r, phi);
            obj.Joint{end+1} = jointParam;
        end
    end
end