% this class is an iterface between the icub robot class and the subchain class

classdef DummyRvc_iCub < handle

    properties
        icub             % pointer to the icub structure (to reproduce the same structure of )
        link             % dummy parameter to be compliant with the subchain class
        name             % dummy parameter to be compliant with the subchain class
        model3d          % dummy parameter to be compliant with the subchain class
        tag              % string that represent the kinematic link that i want to control
    end
    
    methods
        function obj = DummyRobot(icub,tag)
            obj.icub = icub;
            obj.link = 'iCub';
            obj.name = 'iCub';
            obj.model3d = 'iCub';
            obj.tag = tag;
            
        end   
        
        function fkine(obj,q)
            obj.icub.fkine(obj,q,obj.tag);
        end
        
        function jacob0(obj,q)
            obj.icub.jacob0(obj,q,obj.tag);
        end
        
        function jacob_dot(obj,q,dq)
            obj.icub.jacob_dot(q,dq,obj.tag);
        end
        
        
        
        
    end

    


end