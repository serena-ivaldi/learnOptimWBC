%VREP_obj V-REP mirror of simple object
%
% Mirror objects are MATLAB objects that reflect objects in the V-REP
% environment.  Methods allow the V-REP state to be examined or changed.
%
% This is a concrete class, derived from VREP_mirror, for all V-REP Force Sensors 
% and allows access to forces and torque exerted ont he sensor
%
% Example::
%          vrep = VREP();
%          bill = vrep.object('Bill');  % get the human figure Bill
%          bill.setpos([1,2,0]);
%          bill.setorient([0 pi/2 0]);
%
% Methods throw exception if an error occurs.
%
% Methods::
%
% Properties (read/write)::
%
% See also VREP_mirror, VREP_obj, VREP_arm, VREP_camera, VREP_hokuyo.



classdef VAREP_forcesens < VAREP_obj
    
    properties
    end
    
    methods
        
        function obj = VAREP_forcesens(vrep, name)
            %VREP_obj.VREP_obj VREP_obj mirror object constructor
            %
            % v = VREP_base(NAME) creates a V-REP mirror object for a
            % simple V-REP object type.
            obj = obj@VAREP_obj(vrep, name);
        end
        
        function Fc = readforces(obj,world)
            [force,momentum]=obj.vrep.readforcesensor(obj.h);
            % column vector
            Fc = [force';momentum'];
            if(nargin>1)
                T = obj.getpose();
                Fc = [(T(1:3,1:3)*force');(T(1:3,1:3)*momentum')];
            end
        end
    end
end
    