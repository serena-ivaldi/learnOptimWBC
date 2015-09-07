classdef  iCubRobot < Controllers.AbstractController
    properties
        params;
    end

    methods
        % constructor:
        function obj = iCubRobot()
          % Initialization:
          %
          % initialize the mex-wholeBodyModel of the iCub-Robot
          % by using the Unified Robot Description Format (URDF)
          % for the Gazebo simulator ...
          wbm_modelInitialize('icubGazeboSim');
          
          %wbm_setWorldLink();

        end

    end
end  
