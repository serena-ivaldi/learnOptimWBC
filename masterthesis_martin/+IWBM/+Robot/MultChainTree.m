%classdef MultLinkTree < handle
classdef MultChainTree < handle
    properties(Dependent)
        name    % the name of the robot.
        manuf   % the name of the manufacturer (annotation)
        comment % general comment (annotation)

        n       % number of joints (equivalent to DOF)
        config  % joint configuration string, e.g. 'RRRRRR'

        gravity  % gravity vector (direction of the gravity)
        base     % base transform of the robot (pose of the robot)
        tool     % tool transform (from the end-effector to the tool-tip)

        interface % interface to a real robot platform

        model3d
        plotopt3d

        %links

        link

        % icub             % pointer to the icub structure (to reproduce the same structure of )
        % n                % degrees of freedom
        % link             % dummy parameter to be compliant with the subchain class
        % name             % dummy parameter to be compliant with the subchain class
        % model3d          % dummy parameter to be compliant with the subchain class
        % tag              % string that represent the kinematic link that i want to control
    end

    properties(Access = protected)
        tree_robot
    end

    methods
        function obj = MultChainTree()

        end

    end

end
