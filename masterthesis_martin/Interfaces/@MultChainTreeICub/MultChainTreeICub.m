classdef MultChainTreeICub < WBM.Interfaces.MultChainTree
    properties(Dependent)
        tag     % string that represent the kinematic link that i want to control
    end

    properties
        icub    % pointer to the icub structure (to reproduce the same structure of )
        link    % dummy parameter to be compliant with the subchain class
        %name    % dummy parameter to be compliant with the subchain class
        model3d % dummy parameter to be compliant with the subchain class % ???
    end

    methods
        function obj = MultChainTreeICub(icub_wbm, link_name, comment)
            % call the constructor of the superclass ...
            obj = obj@WBM.Interfaces.MultChainTree(icub_wbm, link_name, comment);

            obj.icub    = icub_wbm;
            obj.link    = 'iCub';
            obj.name    = 'iCub';
            obj.model3d = 'iCub';
        end

        function set.tag(obj, link_name)
            obj.ctrl_link = link_name;
        end

        function link_name = get.tag(obj)
            link_name = obj.ctrl_link;
        end

    end
end
