classdef MultChainTreeICub < WBM.Interfaces.MultChainTree
    properties
        icub % pointer to the icub structure.
    end

    methods
        function obj = MultChainTreeICub(icub_wbm, link_name, comment)
            % call the constructor of the superclass ...
            obj = obj@WBM.Interfaces.MultChainTree(icub_wbm, link_name, comment);

            obj.icub = icub_wbm;
            obj.name = 'iCub';
        end

    end
end
