classdef MultChainTreeICub < WBM.Interfaces.MultChainTree
    properties(Dependent)
        hwbm@WBM.Interfaces.IWBM % handle (pointer) to the iCub-WBM interface (read only).
    end

    methods
        function bot = MultChainTreeICub(icub_wbm, ctrl_lnk_name, varargin)
            % call the constructor of the superclass:
            % opt = varargin{1}
            bot = bot@WBM.Interfaces.MultChainTree(icub_wbm, ctrl_lnk_name, varargin{:});

            if isempty(bot.mwbm_info.robot_name)
                bot.mwbm_info.robot_name = 'iCub';
            end
        end

        function hwbm = get.hwbm(bot)
            hwbm = bot.mwbm;
        end

    end
end
