%classdef MultLinkTree < handle
classdef MultChainTree < handle
    properties(Dependent)
        name@char    % the name of the robot.
        manuf@char   % the name of the manufacturer (annotation)
        comment@char % general comment (annotation)

        n@uint16     % number of joints (equivalent to number of DoFs)
        config       % base model and configuration parameters of the robot

        gravity@double vector  % gravity vector (direction of the gravity)
        base@double    matrix  % base transform of the robot (pose of the robot)
        tool@double    matrix  % tool transform (from the end-effector to the tool-tip)

        %interface % interface to a real robot platform

        %model3d
        plotopt3d

        %links

        link      % current kinematic link of the robot that is controlled by the system.
    end

    properties
        model3d % dummy parameter to be compliant with the subchain class % ???
    end

    properties(Access = protected)
        mrobot_wbm@WBMInterface
        mrobot_name@char
        mrobot_manuf@char
        mrobot_comment@char
        mlink_name@char
    end

    methods
        function obj = MultChainTree(robot_wbm, link_name, robot_info)
            if ~isa(robot_wbm, 'WBMInterface')
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if isempty(link_name)
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end

            switch nargin
                case 2
                    obj.mrobot_name    = robot_wbm.robot_name;
                    obj.mrobot_manuf   = robot_wbm.robot_manuf;
                    obj.mrobot_comment = ''; % 'Whole Body Model for the iCub-Robot.'
                case 3
                    if ~isstruct(robot_info)
                        error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                    end
                    obj.mrobot_name    = robot_info.name;
                    obj.mrobot_manuf   = robot_info.manuf;
                    obj.mrobot_comment = robot_info.comment;
                otherwise
                    error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_ARG);
            end

            obj.mrobot_wbm = robot_wbm;
            obj.mlink_name = link_name;
        end

        function w_H_rlnk = fkine(obj, q_j)
            obj.mrobot_wbm.fkine(q_j, obj.mlink_name);
        end

        function dJ = jacob_dot(obj, q_j, dq_j)
            obj.mrobot_wbm.jacob_dot(q_j, dq_j, obj.mlink_name);
        end

        function J_0 = jacob0(obj, q_j, varargin)
            opt.rpy   = false;
            opt.eul   = false;
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin);

            J_0 = obj.mrobot_wbm.jacob0(q_j, obj.mlink_name);

            if opt.rpy
                wf_H_rlnk = obj.mrobot_wbm.fkine(q_j, obj.mlink_name);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_rlnk, 'ZYX'); % use the RPY (ZYX) euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            elseif opt.eul
                wf_H_rlnk = obj.mrobot_wbm.fkine(q_j, obj.mlink_name);
                B_inv = WBM.utilities.tform2angRateTF(wf_H_rlnk, 'ZYX'); % use the ZYZ euler-angles

                J_0 = blkdiag(eye(3,3), B_inv) * J_0;
            end

            if opt.trans
                J_0 = J_0(1:3,1:6);
            elseif opt.rot
                J_0 = J_0(4:6,1:6);
            end
        end

        function robot_info = get.info(obj)
            robot_info.name    = obj.mrobot_name;
            robot_info.manuf   = obj.mrobot_manuf;
            robot_info.comment = obj.mrobot_comment;
        end

        function robot_name = get.name(obj)
            robot_name = obj.mrobot_name;
        end

        function get.manuf(obj, robot_manuf)
            robot_manuf = obj.mrobot_manuf;
        end

        function set.comment(obj, comment)
            obj.mrobot_comment = comment;
        end

        function comment = get.comment(obj)
            comment = obj.mrobot_comment;
        end

        function ndof = get.n(obj)
            ndof = obj.mrobot_wbm.ndof;
        end

        function robot_params = get.config(obj)
            robot_params = obj.mrobot_wbm.base_robot_params;
        end

        function g_wf = get.gravity(obj)
            g_wf = obj.mrobot_wbm.g_wf;
        end

        function set.base(obj, tform)
            obj.mrobot_wbm.base_tform = tform;
        end

        function tform = get.base(obj)
            tform = obj.mrobot_wbm.base_tform;
        end

        function set.tool(obj, tform)
            obj.mrobot_wbm.tool_tform = tform;
        end

        function tform = get.tool(obj)
            tform = obj.mrobot_wbm.tool_tform;
        end

        function set.plotopt3d(obj, sim_config)
            if ~isa(sim_config, 'WBM.absSimConfig')
                error('MultChainTree::set.plotopt3d: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            obj.mrobot_wbm.setupSim(obj, sim_config);
        end

        function sim_config = get.plotopt3d(obj)
            sim_config = obj.mrobot_wbm.sim_config;
        end

        function set.link(obj, link_name)
            if isempty(link_name)
                error('MultChainTree::set.link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            obj.mlink_name = link_name;
        end

        function link_name = get.link(obj)
            link_name = obj.mlink_name;
        end

    end

end
