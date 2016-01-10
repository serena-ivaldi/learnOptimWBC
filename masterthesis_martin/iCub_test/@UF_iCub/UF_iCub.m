classdef UF_iCub < Controllers.AbstractController
    properties
        subchains;       % object that contains the subchain of the robot and the J dot for each subchain;   (maybe i can leave it)
        references;      % object that contains the reference trajectory for each tasks;
        alpha;           % cell array of weight function
        repellers;       % object of repellers
        metric;          % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q))
        current_chain    % index that define the current robot that i want to move
        %ground_truth    % if true for computing the position and velocity of the end effector i will use the non perturbed model
        Kp               % vector of matrix of proportional gain
        Kd               % vector of matrix of derivative gain
        combine_rule     % projector or sum
        regularizer      % this term transform the UF in a regularized UF if it is different from zero. it is a cell array of vector each vector has as many entry as the number of task for the current chain vector
        torque_func      % in this vector i put the handle to the function that i want to use: ComputeRegularizedTorqueSum(...) ComputeTorqueSum(...)
        max_time         % maximum time simulation allowed
        current_time     % current time to force stop for long iteration
        torques          %  resulting torque (cell array of matrix)
        torques_time     % all the time istant when i aply a torque.
        display_opt      % display settings display_opt.step display_opt.trajtrack
        wbm_iCub@WBM.WBM
    end

    methods
        function obj = UF_iCub(sub_chains, references, alpha, repellers, metric, Kp, Kd, combine_rule, regularization, max_time, varargin, wbm_icub)
            obj.subchains = sub_chains;
            obj.references = references;
            obj.alpha = alpha;
            obj.repellers = repellers;
            obj.metric = metric;
            obj.Kp = Kp;
            obj.Kd = Kd;
            obj.combine_rule = combine_rule;
            obj.wbm_iCub = wbm_icub;
            % in this way i can use a generic long vector inside
            % RuntimeVariable than here i take what i need
            for i = 1:obj.subchains.GetNumChains()
                for j=1:obj.subchains.GetNumTasks(i)
                    app_vector(j) = regularization{i}(1,j);
                    if( app_vector(j) == 0)
                        %obj.torque_func{i,j} = @(ind_subchain,ind_task,M,F,t,q,qd,u1)obj.ComputeTorqueSum(ind_subchain,ind_task,M,F,t,q,qd,u1);
                        obj.torque_func{i,j} = @(ind_subchain, ind_task, M, F, t, q, dq, u1)obj.ComputeTorqueSum_iCub(ind_subchain, ind_task, M, F, t, q, dq, u1);
                    else
                        %obj.torque_func{i,j} = @(ind_subchain,ind_task,M,F,t,q,qd,u1)obj.ComputeRegularizedTorqueSum(ind_subchain,ind_task,M,F,t,q,qd,u1);
                        obj.torque_func{i,j} = @(ind_subchain, ind_task, M, F, t, q, dq, u1)obj.ComputeRegularizedTorqueSum_iCub(ind_subchain, ind_task, M, F, t, q, dq, u1);
                    end
                end
                obj.regularizer{i}=app_vector;
            end

            % initialize torque and torque time
            obj.torques = cell(obj.subchains.GetNumChains());
            for i = 1:obj.subchains.GetNumChains()
                obj.torques{i} = [];  %tau(n_of_total_joint on the chain x 1)
            end
            obj.torques_time = cell(obj.subchains.GetNumChains());
            for i = 1 :obj.subchains.GetNumChains()
                obj.torques_time{i} = [];
            end
            obj.max_time = max_time;
            obj.current_time = [];
            % default settings for smoothing and trajectory tracking display (desidered position)
            obj.display_opt.fixed_step = false;
            obj.display_opt.step = 0.00000001;
            obj.display_opt.trajtrack = false;
            % settings for smoothing and trajectory tracking display (reference position)
            %if (nargin > 7)
            %   disp_opt = varargin{1};
            %   obj.display_opt.step =disp_opt.step;
            %   obj.display_opt.trajtrack = disp_opt.trajtrack;
            %end
        end

        function SaveTime(obj,ind_subchain,time)
            obj.torques_time{ind_subchain} = [obj.torques_time{ind_subchain}(:,:),time];
        end

        function SaveTau(obj,ind_subchain,tau)
            obj.torques{ind_subchain} = [obj.torques{ind_subchain}(:,:),tau];
        end

        function CleanTau(obj)
            for i = 1 :obj.subchains.GetNumChains()
                obj.torques{i} = [];
            end
        end

        function CleanTime(obj)
            for i = 1 :obj.subchains.GetNumChains()
                obj.torques_time{i} = [];
            end
        end

        function SetCurRobotIndex(obj,index_chain)
            obj.current_chain = index_chain;
        end

        function i = GetCurRobotIndex(obj)
            i = obj.current_chain;
        end

        function bot = GetActiveBot(obj)
            bot = obj.subchains.GetCurRobot(obj.current_chain);
        end

        function bot = GetActiveBotVis(obj)
            bot = obj.subchains.GetCurRobotVis(obj.current_chain);
        end

        function final_tau = Policy(obj, t, q, qd)

            %DEBUG
            %t
            %q
            %qd
            %---

            [vqT_b, q_j, v_b, dq_j] = obj.wbm_iCub.getState();
            % get the translation p and the base rotation matrix ...
            [p_b, R_b] = WBM.utilities.frame2posRotm(vqT_b);

            %% compute the dynamics for the iCub-Robot:
            % the mass matrix:
            M = obj.wbm_iCub.massMatrix(R_b, p_b, q_j);
            % generalized bias forces (Coriolis & g-force):
            gF = obj.wbm_iCub.generalBiasForces(R_b, p_b, q_j, dq_j, v_b);

            % % active robot
            % cur_bot = obj.GetActiveBot;
            % % current chain index
            % i = obj.GetCurRobotIndex;
            % DOF = cur_bot.n;
            %
            % % the dynamic computation between controller and simulator has
            % % to be different
            % M = cur_bot.inertia(q);
            % F = cur_bot.coriolis(q,qd)*qd' + cur_bot.gravload(q)';

            % adding the stabilization part in joint space if i have only one
            % controller
            if(strcmp(obj.combine_rule,'sum'))
                for j = 1:obj.subchains.GetNumTasks(i)
                    % for stability reason when a controller use less dof
                    % then the free one i have to add a stabilizing action
                    % trough a null space porjected component
                    if(obj.subchains.GetNumSubLinks(i,j) < DOF )
                        deg = pi/180;
                        %kp = 700;
                        kp = 5;
                        kd = 2*sqrt(kp);
                        qd_des =zeros(size(q,2),1);
                        q_des  = [122;121; 19; 60; 90; 0]*deg; %  TODO to generalize for different lenght of kinematic chain
                        u1 = ( kd*(qd_des - qd') + kp*(q_des - q'));
                    else
                        u1 = zeros(size(q,2),1);
                    end
                    tau=obj.torque_func{i,j}(i,j,M,F,t,q,qd,u1);
                    app_tau(:,j) = obj.alpha{i,j}.GetValue(t)*tau;
                end

                final_tau = sum(app_tau,2);
                obj.SaveTau(i,final_tau);
                obj.SaveTime(i,t);
            elseif(strcmp(obj.combine_rule,'projector'))
                for j = 1:obj.subchains.GetNumTasks(i)
                    tau=obj.torque_func{i,j}(i,j,M,F,t,q,qd,u1);
                    app_tau(:,j) = obj.alpha{i,j}.GetValue(t)*tau;
                end

                final_tau = sum(app_tau,2);
                % compute the projector in the null space of repulsor (number of repulsor)
                for j = 1:obj.repellers.GetNumTasks(i)
                    obj.repellers.SetJacob(cur_bot,q,qd,i,j)
                end
                N = obj.repellers.ComputeProjector(i,DOF,obj.subchains.GetNumTasks(i),obj.alpha,t);
                final_tau = ((M*N)/M)*final_tau;

                obj.SaveTau(i,final_tau)
                obj.SaveTime(i,t);
            end
        end

        %% all the function from this point DO NOT SUPPORT multichain structure (this part work only with RBF)
        % in this function i update the value of the alpha function giving
        % new set of parameters

        % the implicit rule with repellers is that first i update the rbf
        % functions for the task and after i update the alpha function
        % for repellers
        % and then i update the parameter that govern the reference
        function UpdateParameters(obj,parameters)
            disp('im in update parameters')
            for i=1:size(obj.alpha,1)
                index = 1;
                for j=1:size(obj.alpha,2)
                    n_param = obj.alpha{i,j}.GetParamNum();
                    app_param = parameters(index:index+n_param - 1);
                    obj.alpha{i,j}.ComputeNumValue(app_param')
                    index = index+n_param;
                end
            end
            % update parameters of the references (if there are some)
            for i=1:size(obj.references.parameter_dim,1)
                index = 1;
                for 9j=1:size(obj.references.parameter_dim,2)
                    n_param = obj.references.parameter_dim{i,j};
                    app_param = parameters(index:index+n_param - 1);
                    if(n_param>0)
                        obj.references.cur_param_set{i,j} = (app_param');
                        index = index+n_param;
                    end

                end
            end
        end

        function n_param=GetTotalParamNum(obj)
            n_param = 0;

            for i=1:1:size(obj.alpha,1)
                for j=1:size(obj.alpha,2)
                    n_param = n_param + obj.alpha{i,j}.GetParamNum();
                end
            end
            for i=1:1:size(obj.references.parameter_dim,1)
                n_param = n_param + obj.references.GetNumParam(i);
            end
        end

    end
end
