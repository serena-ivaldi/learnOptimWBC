classdef  BalanceController < Controllers.AbstractController
    
   properties
      subchains            % object that contains the subchain of the robot and the J dot for each subchain  (maybe i can leave it) 
      references           % object that contains the reference trajectory for each primary tasks 
      Secondary_refs       % object that contains the reference trajecotry for secondary tasks when specified
      alpha                % cell array of weight function
      repellers            % object of repellers
      metric               % vector of matlab command     for example M_inv^2, M_inv,eye(lenght(q)) 
      current_chain        % index that define the current robot that i want to move
      Param                % cell array of matrix that are contains to kind of object Param{i,j}.Kp, Param{i,j}.Kd  and Param{i,j}.M, obj.Param{i,j}.D, obj.Param{i,j}.P 
      Param_secondary      % cell array of matrix that are contains to kind of object Param{i,j}.Kp, Param{i,j}.Kd  and Param{i,j}.M, obj.Param{i,j}.D, obj.Param{i,j}.P 
      current_time         % current time to force stop for long iteration
      torques              %  resulting torque (cell array of matrix)
      torques_time         % all the time istant when i aply a torque.
      display_opt          % display settings display_opt.step display_opt.trajtrack
      gains
      simulation_results   % in this variable i save the result of the integration for fixed step  
      simulation_iterator  % i use this to save the data during the execution of the simulation
   end


   methods
      
       function obj = BalanceController(sub_chains,references,Secondary_refs,alpha,repellers,metric,Param,Param_secondary,combine_rule,regularization,varargin)
         obj.subchains = sub_chains;
         obj.references = references;
         obj.Secondary_refs = Secondary_refs;
         obj.alpha = alpha;
         obj.repellers = repellers;
         obj.metric = metric;
         obj.Param = Param;
         obj.Param_secondary = Param_secondary;
         obj.current_time = [];
         % default settings for smoothing and trajectory tracking display (desidered position) 
         obj.display_opt.fixed_step = false;
         obj.display_opt.step = 0.00000001;
         obj.display_opt.trajtrack = false;
         %% Gains for two feet on ground
         if sum(varargin{1}.feet_on_ground(1:2)) == 2
            % CoM and angular momentum gains
            gainsPCoM           = diag([40 45 40]);
            gainsDCoM           = 2*sqrt(gainsPCoM);
            gainsPAngMom        = diag([1 5 1]);
            gainsDAngMom        = 2*sqrt(gainsPAngMom);

            % impedances acting in the null space of the desired contact forces
            impTorso            = [ 20  30  20];
            impArms             = [ 10  10  10   5   5];
            impLeftLeg          = [ 35  40  10  30   5  10];
            impRightLeg         = [ 35  40  10  30   5  10];
         end

         %% Parameters for one foot on ground
         if  sum(varargin{1}.feet_on_ground(1:2)) == 1

            % CoM and angular momentum gains
            gainsPCoM          = diag([30 35 30]);
            gainsDCoM          = 2*sqrt(gainsPCoM);
            gainsPAngMom       = diag([2.5 5 2.5]);
            gainsDAngMom       = 2*sqrt(gainsPAngMom);

            % impedances acting in the null space of the desired contact forces
            impTorso           = [ 10   15   10];
            impArms            = [  5    5    5   2.5   2.5];

            if varargin{1}.feet_on_ground(1) == 1

                impLeftLeg     = [ 15   15  15  15  5  5];
                impRightLeg    = [ 10   10  15  15  5  5];
            else
                impLeftLeg     = [ 10   10  15  15  5  5];
                impRightLeg    = [ 15   15  15  15  5  5];
            end
         end

         %% Definition of the impedances and dampings vectors
         obj.gains.impedances   = [impTorso,impArms,impArms,impLeftLeg,impRightLeg];
         obj.gains.dampings     = 2*sqrt(obj.gains.impedances);

         %if (size(obj.gains.impedances,2) ~= ndof)
         %   error('Dimension mismatch between ndof and dimension of the variable impedences. Check these variables in the file gains.m');
         %end

         %% Momentum and postural gains
         obj.gains.impedances         = diag(obj.gains.impedances);
         obj.gains.dampings           = diag(obj.gains.dampings);
         obj.gains.momentumGains      = [gainsDCoM zeros(3); zeros(3) gainsDAngMom];
         obj.gains.intMomentumGains   = [gainsPCoM zeros(3); zeros(3) gainsPAngMom];

         % desired shape for the state matrix of the linearized system, for gains tuning procedure
         obj.gains.KSdes              = obj.gains.impedances;
         obj.gains.KDdes              = 2*sqrt(obj.gains.KSdes);

         % gains for feet correction to avoid numerical intrgration errors
         obj.gains.corrPosFeet        = 100;   
      end    
      % i do  not need cell object because the time is unique
      function SaveTime(obj,ind_subchain,time)
         obj.torques_time = [obj.torques_time,time];
      end
       
      % i do not need cell object because the simulated model is a unique
      % big system of differential equations
      function SaveTau(obj,ind_subchain,tau)
         obj.torques = [obj.torques,tau];   
      end
      
      function CleanTau(obj)
          for i = 1 :obj.subchains.GetNumChains()
            obj.torques = [];
          end
      end
      
      function CleanTime(obj)
         for i = 1 :obj.subchains.GetNumChains()
            obj.torques_time = [];
         end
      end
      %% TODO
      % this management of multiple chain has to be changed in favor of a 
      % a unique system to refer to to compute the dynamics component of
      % the robot
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
      %% end
      
      function WS = GetWholeSystem(obj)
          WS = obj.subchains.whole_system;
      end
      
      %% all the function from this point DO NOT SUPPORT multichain structure (this part work only with RBF)
      % in this function i update the value of the alpha function giving
      % new set of parameters
      
      % the implicit rule with repellers is that first i update the rbf
      % functions for the task and after i update the alpha function
      % for repellers 
      % and then i update the parameter that govern the reference
      function UpdateParameters(obj,parameters)
       %disp('im in update parameters')  
       %% in this controller i remove the update of the activation fuction 
       %% because we do not have any of them in place
       disp('im in update parameter inside balance controller')
       parameters
%          for i=1:size(obj.alpha,1) 
%              index = 1;
%              for j=1:size(obj.alpha,2)  
%                  n_param = obj.alpha{i,j}.GetParamNum();
%                  app_param = parameters(index:index+n_param - 1);
%                  obj.alpha{i,j}.ComputeNumValue(app_param')
%                  index = index+n_param;
%              end
%          end
         % update parameters of the references (if there are some)
         index = 1;
         for i=1:size(obj.references.parameter_dim,1) 
             for j=1:size(obj.references.parameter_dim,2)  
                 n_param = obj.references.parameter_dim{i,j};
                 app_param = parameters(index:index+n_param - 1);
                 if(n_param>0)
                     obj.references.cur_param_set{i,j} = (app_param');
                     index = index+n_param;
                 else
                     obj.references.cur_param_set{i,j} = app_param;
                 end
             end
         end         
      end
      
       
      function n_param=GetTotalParamNum(obj)
          
          n_param = 0;
%           for i=1:1:size(obj.alpha,1) 
%              for j=1:size(obj.alpha,2) 
%                  n_param = n_param + obj.alpha{i,j}.GetParamNum();
%              end
%           end
          for i=1:1:size(obj.references.parameter_dim,1) 
             n_param = n_param + obj.references.GetNumParam(i);
          end
      end
      
      function  [final_tau] = Policy(obj,t,q,qd,Fc,Jc,param)
         import WBM.utilities.frame2posRotm;
         import WBM.utilities.rotm2eulAngVelTF
         icub = obj.GetWholeSystem(); 
         %% FORWARD KINEMATICS
         % feet pose (quaternions), CoM position
         poseLFoot_qt                     = wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,q,'l_sole');
         poseRFoot_qt                     = wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,q,'r_sole');
         poseLULeg_qt                     = wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,q,'l_upper_leg');
         poseRULeg_qt                     = wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,q,'r_upper_leg');
         poseCoM                          = wbm_forwardKinematics(icub.state.w_R_b,icub.state.x_b,q,'com');
         xCoM                             = poseCoM(1:3);
         dposeCoM                         = icub.dynamic.JCoM*icub.state.Nu;
         dxCoM                            = dposeCoM(1:3);
         %% Feet orientation using Euler angles
         % feet current position and orientation (rotation matrix)
         [x_Lfoot,b_R_Lfoot]              = frame2posRotm(poseLFoot_qt);
         [x_Rfoot,b_R_Rfoot]              = frame2posRotm(poseRFoot_qt);
         [x_LULeg,b_R_x_LULeg]            = frame2posRotm(poseLULeg_qt);
         [x_RULeg,b_R_x_RULeg]            = frame2posRotm(poseRULeg_qt);

         % orientation is parametrized with euler angles
         [theta_Lfoot,~]             = rotm2eulAngVelTF(b_R_Lfoot);
         [theta_Rfoot,~]             = rotm2eulAngVelTF(b_R_Rfoot);
         [theta_LULeg,~]             = rotm2eulAngVelTF(b_R_x_LULeg);
         [theta_RULeg,~]             = rotm2eulAngVelTF(b_R_x_RULeg);
         poseLFoot_ang                    = [x_Lfoot; theta_Lfoot];
         poseRFoot_ang                    = [x_Rfoot; theta_Rfoot];
         poseLULeg_ang                 = [x_LULeg;theta_LULeg];
         poseRULeg_ang                 = [x_RULeg;theta_RULeg];
         % feet velocity, CoM velocity
         FORKINEMATICS.xCoM               = xCoM;
         FORKINEMATICS.dxCoM              = dxCoM;
         FORKINEMATICS.poseRFoot_ang      = poseRFoot_ang;
         FORKINEMATICS.poseLFoot_ang      = poseLFoot_ang;
         FORKINEMATICS.poseLULeg_ang   = poseLULeg_ang;
         FORKINEMATICS.poseRULeg_ang   = poseRULeg_ang;
         %% CoM and joints trajectory generator
         trajectory.jointReferences.ddqjRef = zeros(icub.ndof,1);
         trajectory.jointReferences.dqjRef  = zeros(icub.ndof,1);
         trajectory.jointReferences.qjRef   = icub.init_state.qi;%param.qfinal;
         trajectory.desired_x_dx_ddx_CoM = myTrajectoryGenerator(obj,t,icub.init_state.xCoMRef,param.xComfinal);
         
         %% given the value of the com trajectory if the desired com is different from the starting position i will
         %% update the feet on ground to remove the bottom contact
               
         controlParam = InnerPolicy(obj,param,obj.gains,trajectory,icub.dynamic,FORKINEMATICS,icub.state,icub.contact_jacobians);  
         if  param.use_QPsolver == 1                 
             % quadratic programming solver for the nullspace of contact forces
             controlParam.fcDes = QPSolver(controlParam,param,FORKINEMATICS);
         end

         controlParam.tau = controlParam.tauModel + controlParam.Sigma*controlParam.fcDes;
         final_tau = controlParam.tau;

         obj.SaveTau(1,final_tau) 
         obj.SaveTime(1,t);  
              
      end
      
      function controlParam = InnerPolicy(obj,CONFIG,gain,trajectory,DYNAMICS,FORKINEMATICS,STATE,contact_jacobians)
      
          import WBM.utilities.skewm;
          icub = obj.GetWholeSystem();
          Jc = contact_jacobians.Jc;
          dJc_nu = contact_jacobians.dJcNu;
          %% Config parameters
          pinv_tol            = CONFIG.pinv_tol;
          pinv_damp           = CONFIG.pinv_damp;
          feet_on_ground      = CONFIG.feet_on_ground;
          ndof                = icub.ndof;

          %% Gains
          impedances          = gain.impedances;
          dampings            = gain.dampings;
          momentumGains       = gain.momentumGains;
          intMomentumGains    = gain.intMomentumGains;

          %% Dynamics
          M                   = DYNAMICS.M;
          g                   = DYNAMICS.g;
          C_nu                = DYNAMICS.C_nu;
          H                   = DYNAMICS.H;
          h                   = g + C_nu;
          m                   = M(1,1);
          Mb                  = M(1:6,1:6);
          Mbj                 = M(1:6,7:end);
          Mj                  = M(7:end,7:end);
          Mjb                 = M(7:end,1:6);
          Mbar                = Mj - Mjb/Mb*Mbj;
          % The centroidal momentum jacobian is reduced to the joint velocity. This
          % is then used to compute the approximation of the angular momentum integral
          JH                  = DYNAMICS.JH;
          JG                  = JH(:,7:end) - JH(:,1:6)*(eye(6)/Jc(1:6,1:6))*Jc(1:6,7:end);

          %% Forward kinematics
          xCoM                = FORKINEMATICS.xCoM;
          dxCoM               = FORKINEMATICS.dxCoM;
          poseRFoot_ang       = FORKINEMATICS.poseRFoot_ang;
          poseLFoot_ang       = FORKINEMATICS.poseLFoot_ang;
          poseLULeg_ang    = FORKINEMATICS.poseLULeg_ang;
          poseRULeg_ang    = FORKINEMATICS.poseRULeg_ang;
          %% Robot State
          qj                  = STATE.q;
          dqj                 = STATE.qd;

          % Trajectory
          x_dx_ddx_CoMDes     = trajectory.desired_x_dx_ddx_CoM;
          ddqjRef             = trajectory.jointReferences.ddqjRef;
          dqjRef              = trajectory.jointReferences.dqjRef;
          qjRef               = trajectory.jointReferences.qjRef;
          qjTilde             = qj-qjRef;
          dqjTilde            = dqj-dqjRef;

          % General parameters
          gravAcc             = 9.81;
          S                   = [zeros(6,ndof);
                               eye(ndof,ndof)];
          f_grav              = [zeros(2,1);
                              -m*gravAcc;
                               zeros(3,1)];

          %% Multiplier of contact wrenches at CoM
          x_RFoot              = poseRFoot_ang(1:3);
          x_LFoot              = poseLFoot_ang(1:3);
          x_LULeg              = poseLULeg_ang(1:3);
          x_RULeg              = poseRULeg_ang(1:3);
          r_RF                 = x_RFoot - xCoM;       % Application point of the contact force on the right foot w.r.t. CoM
          r_LF                 = x_LFoot - xCoM;       % Application point of the contact force on the left  foot w.r.t. CoM
          r_LUL                = x_LULeg - xCoM;
          r_RUL                = x_RULeg - xCoM;
          AL                   = [eye(3),    zeros(3);
                                skewm(r_LF),eye(3)];
          AR                   = [eye(3),    zeros(3);
                                skewm(r_RF),eye(3)];
          LUL                   = [eye(3),    zeros(3);
                                skewm(r_LUL),eye(3)];     
          RUL                   = [eye(3),    zeros(3);
                                 skewm(r_RUL),eye(3)];     

          % One foot or two feet on ground selector
          if      sum(feet_on_ground) == 4
              A      = [AL,AR,LUL,RUL];
              pinvA  = pinv(A,pinv_tol);
          elseif      sum(feet_on_ground) == 3
              if(feet_on_ground(3)==1)
                  A      = [AL,AR,LUL];
                  pinvA  = pinv(A,pinv_tol);
              else
                  A      = [AL,AR,RUL];
                  pinvA  = pinv(A,pinv_tol);
              end
          elseif      sum(feet_on_ground) == 2
              A      = [AL,AR];
              pinvA  = pinv(A,pinv_tol);
          else
              if      feet_on_ground(1) == 1

                  A  = AL;
              elseif  feet_on_ground(2) == 1

                  A  = AR;
              end
              pinvA  = eye(6)/A;
          end

          %% Desired momentum derivative
          % closing the loop on angular momentum integral
          deltaPhi           =  JG(4:end,:)*(qj-qjRef);
          accDesired         =  [m.*x_dx_ddx_CoMDes(:,3); zeros(3,1)];
          velDesired         = -momentumGains*[m.*(dxCoM-x_dx_ddx_CoMDes(:,2)); H(4:end)];
          posDesired         = -intMomentumGains*[m.*(xCoM-x_dx_ddx_CoMDes(:,1)); deltaPhi];
          HDotDes            =  accDesired + velDesired + posDesired;

          %% Control torques equation
          JcMinv             = Jc/M;
          Lambda             = JcMinv*S;
          JcMinvJct          = JcMinv*transpose(Jc);
          pinvLambda         = pinv(Lambda,pinv_tol);

          % multiplier of contact wrenches in tau0
          JBar               = transpose(Jc(:,7:end)) - Mbj'/Mb*transpose(Jc(:,1:6));

          % nullspaces
          NullA              = eye(6*CONFIG.numContacts)-pinvA*A;
          NullLambda         = eye(ndof)-pinvLambda*Lambda;

          Sigma              = -(pinvLambda*JcMinvJct + NullLambda*JBar);
          SigmaNA            =  Sigma*NullA;

          % Postural task correction
          pinvLambdaDamp     =  Lambda'/(Lambda*Lambda' + pinv_damp*eye(size(Lambda,1)));
          NullLambdaDamp     =  eye(ndof)-pinvLambdaDamp*Lambda;
          posturalCorr       =  NullLambdaDamp*Mbar;
          impedances         =  impedances*pinv(posturalCorr,pinv_tol) + 0.01*eye(ndof);
          dampings           =  dampings*pinv(posturalCorr,pinv_tol) + 0.01*eye(ndof);

          tauModel           =  pinvLambda*(JcMinv*h - dJc_nu) + NullLambda*(h(7:end) -Mbj'/Mb*h(1:6)...
                              + Mbar*ddqjRef - impedances*posturalCorr*qjTilde - dampings*posturalCorr*dqjTilde);

          %% Desired contact forces computation
          fcHDot             = pinvA*(HDotDes - f_grav);

          % Desired f0 without Quadratic Programming
          f0                 = zeros(6,1);

          if  sum(feet_on_ground) >= 2

              f0             = -pinv(SigmaNA,pinv_tol)*(tauModel+Sigma*fcHDot);
          end

          fcDes              = fcHDot + NullA*f0;

          %% Output parameters
          controlParam.fcHDot   = fcHDot;
          controlParam.HDotDes  = HDotDes;
          controlParam.fcDes    = fcDes;
          controlParam.f_grav   = f_grav;
          controlParam.tauModel = tauModel;
          controlParam.Sigma    = Sigma;
          controlParam.f0       = f0;
          controlParam.NullA    = NullA;
          controlParam.A        = A;

      end
      
      
      function  desired_x_dx_ddx_CoM = myTrajectoryGenerator(obj,t,xCoMInit,xComfinal)
                % in this way i specify in a fixed way the final value and
                % the starting value
                if(t<=0.1)
                    xCoMDes     = xCoMInit;
                    dxCoMDes    = zeros(size(xCoMInit));
                    ddxCoMDes   = zeros(size(xCoMInit));
                elseif(t>9.9)
                    xCoMDes     = xComfinal;
                    dxCoMDes    = zeros(size(xComfinal));
                    ddxCoMDes   = zeros(size(xComfinal));
                else
                    [xCoMDes,dxCoMDes,ddxCoMDes]=obj.references.GetTraj(1,1,t);
                end

                desired_x_dx_ddx_CoM = [xCoMDes dxCoMDes ddxCoMDes];
      end
   end   
end








































