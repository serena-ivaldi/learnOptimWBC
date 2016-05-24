%% 
%  This is the main program for integrating the forward dynamics of the robot iCub in matlab.
%  It integrates the robot state defined in forwardDynamics_SoT.m and the user can set 
%  how many feet are on the ground, decide if activate the robot's movements, 
%  plot forces, torques and joints variables, and activate a demo of the
%  robot's movements.

% #TODO substitute icub with controller.subchains
function [t,chi,visual_param] = DynSim_iCub(controller,params) 
    WS = controller.GetWholeSystem();
    %% Updating the robot position and define the world link
    WS.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);
    [~,T_b,~,~] = WS.GetState();

    params.chiInit = [T_b; params.qjInit; WS.dx_b; WS.omega_W; params.dqjInit];
    %integration functio
    forwardDynFunc  = @(t,chi)forwardDynamics(t,chi,controller,params);
    %% Integrate forward dynamics
    if params.demo_movements == 0 

      options = odeset('RelTol',1e-3,'AbsTol', 1e-4);

    else

      options = odeset('RelTol',1e-6,'AbsTol',1e-6);

    end   
    [t,chi,visual_param] = ode15s(forwardDynFunc,params.tStart:params.sim_step:params.tEnd,params.chiInit,options);
    delete(params.wait)       
end 

function [dchi,visual_param]=forwardDynamics(t,chi,controller,param)
%% forwardDynamics_SoT
%  This is the forward dynamics of the model loaded in the 
%  wholeBodyInterface from the URDF description. The dynamic model is
%  described as an explicit ordinary differential equation of the form:
%
%              dchi = forwardDynamics(t,chi)
%
%  where chi is the variable to be integrated. For a floating base
%  articulated chain, the variable chi contains the following
%  subvariables:
%
%  x_b:      the cartesian position of the base (R^3)
%  qt_b:     the quaternion describing the orientation of the base (global parametrization of SO(3))
%  qj:       the joint positions (R^ndof)
%  dx_b:     the cartesian velocity of the base (R^3)
%  omega_b:  the velocity describing the orientation of the base (SO(3))
%  dqj:      the joint velocities (R^ndof)
  % i get the pointer to the whole system
  icub = controller.GetWholeSystem();
  waitbar(t/param.tEnd,param.wait)
  ndof = icub.ndof;
% disp(t)

%% Extraction of state

x_b  = chi(1:3,:);  %TODO floating base flag required (parameter of the simulator)
qt_b = chi(4:7,:);  %TODO floating base flag required (parameter of the simulator)

% normalize quaternions to avoid numerical errors
% qt_b = qt_b/norm(qt_b);

qj   = chi(8:ndof+7,:);

% linear and angular velocity
dx_b    = chi(ndof+8:ndof+10,:);   %TODO floating base flag required (parameter of the simulator)
omega_w = chi(ndof+11:ndof+13,:);  %TODO floating base flag required (parameter of the simulator)
dqj     = chi(ndof+14:2*ndof+13,:);

Nu      = [dx_b;omega_w;dqj];

% Obtaining the rotation matrix from root link to world frame
qT         = [x_b;qt_b];
[~,R_b]    = frame2posrot(qT);

%% Joints limits check
% % % limits = param.limits;
% % % l_min  = limits(:,1);
% % % l_max  = limits(:,2);
% % % tol    = 0.01;
% % % 
% % % res = qj < l_min + tol | qj > l_max - tol;
% % % res = sum(res);
% % % 
% % % if res==0
% % % 
% % % else
% % %  
% % %  disp('Joint limits reached at time:')    
% % %  disp(t)
% % %  error('Joint limits reached '); 
% % % 
% % % end

%% Building up contact jacobian
% % contact jacobians
% Jc    = zeros(6*param.numConstraints,6+ndof);
% dJcNu = zeros(6*param.numConstraints,1);
% 
% for i=1:param.numConstraints
%     
%     Jc(6*(i-1)+1:6*i,:)    = wbm_jacobian(R_b,x_b,qj,param.constraintLinkNames{i});
%     dJcNu(6*(i-1)+1:6*i,:) = wbm_djdq(R_b,x_b,qj,dqj,[dx_b;omega_w],param.constraintLinkNames{i});
%     
% end
% 
% % CoM jacobian
% J_CoM  = wbm_jacobian(R_b,x_b,qj,'com');

%% compute contact forces
% % Real contact forces computation
% S               = [ zeros(6,ndof);
%                     eye(ndof,ndof)];
% JcMinv          = Jc/M;
% JcMinvS         = JcMinv*S;
% 
% fc              = (JcMinv*transpose(Jc))\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);

% i make the hypothesis that i compute the contact jacobian outside 
fc = zeros(6,1);
Jc_t = zeros(ndof + 6,6);
%% MexWholeBodyModel functions
%TODO floating base flag required (parameter of the simulator)
icub.SetFloatingBaseState(x_b,qt_b,dx_b,omega_w); %TODO floating base flag required (parameter of the simulator)
%% Feet correction to avoid numerical integration errors
% % feet correction gain
% K_corr_pos  = 2.5;
% K_corr_vel  = 2*sqrt(K_corr_pos);
% 
% % feet current position and orientation
% [x_lfoot,R_b_lfoot]    = frame2posrot(l_sole);
% [x_rfoot,R_b_rfoot]    = frame2posrot(r_sole);
% 
% % orientation is parametrized with euler angles
% [~,phi_lfoot]          = parametrization(R_b_lfoot);
% [~,phi_rfoot]          = parametrization(R_b_rfoot);
% 
% pos_leftFoot           = [x_lfoot; phi_lfoot'];
% pos_rightFoot          = [x_rfoot; phi_rfoot'];
% 
% % feet original position and orientation
% lsole_ini              = param.lfoot_ini;
% rsole_ini              = param.rfoot_ini;
% 
% [xi_lfoot,R_bi_lfoot]  = frame2posrot(lsole_ini);
% [xi_rfoot,R_bi_rfoot]  = frame2posrot(rsole_ini);
% 
% [~,phi_rfoot_ini]      = parametrization(R_bi_rfoot);
% [~,phi_lfoot_ini]      = parametrization(R_bi_lfoot);
% 
% lfoot_ini_tot          = [xi_lfoot; phi_lfoot_ini'];
% rfoot_ini_tot          = [xi_rfoot; phi_rfoot_ini'];
%   
% % error between original and current feet position and orientation
% if     param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 0
%      
%        pos_feet_delta = pos_leftFoot-lfoot_ini_tot;
%  
% elseif param.feet_on_ground(1) == 0 && param.feet_on_ground(2) == 1
%      
%        pos_feet_delta = pos_rightFoot-rfoot_ini_tot;       
% 
% elseif param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 1
%     
%        pos_feet_delta = [(pos_leftFoot-lfoot_ini_tot);...
%                          (pos_rightFoot-rfoot_ini_tot)];    
% end
% 
% % parameters for controller
% feet.l_sole         = l_sole;
% feet.r_sole         = r_sole;

%% Control torques calculation
% gains and friction cones definition
%[gains,constraints,trajectory] = gainsAndConstraints_SoT(param);

% CoM trajectory generator
%desired_x_dx_ddx_CoM = generTraj_SoT(xCoMDes,t,trajectory);

% controller
%tau = stackOfTaskController(param, constraints, feet, gains, Nu, M, h, H, Jc, dJcNu, xCoM, J_CoM, desired_x_dx_ddx_CoM);    
% evaluate the torque function if one is given
if isobject(controller)
    tau = controller.Policy(t,qj,dqj,fc,Jc_t);
else   
    tau = zeros(ndof + 6,1);
end        

%% State derivative computation
% this is for advancing the simulation 
[M,h,~] = icub.WholeBodyDynamics(qj,dqj);
% Need to calculate the quaternions derivative
omega_b = transpose(R_b)*omega_w; %TODO floating base flag required (parameter of the simulator)                          
dqt_b   = quaternionDerivative(omega_b,qt_b);   %TODO floating base flag required (parameter of the simulator)    

dx      = [dx_b;dqt_b;dqj];
dNu     = M\(Jc_t*fc + [zeros(6,1); tau]-h);

dchi    = [dx;dNu];  

%% Visualization 
% These are the variables that can be plotted by the visualizer.m
% function
%  visual_param.Href      =  [M(1,1)*desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
%  visual_param.H         =  H;
%  visual_param.pos_feet  =  [l_sole;r_sole];
%  visual_param.fc        =  fc;
%  visual_param.tau       =  tau;
%  visual_param.qj        =  qj;
%  visual_param.error_com =  errorCoM;
%  visual_param.f0        =  f0;

end







   