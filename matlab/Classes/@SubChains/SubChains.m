
% SubChains became really usefull when i have DIFERRENT kinematic chains

% TODO: to manage different kinematic the idea is to create an extended
% dynamic matrix to simulate the system as a unique object
% some hypothesis to solve the problem
% we can add a pointer to an object that build the extended dynamical
% matrix and then call it during the computation of the tau 
%(like having an iCub object to query the dynamical matrix... 7
%and then use the local informations inside the single subchain...
%to compute the control actions) 
% maybe i can define an abstract class called whole system to distinguish
% between single subchain and the total system.
classdef  SubChains < handle


    properties
       target_link;    %  cell array of vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
       sub_chains;     %  cell array of kinematic model symbolic and not
       sub_chains_vis; %  cell array of robot for visualization purpose
       symbolic_flag;  %  is true if i have a symbolic counterpart fo the kinematic chains
       whole_system;   %  this field is a pointer to the whole dynamic structure where i define complex multichain body connected to each other
                       % in this case the kinematic information has to specified for each chain separately (inside sub_chains) but the dynamic infomarmation 
                       % depends on the overall structure
       floating_base;  % a flag that tell us if in the computation of the controller we hanve to take into account the floating base 
    end
    
    
    methods
  
      function obj = SubChains(target_link,sub_chains,varargin)        
         
         obj.whole_system = []; 
         obj.target_link = target_link;
         obj.floating_base = false;
         
         for i = 1:size(target_link,1)
            
            rob_name = sub_chains{i}.name;
            rob_name(rob_name==' ')=[];
            
            % here i use the model of the symbolic robot
            if which(strcat(rob_name,'_done.m'))
               app_rob = eval(strcat(rob_name,'()'));
               app_rob.name = sub_chains{i}.name;
               app_rob.model3d = sub_chains{i}.model3d;
               app_rob.links = sub_chains{1, 1}.links; % this line is fundamental whe we want to use a symbolic KINEMATIC model with frne (mex rne)
               %app_rob.robot2 = app_rob.nofriction('all');
               obj.sub_chains_vis{i} = sub_chains{i};
               obj.sub_chains{i} = app_rob;
               obj.symbolic_flag(i) = 1;
               
            else   
               obj.sub_chains{i} = sub_chains{i};
               obj.sub_chains_vis{i} = sub_chains{i};
               obj.symbolic_flag(i) = 0;
            end
            
            if(length(varargin) == 1)
                obj.whole_system = varargin{1};
            elseif(length(varargin) == 2);
                obj.whole_system = varargin{1};
                obj.floating_base = true;
            end
            
         end
      end
      %% TODO
      % this management of multiple chain has to changed in favor of a 
      % a unique system to refer to to compute the dynamics component of
      % the robot
      % get active subchain
      function rob = GetCurRobot(obj,current_chain)
          rob = obj.sub_chains{current_chain};
      end
      
       % get active subchain for visualization
      function rob = GetCurRobotVis(obj,current_chain)
          rob = obj.sub_chains_vis{current_chain};
      end
      %% end
      
      % number of subchains
      function n=GetNumChains(obj)
         n=size(obj.target_link,1);
      end
       
       % number of task in the subchain
       function n=GetNumTasks(obj,ind_subchain)
          n=size(obj.target_link{ind_subchain},2);
       end 
       
       % number of total degrees of freedom of the current subchain
       function n=GetNumLinks(obj,ind_subchain)
          n=obj.sub_chains{ind_subchain,1}.n;
       end 
       % number of the link that is used as e-e
       function n=GetNumSubLinks(obj,ind_subchain,ind_task)
          n=obj.target_link{ind_subchain}(ind_task);
       end 
       
       %% dynamic interface
       
       function M = GetM(obj,q)
           if(~isempty(obj.whole_system))
               M = obj.whole_system.inertia(q);
           else
               % TODO i need to build the inertia matrix of the overall
               % system
           end
       end
       
       function F = GetF(obj,q,qd,fc,Jc_t)
           % in the case i compute the F using an external object like icub
           if(~isempty(obj.whole_system))
               F = obj.whole_system.F(q,qd,fc,Jc_t);
           else
               % TODO i need to build the coriolis and gravload of the
               % overall system before computing it
               F = coriolis(q,qd)'*qd + gravload(q) - Fc;
           end
           
       end
       
       % i do not need to remove F because is contained in F;
       function [M,F,Jc_t] = RemoveFloatingBase(obj,M,F,start) 
           % here i make the hypotesis that the floating base part is in
           % the upper part of the dynamic matrices
           M = M(start:end,:);
           F = F(start:end,:);        
       end
      
    end
      
    
end