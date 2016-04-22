
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
       
    end
    
    
    methods
  
      function obj = SubChains(target_link,sub_chains)        
         
         obj.target_link = target_link;
         
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
            
            
         end
      end
      
      % get active subchain
      function rob = GetCurRobot(obj,current_chain)
          rob = obj.sub_chains{current_chain};
      end
      
       % get active subchain for visualization
      function rob = GetCurRobotVis(obj,current_chain)
          rob = obj.sub_chains_vis{current_chain};
      end
      
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
       
       
       
      
    end
      
    
end