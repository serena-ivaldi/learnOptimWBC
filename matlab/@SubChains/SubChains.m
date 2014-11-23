% SubChains naturally contains the robot without perturbation
% SubChains became really usefull when i have DIFERRENT kinematic chains
classdef  SubChains < handle


    properties
       target_link;    %  cell array of vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
       sub_chains;     %  cell array of kinematic model symbolic and not 
       symbolic_flag;  %  is true if i have a symbolic counterpart fo the kinematic chains
       
    end
    
    
    methods
 
      % to copy the whole robot we can use the copy constructor of SerialLink 
      % r1 = SerialLink(r0)   
      function obj = SubChains(target_link,sub_chains)        
         
         obj.target_link = target_link;
         
         for i = 1:size(target_link,1)
            
            rob_name = sub_chains{i}.name;
            rob_name(rob_name==' ')=[];
            
            if find(strcat(rob_name,'_done.m'))
               obj.sub_chains{i} = CodeGenerator(sub_chains{i});
               obj.symbolic_flag(i) = 1;
            else   
               obj.sub_chains{i} = sub_chains{i};
               obj.symbolic_flag(i) = 0;
            end
            
            
         end
      end
      
      function n=GetNumChains(obj)
         n=size(obj.target_link,2);
      end
       
       function n=GetNumTasks(obj,ind_subchain)
          n=size(obj.target_link{ind_subchain},2);
       end 
       
       function n=GetNumLinks(obj,ind_subchain)
          n=obj.sub_chains{ind_subchain}.n;
       end 
       
       function n=GetNumSubLinks(obj,ind_subchain,ind_task)
          n=obj.target_link{ind_subchain}(ind_task);
       end 
       
      
    end
      
    
end