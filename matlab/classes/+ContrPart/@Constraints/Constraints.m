classdef  Constraints < handle
    
   properties
     constraints_list     % in this vector  define the set of constraints per each kinematic chain (torquelimit,vellimit,obsavoid)
     constraints_data     % row vector of column vector of data per each constraint
     constraints_handle   % cell vector of function for computing function
     number_of_constraint % number of constraints
   end


   methods
      
       function obj = Constraints(constraints_list,constraints_data,varargin)
       
           obj.constraints_list = constraints_list;
           obj.constraints_data = constraints_data;
           obj.number_of_constraint = size(constraints_list,2);
           for i=1:obj.number_of_constraint
               
               switch constraints_list{i}
                   case 'torquelimit' 
                        constraints_handle{1,i}=@TorqueLimit;
                   case 'vellimit'
                        constraints_handle{1,i}=@VelocityLimit; 
                   case 'obsavoid'
                        constraints_handle{1,i}=@ObstacleAvoidance;    
                   otherwise 
                   error('Unexpected structure for repulsor');
             end  
               
               
           end
         
       end    

      
       
       
       function [g hi]=GetConstrValue(obj,constr_index,DOF,delta_t,n_of_task,J_list,Projector_list,qd,cp)
           
           [g hi] = obj.constraints_handle{constr_index}(obj,obj.constraints_data(constr_index),DOF,delta_t,n_of_task,J_list,Projector_list,qd,cp);
     
       end

       % every constraint function has to give back the row of the A matrix
       % (matrix of disequality) and a piece of vector  b that define the
       % value of the constraint
        
       function [g hi]=TorqueLimit(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,qd,cp)
       % 2 parameters 
       %param(1,1) = 1 or 0. if 1 upper bound if 0 lower bound
       %param(2,1) = value of the bound
       g = zeros(DOF,DOF*(n_of_task+1));
       hi= ones(DOF,1);
       I = eye(DOF);
       if(param(1,1))
            g(:,1:DOF) = I;   
            hi = param(2,1)*hi;
       else
            g(:,1:DOF) = -I; 
            hi = -param(2,1)*hi;
       end
           
           
       end  
       
       function [g hi]=VelocityLimit(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,qd,cp)
           % 2 parameters 
           %param(1,1) = 1 or 0. if 1 upper bound if 0 lower bound
           %param(2,1) = value of the bound  
           g = zeros(DOF,DOF*(n_of_task+1));


           if(param(1,1))

                index = DOF + 1;
                for i=1:n_of_task
                    g(:,index:index + DOF) = Projector_list{i};   
                    index = index + DOF;
                end
                hi = (param(2,1)*ones(dof,1)-qd')*delta_t;
           else
               index = DOF + 1;
               for i=1:n_of_task
                    g(:,index:index + DOF) = -Projector_list{i};   
                    index = index + DOF;      
                end
                hi = (param(2,1)*ones(dof,1)-qd')*delta_t;
           end
       
           
       end  
       
       function [g hi]=ObstacleAvoidance(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,qd,cp)
           global G_OB;
           % 2 parameters 
           %param(1,1) = obstacle in the world 
           %param(2,1) = task involved in the obstacle avoidance 
           current_task_dimension = size(J_list{param(2,1)},1);
           g = zeros(current_task_dimension,DOF*(n_of_task+1)); 

           index = DOF + 1;
           for i=1:n_of_task
                g(:,index:index + DOF) = G_OB(param(1,1)).normal(cp(param(2,1)))'*J_list{param(2,1)}*Projector_list{i};   
                index = index + DOF;
           end
           hi = (MaxAllowVel(G_OB(param(1,1)).dist) - G_OB(param(1,1)).normal(cp(param(2,1)))'*J_list{param(2,1)}*qd')/delta_t;
       end  
      
      
   end
    
end

function val=MaxAllowVel(obj_dist)
          %TODO pass tresh from outside
          tresh = 0.02;
          val = 1/(-tresh+obj_dist);

end




