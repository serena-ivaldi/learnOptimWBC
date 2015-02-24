classdef  Constraints < handle
    
   properties
     bot                  % current bot used for the simulation
     target_link          % vector that define wich kind of link i want to control with the e-e effector too (row vector) one for every kinematic chain
     constraints_list     % in this vector  define the set of constraints per each kinematic chain (torquelimit,vellimit,obsavoid)
     constraints_data     % row vector of column vector of data per each constraint
     constraints_handle   % cell vector of function for computing function
     number_of_constraint % number of constraints
   end


   methods
      
       function obj = Constraints(rob,target_link,constraints_list,constraints_data,varargin)
            
           rob_name = rob{1}.name;
           rob_name(rob_name==' ')=[];
           if find(strcat(rob_name,'_done.m'))
               app_rob = eval(strcat(rob_name,'()'));
               app_rob.name = rob{1}.name;
               app_rob.model3d = rob{1}.model3d;
               obj.bot = app_rob;
           else   
               obj.bot = rob{1};
           end
           obj.target_link = target_link;
           obj.constraints_list = constraints_list;
           obj.constraints_data = constraints_data;
           obj.number_of_constraint = size(constraints_list,2);
           for i=1:obj.number_of_constraint
               
               switch constraints_list{i}
                   case 'torquelimit' 
                        obj.constraints_handle{1,i}=@TorqueLimit;
                   case 'vellimit'
                        obj.constraints_handle{1,i}=@VelocityLimit; 
                   case 'obsavoid'
                        obj.constraints_handle{1,i}=@ObstacleAvoidance;    
                   otherwise 
                   error('Unexpected structure for repulsor');
               end  
               
               
           end
         
       end    

      
       
       
       function [g,hi]=GetConstrValue(obj,constr_index,DOF,delta_t,n_of_task,J_list,Projector_list,q,qd,cp)
           
           [g,hi] = obj.constraints_handle{constr_index}(obj,obj.constraints_data(:,constr_index),DOF,delta_t,n_of_task,J_list,Projector_list,q,qd,cp);
     
       end

       % every constraint function has to give back the row of the A matrix
       % (matrix of disequality) and a piece of vector  b that define the
       % value of the constraint
        
       function [g,hi]=TorqueLimit(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,q,qd,cp)
       % 2 parameters 
       %param(1,1) = 1 or 0. if 1 upper bound if 0 lower bound
       %param(2,1) = absolute value of the bound 
       g = zeros(DOF,DOF*(n_of_task+1));
       hi= ones(DOF,1);
       I = eye(DOF);
       if(param(1,1))
            g(:,1:DOF) = I;   
            hi = param(2,1)*hi;
       else
            g(:,1:DOF) = -I; 
            hi = param(2,1)*hi;
       end
           
           
       end  
       
       function [g,hi]=VelocityLimit(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,q,qd,cp)
           % 2 parameters 
           %param(1,1) = 1 or 0. if 1 upper bound if 0 lower bound
           %param(2,1) =absolute value of the bound  
           g = zeros(DOF,DOF*(n_of_task+1));


           if(param(1,1))

                index = DOF;
                for i=1:n_of_task
                    g(:,index + 1:index + DOF) = Projector_list{i};   
                    index = index + DOF;
                end
                hi = (param(2,1)*ones(DOF,1)-qd')*delta_t;
           else
               index = DOF;
               for i=1:n_of_task
                    g(:,index + 1:index + DOF) = -Projector_list{i};   
                    index = index + DOF;      
                end
                hi = (param(2,1)*ones(DOF,1)-qd')*delta_t;
           end
       
           
       end  
       
       function [g,hi]=ObstacleAvoidance(obj,param,DOF,delta_t,n_of_task,J_list,Projector_list,q,qd,cp)
           global G_OB;
           % 2 parameters 
           %param(1,1) = obstacle in the world 
           %param(2,1) = task involved in the obstacle avoidance defined by the number of the joints (ex: in lbr e-e = 7 elbow = 3) 
           g = zeros(1,DOF*(n_of_task+1)); 
          
           
           % check if the jacobian of the control point is precomputed to
           % if not i have to compute from scratch the jacobian
           kin_index=find(obj.target_link{1}==param(2,1),1);
           
           if(~isempty(kin_index))
              % i take the first index because i dont need the other
              cur_J = J_list{kin_index};
              
              normal_dist = G_OB(param(1,1)).Normal(cp(:,kin_index)')';
              cur_distance = G_OB(param(1,1)).Dist(cp(:,kin_index)',2);
              % i need this check because if the first task is a posture
              % task and i want to do obs avoidance with the end effector i
              % will take a complete jacobian with the orientation part too
              if(size(cur_J,1)>3)
                 cur_J = cur_J(1:3,:);
              end
           else
              [cur_J,cur_cp] = obj.ControlPointJacob(param(2,1),q);
              normal_dist = G_OB(param(1,1)).Normal(cur_cp')';
              cur_distance = G_OB(param(1,1)).Dist(cur_cp',2);
           end
           % the minus in the nexet line assures that the normal vector
           % point from the end_effector to the surface and not vice versa
           % as a normal vector of a surface
           hortho_distance_per_J= - normal_dist*cur_J;
           index = DOF;
           
           for i=1:n_of_task
                g(:,index + 1:index + DOF) = hortho_distance_per_J*Projector_list{i};   
                index = index + DOF;
           end
           %DEBUG
%            MaxAllowVel3(cur_distance,delta_t)
%            hortho_distance_per_J*qd'
           %---
           
           hi = ( MaxAllowVel3(cur_distance,delta_t)  - hortho_distance_per_J*qd')/delta_t;
       end  
      
     
   end
    
end

function val=MaxAll(obj_dist)
          %TODO pass tresh from outside
          Max_vel = 1;
          tresh = 0.02;
           val=-tresh+obj_dist;
          if(norm(val,1)<0.001)
              val = Max_vel;     
          end
          val = Max_vel+val;
           
end


function val=MaxAllowVel(dist,delta_t)
          
   val = (exp((dist-0.1)/delta_t))*0.5 - (exp((-dist-0.1)/delta_t))*0.49;      
end

function val=MaxAllowVel1(dist,delta_t)
          
   val =  dist^3/delta_t;    
end

function val=MaxAllowVel2(dist,delta_t)
          
       if dist<0.09
          val = -1;
       else
          val = 10000;
       end
          
end

function val=MaxAllowVel3(dist,delta_t)
          
       if dist<1
          val = -1000*log(1.1 - dist);
       else
          val = 2000;
       end
          
end



