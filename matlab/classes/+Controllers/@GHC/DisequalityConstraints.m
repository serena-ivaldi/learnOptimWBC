function [A,b] = DisequalityConstraints(obj,DOF,n_of_task,delta_t,J_list,projector_list,qd,cp)
    A=[];
    b=[];
    for i = 1:obj.constraints.number_of_constraint
      
        [app_A app_b] = obj.constraints.GetConstrValue(i,DOF,delta_t,n_of_task,J_list,projector_list,qd,cp);
        A = [A;app_A];
        b = [b;app_b];

    end


end