%here i compute the matrix for the objective function

function [H,f,J_list,cp]=ObjectiveFunction(obj,DOF,ind_subchain,t,q,qd)
  
% preallocate matrix 
  % plus 1 because i have to consider also the torque in the vector that we
  % want to optimize
  H=zeros(DOF*(obj.subchains.GetNumTasks(ind_subchain)+1)); 
  app_mat_row = zeros(DOF,DOF*(obj.subchains.GetNumTasks(ind_subchain)+1));
  f=zeros(1,DOF*(obj.subchains.GetNumTasks(ind_subchain)+1));
  J_list=cell(obj.subchains.GetNumTasks(ind_subchain),1);
  I_rel=obj.regularizer(ind_subchain)*eye(7);
  % insert in H and f the term related to torque (in f we have zero so i dont have to add anything)
  app_mat_row(:,1:DOF) = I_rel;
  H(1:DOF,:) = app_mat_row;
  app_mat_row = zeros(DOF,DOF*(obj.subchains.GetNumTasks(ind_subchain)+1));
  % vector of control point associated to the joint position of te robot 
  % in the cartesian space
  cp = zeros(3,obj.subchains.GetNumTasks(ind_subchain));
  ind = DOF;
  for j = 1 : obj.subchains.GetNumTasks(ind_subchain)
      
      [J_old,Jd_old,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,j);
      cp(:,j) = x;
      [b,J,J_dot] = obj.ControlLaw(ind_subchain,j,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd);
      % J_dot is just multiplied by qd in obj.ControlLaw
      C = J_dot' - b';
      f(1,ind + 1:ind + DOF) = C*J;
      app_mat_row(:,ind + 1:ind + DOF) = J'*J+I_rel;
      H(ind + 1: ind + DOF,:) = app_mat_row;
      app_mat_row = zeros(DOF,DOF*(obj.subchains.GetNumTasks(ind_subchain)+1));
      J_list{j} = J;
      ind = ind + DOF;
  end


end


