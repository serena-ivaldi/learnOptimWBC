% here i compute the matrix for the objective function

function [H,f,J_list]=ObjectiveFunction(obj,DOF,ind_subchain,q,qd)
  
% preallocate matrix 
  % plus 1 because i have to consider also the torque in the vector that we
  % want to optimize
  H=zeros(DOF*(obj.subchains.GetNumTasks(ind_subchain)+1)); 
  f=zeros(DOF,DOF*(obj.subchains.GetNumTasks(ind_subchain)+1));
  J_list=cell(GetNumTasks(ind_subchain),1);
  I_rel=regularizer(ind_subchain)*eye(7);
  O=zeros(7);
  
  
  % insert in H and f the term related to torque (in f we have zero so i dont have to add anything)
  H(1:DOF,:) = [O I_rel];
  
  ind = DOF+1;
  for j = 1 : obj.subchains.GetNumTasks(ind_subchain)
      
      [J_old,Jd_old,x,xd,rpy,rpyd] = obj.subchains.DirKin(q,qd,ind_subchain,j);
      [b,J,J_dot] = obj.ControlLaw(ind_subchain,j,t,J_old,Jd_old,x,xd,rpy,rpyd,q,qd);
      C = J_dot'-b;
      Cs=(C' + C)/2;
      f(:,ind:ind+DOF) = Cs*J;
      H(ind:ind+DOF,:) = [J'*J I_rel];
      J_list{j} = J;
  end


end