



classdef ParamIdentification < handle
   
   properties
      
      bot        % symbolic robot
      dh         % matrix of denavit hartberg parameter 'd' 'a' 'alpha'
      dyn_param  % matrix of parameter 'I','r','m','qlimdown','qlimup','Bdown','Bup','Tc','Jm','G'
      
      R          % array of Rotation matrix
      Jv         % array of jacobian matrix (traslational part)
      Jw         % array of jacobian matrix (rotational part)
      
      
      q_t        % symbolic joint position function of time q(t)
      qdiff_t    % symbolic joint veolcities function of time qd(t)  
      qddiff_t   % symbolic joint accelerations function of time qdd(t)
      
      q          % symbolic joints position (column vector)
      qd         % symbolic joints velocities (column vector)
      qdd        % symbolic joints accelerations (column vector)
      dof        % number of degree of freedom
      tensor     % tensor of third order 
      regressor  % symbolic expression of the regressor matrix
   end
   
   
   methods
      
      function obj=ParamIdentification()
         
         
         
         [obj.q,obj.qd,obj.qdd]=obj.bot.gencoords();
      
         [obj.q_t,obj.qdiff_t,obj.qddiff_t] = GenTimeCoord(obj.bot.n);
         E1 = [1,0,0;0,0,0;0,0,0];
         E2 = [0,1,0;1,0,0;0,0,0];
         E3 = [0,0,1;0,0,0;1,0,0];
         E4 = [0,0,0;0,1,0;0,0,0];
         E5 = [0,0,0;0,0,1;0,1,0];
         E6 = [0,0,0;0,0,0;0,0,1];
         obj.tensor={E1,E2,E3,E4,E5,E6};
         
      end
      
      
      function ComputeRegressor(obj)
         
         obj.regressor = [];
         for i = 1:obj.dof
            
            Y = obj.ComputeYi(i);
            obj.regressor = [obj.regressor Y];
         end
         
      end
      
      
      function Y = ComputeYi(obj,i)
         
         [Xd_zero, Xd_one, Xd_two]=obj.ComputeXdi(obj.R{i},obj.Jv{i},obj.Jw{i});
         [Wzero, Wone, Wtwo]=obj.ComputeWi(obj.R{i},obj.Jv{i},obj.Jw{i});
         [Z_zero, Z_one] = obj.ComputeZi(obj.R{i},obj.Jv{i});
         
         Y0 = Xd_zero - Wzero + Z_zero;
         Y1 = Xd_one  - Wone  + Z_one;
         Y2 = Xd_two  - Wtwo;
         
         Y = [Y0, Y1 , Y2];
             
      end
      
      
      function [Xd_zero, Xd_one, Xd_two]=ComputeXdi(obj,R_i,J_vi,J_wi)
      
         sym t;
         
         % substituions for computing the derivative of the expression
         % respect of the time
         Rt_i = VecSubs(R_i,obj.q,obj.q_t);
         Jt_vi= VecSubs(J_vi,obj.q,obj.q_t);
         Jt_wi= VecSubs(J_wi,obj.q,obj.q_t);
         
         % compute Xd_0
         
         A = (Jt_vi'*Jt_vi)*obj.qdiff_t;
         Xd_zero = diff(A,t);
         
         % substituions
         Xd_zero = VecSubs(Xd_zero,obj.q_t,obj.q);
         Xd_zero = VecSubs(Xd_zero,obj.qdiff_t,obj.qd);
         Xd_zero = VecSubs(Xd_zero,obj.qddiff_t,obj.qdd);
         
         % compute Xd_1
         
         B = ( Jt_vi'*skew(Jt_wi*obj.qdiff_t) - Jt_wi'*skew(Jt_vi*obj.qdiff_t) )*Rt_i;
         Xd_one = diff(B,t);
         
         % substituions
         Xd_one = VecSubs(Xd_one,obj.q_t,obj.q);
         Xd_one = VecSubs(Xd_one,obj.qdiff_t,obj.qd);
         Xd_one = VecSubs(Xd_one,obj.qddiff_t,obj.qdd);
         
         
         % compute Xd_2
         
         C_left = Jt_wi'*Rt_i;
         C_right = C_left'*obj.qdiff_t;
         for i=1:6
            C(:,i) = C_left*obj.tensor{i}*C_right;
         end
         
         
         Xd_two = diff(C,t);
         
          % substituions
         Xd_two = VecSubs(Xd_two,obj.q_t,obj.q);
         Xd_two = VecSubs(Xd_two,obj.qdiff_t,obj.qd);
         Xd_two = VecSubs(Xd_two,obj.qddiff_t,obj.qdd);
            
      end
      
      % in the lagrange expression derivative of the kinetic energy
      % respect to q. one for each link;
      function [Wzero, Wone, Wtwo]=ComputeWi(obj,R_i,J_vi,J_wi)
         
         %Compute W0 
         A = (J_vi')*J_vi;
         Ad=MatrixDerivative(A,obj.q);
         for i =1:obj.dof
            Wzero(i) =  1/2*obj.qd'*Ad{i}*obj.qd;
         end
         
         % compute W1
         B = R_i'*skew(J_wi*obj.qd)'*J_vi*obj.qd - R_i'*skew(J_vi*obj.qd)'*J_wi*obj.qd;
         Bd = MatrixDerivative(B',obj.q);
         for i =1:obj.dof
            Wone(i,:) = 1/2*Bd{i};
         end
         
         % compute W2
         
         C_left  = J_wi'*R_i;
         C_right = (C_left)';
         for i=1:6
            C(1,i) = 1/2 * obj.qd' * C_left * obj.tensor{i} * C_right * obj.qd;
         end
         
         Wtwo = jacobian(C,obj.q)';
         
      end
      
      function   [Z_zero, Z_one] = ComputeZi(obj,R_i,J_vi)
         
         %column vector  
         g = obj.rob.gravity;
         
         %compute Z0
         Z_zero = - J_vi' * g;
         
         % compute Z1
         A = R_i' * g;
         A = A';
         
         Ad=MatrixDerivative(A,obj.q);
         for i =1:obj.dof
            Z_one(i,:) = -Ad{i};
         end
         
      end
      
   end

end


function [q_t,qdiff_t,qddiff_t] = GenTimeCoord(n)

    sym t
    if nargout > 0
        for j=1:n
            q_t(j) = sym( sprintf('q%d(t)', j), 'real' );
            qdiff_t(j) = diff(q_t(j),t);
            qddiff_t(j)   = diff(qdiff_t(j),t);
        end
    end

end


function [Md]=MatrixDerivative(M,x)

   number_of_matrix = length(x);
   Md = cell(number_of_matrix,1);
   % vector obtained by stacking each row in one big row
   vecM = M(:)';

   % each row of this matrix is the gradient of each element of the 
   % intial matrix taken per row.
   % each column of matrix_of_gradient is the original matrix derivate 
   % by one variable.
   matrix_of_gradient = jacobian(vecM,x);

   % to extract from matrix_of_gradient the original matrix derivate 
   % for each variable i have to take each column of matrix_of_gradient
   % and reshape it counting how many element i have in row of the 
   % original matrix
   for i=1:size(matrix_of_gradient,2)

      column = matrix_of_gradient(:,i);
      Md{i} = reshape(column',size(M));
      
   end

end


function new_s=VecSubs(s,old,new)

   len = length(old);
   new_s = s;
   
   for i=1:len
      new_s = subs(s,old(i),new(i));
   end
   
end


