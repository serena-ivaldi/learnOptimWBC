



classdef ParamIdentification < handle
   
   properties
      
      bot        % symbolic robot
      dh         % matrix of denavit hartberg parameter 'd' 'a' 'alpha'
      dyn_param  % 
      
      q          % symbolic joints position (column vector)
      qd         % symbolic joints velocities (column vector)
      qdd        % symbolic joints accelerations (column vector)
      dof        % number of degree of freedom
      tensor     % tensor of third order 
      regressor  % symbolic exrepssion of the regressor matrix
   end
   
   
   methods
      
      function obj=ParamIdentification()
         
         
         
         [q,qd,qdd]=obj.bot.gencoords();
         obj.q = q;
         obj.qd = qd;
         obj.qdd = qdd;
         E1 = [1,0,0;0,0,0;0,0,0];
         E2 = [0,1,0;1,0,0;0,0,0];
         E3 = [0,0,1;0,0,0;1,0,0];
         E4 = [0,0,0;0,1,0;0,0,0];
         E5 = [0,0,0;0,0,1;0,1,0];
         E6 = [0,0,0;0,0,0;0,0,1];
         obj.tensor={E1,E2,E3,E4,E5,E6};
         
      end
      
      function ComputeRegressor(obj)
         
         
         
             
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
      
      function   [Zzero, Zone] = ComputeZi(obj,R_i,J_vi)
         
         %column vector  
         g = obj.rob.gravity;
         
         %compute Z0
         Zzero = - J_vi' * g;
         
         % compute Z1
         A = R_i' * g;
         A = A';
         
         Ad=MatrixDerivative(A,obj.q);
         for i =1:obj.dof
            Zone(i,:) = -Ad{i};
         end
         
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



