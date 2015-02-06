disp('GHC_STATICPARAM')

%% chained alpha 
matrix1 = [0 1 0;0 0 0;1 1 0];  % 2>1>3
matrix2 = [0 0 0;1 0 0;1 1 0];  % 1>2>3
matrix3 = [0 0 0;1 0 1;1 0 0];  % 1>3>2
matrix4 = [0 0 1;1 0 1;0 0 0];  % 3>1>2
matrix_value(:,:,1) = matrix1;
matrix_value(:,:,2) = matrix2;
matrix_value(:,:,3) = matrix3;
matrix_value(:,:,4) = matrix4;
ti =[2 6 8];


%% CONTROLLER PARAMETERS
% % row vector one for each chain
kp = [1000 1000 1000]; 
kd = [2*sqrt(kp) 2*sqrt(kp) 2*sqrt(kp)];

for i= 1:chains.GetNumChains()
  
   for par = 1:chains.GetNumTasks(i)
       K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
       K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
       Kp{i,par} = K_p;
       Kd{i,par} = K_d;
       
   end
   
end
%---

