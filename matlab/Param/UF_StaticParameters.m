% REPELLER PARAMETERS
% scenario dependant
rep_subchain = [3 3];
rep_target_link{1} = rep_subchain;
rep_type = {'cartesian_x' 'cartesian_x'};
rep_mask {1,1}=[1,1,1]; rep_mask {1,2}=[1,1,1];
rep_type_of_J_rep = {'DirectionCartesian' 'DirectionCartesian'};
for ii=1:chains.GetNumChains()
    chain_dof(ii) = chains.GetNumLinks(ii);
end



%CONTROLLER PARAMETERS
% the metric change between regularized and not regularized because in the
% regularized case i have to do N^(-1) 
% not regularized case i have N^(-1/2)
metric = {'M'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        


kp = [700]; % row vector one for each chain
for i= 1:chains.GetNumChains()
   K_p = zeros(3,3,size(kp,2));
   K_d = zeros(3,3,size(kp,2));
   for par = 1:chains.GetNumTasks(i)
       K_p(:,:,par) = kp(i,par)*eye(3);  
       kd = 2*sqrt(kp(i,par));
       K_d(:,:,par) = kd*eye(3); 
   end
   Kp{i} = K_p;
   Kd{i} = K_d;
end


% INSTANCE PARAMETERS
fitness= @fitness6;