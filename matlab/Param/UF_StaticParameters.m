disp('UF_STATICPARAM')

%%%;;

% REPELLER PARAMETERS
% scenario dependant
rep_subchain = [7];
rep_target_link{1} = rep_subchain;
rep_type = {'cartesian_x'};
rep_mask {1,1}=[1,1,1]; 
rep_type_of_J_rep = {'DirectionCartesian'};
for ii=1:chains.GetNumChains()
    chain_dof(ii) = chains.GetNumLinks(ii);
end



%CONTROLLER PARAMETERS
% the metric change between regularized and not regularized because in the
% regularized case i have to do N^(-1) 
% not regularized case i have N^(-1/2)
metric = {'M','M','M','M'};  % ex: if N = M^(-1) so N^(-1/2) = (M^(-1))^(-1/2) = M^(1/2);        


kp = [50,1,5,700]; % row vector one for each chain
kd = [2*sqrt(kp),2*sqrt(kp),2*sqrt(kp),2*sqrt(kp)];

for i= 1:chains.GetNumChains()
   for par = 1:chains.GetNumTasks(i)
       K_p = kp(i,par)*eye(size(dim_of_task{i,par},1));  
       K_d = kd(i,par)*eye(size(dim_of_task{i,par},1)); 
       Kp{i,par} = K_p;
       Kd{i,par} = K_d;
   end
   
end



%%%EOF


%% DO NOT MODIFY THIS PART 

rawTextFromStoragePart = fileread(which(mfilename));
rawTextFromStoragePart = regexp(rawTextFromStoragePart,['%%%;;' '(.*?)%%%EOF'],'match','once');