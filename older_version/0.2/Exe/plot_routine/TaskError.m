
% compute the obtained value 
q4err = q{1};
for i_= 1:size(reference.target_link{1},2) 
   o_v = [];
   for j_ = 1:size(q4err,1)
       if(strcmp(reference.type{i_},'cartesian_x'));
           q_cur = q4err(j_,:);
           kinematic=CStrCatStr({'controller.subchains.sub_chains{1}.T0_'},num2str(controller.subchains.GetNumSubLinks(1,i_)),{'(q_cur)'});
           T = eval(kinematic{1});
           o_v = [o_v T(1:3,4)];   
       else
           o_v =  [o_v q4err(j_,:)'];
       end   
   end
   computed_value{i_} = o_v;     
end

% compute error
all_results =[];
for k_ = 1:size(computed_value,2)
    
  ref_mat=repmat(reference.trajectories{1,k_}.p,1,size(q4err,1));  
  diff_mat =ref_mat - computed_value{k_};
  diff_mat = diff_mat.*diff_mat;
  result = sum(diff_mat,1);
  result = sqrt(result);
  all_results = [all_results result'];
    
end


plot(all_results);