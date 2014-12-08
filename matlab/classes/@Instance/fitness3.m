function fit = fitness3(obj,t,q)

alpha = 0.99;
downsaple = 10;
tot_sample = size(t,2)/downsaple;
repuls_pos = [-0.125538272258140; -0.5; 0.455692460313405];
traj_err= 0;
repuls  = 0;
contr = obj.controller;

for i=1:downsaple:size(t,2)
    q_cur = q{1}(i,:);
    % compute the trajectory error (mean square average)
    kinematic=strcat('contr.subchains.sub_chains{1}.T0_',num2str(contr.subchains.GetNumSubLinks(1,1)),'(q_cur)');
    T = eval(kinematic);
    ee = T(1:3,4);
    attr_pos = contr.references.GetTraj(1,1,t(i)); 
    traj_err = traj_err + norm((ee - attr_pos))/tot_sample;
    % compute the repulsive component (average) 1/r^2
    repuls= repuls + (1/(norm(ee - repuls_pos))^2)/tot_sample;

end


fit = -(alpha)*traj_err - (1-alpha)*repuls;
end