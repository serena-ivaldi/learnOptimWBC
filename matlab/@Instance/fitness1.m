% in this function i compute the mean error of the whole trajectory and i
% compute the effect of a repulsive point repsect of the elbow
%at this point the parameter of the fitness are definend inside the
%function itself
function fit = fitness1(obj,t,q)

downsaple = 10;
tot_sample = size(t,2)/downsaple;
repuls_pos = [-0.15; -0.4000; 0.3100;];
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
    kinematic=strcat('contr.subchains.sub_chains{1}.T0_',num2str(contr.subchains.GetNumSubLinks(1,2)),'(q_cur)');
    T = eval(kinematic);
    elbow = T(1:3,4);
    repuls= repuls + (1/(norm(elbow - repuls_pos))^2)/tot_sample;

end


fit = traj_err - repuls
end