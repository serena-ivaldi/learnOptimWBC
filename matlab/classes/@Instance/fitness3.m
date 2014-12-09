% this fitness work with scenario 3 e 4
function fit = fitness3(obj,t,q)
global G_OB;
alpha = 0.99;
downsaple = 10;
tot_sample = size(t,2)/downsaple;
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
    dist = G_OB(1).dist(ee);
    repuls= repuls + (1/(dist)^2)/tot_sample;

end


fit = -(alpha)*traj_err - (1-alpha)*repuls;
end