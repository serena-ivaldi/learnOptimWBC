% in this function i compute the mean error of the whole trajectory and i
% compute the effect of a repulsive point repsect of the elbow
%at this point the parameter of the fitness are definend inside the
%function itself
% work with scenario 1
function fit = fitness1(obj,t,q)
global G_OB;
alpha = 0.999;
downsaple = 10;
tot_sample = size(t,2)/downsaple;
traj_err= 0;
repuls  = 0;
contr = obj.controller;

for i=1:downsaple:size(t,2)
    q_cur = q{1}(i,:);
    % compute the trajectory error (mean square average)
    kinematic=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
    T = eval(kinematic{1});
    ee = T(1:3,4);
    attr_pos = contr.references.GetTraj(1,1,t(i)); 
    traj_err = traj_err + norm((ee - attr_pos))/tot_sample;
    % compute the repulsive component (average) 1/r^2
    kinematic=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,2)),{'(q_cur)'});
    T = eval(kinematic{1});
    elbow = T(1:3,4);
    dist = G_OB(1).dist(elbow);
    repuls= repuls + ((1/dist)^2)/tot_sample;

end


fit = -(alpha)*traj_err - (1-alpha)*repuls;
end