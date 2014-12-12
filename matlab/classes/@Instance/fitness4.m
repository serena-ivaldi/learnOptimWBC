% this fitness work with scenario 3 e 4
function fit = fitness4(obj,t,q)
global G_OB;
downsaple = 10;
traj_err= 0;
repuls  = 0;
L = 1;
penalty = 10;
sigma = 0.1; 
contr = obj.controller;

for i=1:downsaple:size(t,2)
    q_cur = q{1}(i,:);
    % compute the trajectory error (absolute error)
    kinematic=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
    T = eval(kinematic{1});
    ee = T(1:3,4);
    attr_pos = contr.references.GetTraj(1,1,t(i)); 
    traj_err = traj_err + norm((ee - attr_pos),L);
    % compute the repulsive component 
    dist = G_OB(1).Dist(ee',L);
    repuls= repuls + penalty*exp(-(dist)/(2*sigma^(2)));

end

fit = -traj_err - repuls;
end