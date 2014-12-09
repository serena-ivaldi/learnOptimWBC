% in this function i compute the mean error of the whole trajectory and i
% compute the effect of a repulsive point repsect of the elbow
%at this point the parameter of the fitness are definend inside the
%function itself
% work with scenario 2
function fit = fitness2(obj,t,q)
global G_OB;
traj_err= 0;
repuls  = 0;
alpha = 0.5;
downsaple = 1;
tot_sample = size(t,2)/downsaple;
contr = obj.controller;

for i=1:downsaple:size(t,2)
    q_cur = q{1}(i,:);
    % compute the trajectory error (mean square average)
    kinematic=strcat('contr.subchains.sub_chains{1}.T0_',num2str(contr.subchains.GetNumSubLinks(1,1)),'(q_cur)');
    T = eval(kinematic);
    ee = T(1:3,4);
    kinematic=strcat('contr.subchains.sub_chains{1}.T0_',num2str(contr.subchains.GetNumSubLinks(1,2)),'(q_cur)');
    T = eval(kinematic);
    elbow = T(1:3,4);
    
    traj_err = traj_err + norm((ee' - contr.references.GetTraj(1,1,t(i))))/tot_sample + norm((elbow' - contr.references.GetTraj(1,2,t(i))))/tot_sample;
    % compute the repulsive component (average) 1/r^2
    dist_e_e =  G_OB(1).dist(ee');
    dist_elbow =G_OB(1).dist(elbow');
    if(dist_e_e<=G_OB(1).tol || dist_elbow<=G_OB(1).tol )
       disp('hit the obstacle')
       error('hit the obstacle!')
    end   
    repuls= repuls + (1/dist_e_e)/tot_sample + (1/dist_elbow)/tot_sample;

end

fit = -(alpha)*traj_err - (1-alpha)*repuls;
end