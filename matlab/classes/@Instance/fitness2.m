% in this function i compute the mean error of the whole trajectory and i
% compute the effect of a repulsive point repsect of the elbow
%at this point the parameter of the fitness are definend inside the
%function itself
function fit = fitness2(obj,t,q)


[X,Y,Z]=meshgrid(-0.05:0.01:0.3,-0.4,0.31:0.01:0.7);
elbow_point = [-0.2 -0.4000 0.3100];
e_e_point = [0, -0.7,0.5100];
traj_err= 0;
repuls  = 0;
alpha = 0.5;
hit_tresh = 0.002;
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
    
    traj_err = traj_err + norm((ee' - e_e_point))/tot_sample + norm((elbow' - elbow_point))/tot_sample;
    % compute the repulsive component (average) 1/r^2
    dist_e_e = MinDist(X,Y,Z,ee');
    dist_elbow = MinDist(X,Y,Z,elbow');
    if(dist_e_e<=hit_tresh || dist_elbow<=hit_tresh )
       disp('hit the obstacle')
       error('hit the obstacle!')
    end   
    repuls= repuls + (1/dist_e_e)/tot_sample + (1/dist_elbow)/tot_sample;

end

fit = -(alpha)*traj_err - (1-alpha)*repuls;
end