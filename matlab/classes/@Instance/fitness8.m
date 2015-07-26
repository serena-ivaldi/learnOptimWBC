% this fitness work with scenario 

function fit = fitness8(obj,t,q)
    global G_OB;
    
    contr = obj.controller;
    traj_err= 0;     
    
    %%%;;
    downsaple = 10;
    L = 1;
    penalty = 5000; %10     % not used
    sigma = 0.1;            % not used
    max_effort = 2000;      % not used
    max_traj_error = 10000;
    max_penalties  = 20000; % not used
    weight_effort = 1;      % not used
    weight_traj_err = 1;    % not used
    weight_penal = 5;       % not used
    plot_subchain1 = [7];
    plot_target_link{1} = plot_subchain1;
    % reference parameters
    plot_type = {'cartesian_x'};
    plot_control_type = {'tracking'};
    plot_type_of_traj = {'func'};
    plot_traj = {'lemniscate'};
    plot_time_law = {'linear'};
    %parameters first chains
    plot_geom_parameters{1,1} = [1.2 pi/4 -pi/2 0 0 1.178];
    plot_dim_of_task{1,1}={[1;1;1]};

    %% reference
    % if type_of_task = sampled i have to specify the Time to reach the
    % end of the trajectories that is equal to the simulation time
    plot_reference = References(plot_target_link,plot_type,plot_control_type,plot_traj,plot_geom_parameters,plot_time_law,contr.references.time_struct,plot_dim_of_task,plot_type_of_traj);
    plot_reference.BuildTrajs();    
    %%%EOF
    

    for i=1:downsaple:size(t,2)
        q_cur = q{1}(i,:);
        % compute the trajectory error (absolute error)
        kinematic=CStrCatStr({'contr.subchains.sub_chains{1}.T0_'},num2str(contr.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
        T = eval(kinematic{1});
        ee = T(1:3,4);
        attr_pos = plot_reference.GetTraj(1,1,t(i)); 
        traj_err = traj_err + norm((ee - attr_pos),L);
    end
 
    
    
    % saturations 
    
    if(traj_err>max_traj_error)
       traj_err = max_traj_error;
    end
    
    traj_err  = traj_err/max_traj_error;
    
    %%DEBUG
    fprintf('traj error is %f\n', traj_err)
    %---
    fit = (-traj_err);
end