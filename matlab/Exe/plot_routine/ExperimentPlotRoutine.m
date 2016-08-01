function [alpha_,fitness_,torque_,cartesian_position_,joint_position_]=ExperimentPlotRoutine(complete_path,input,time_struct,bestAction,bot1,learn_approach,interpolation_step)
    % conversion from rad to deg
    RAD = 180/pi;
    % global variables for the function
    simulator         = input{1}; % rbt or v-rep
    controller = [];
    time_sym_struct = [];
    p = [];
    pd = [];
    p_r = [];
    pd_r = [];
    p_l = [];
    pd_l = [];
    if(strcmp(simulator,'rbt'))
         %% compute the joint position and velocity
        qinit             = input{2}; % initial position
        qdinit            = input{3}; % initial velocity
        time_sym_struct   = input{4}; %time struct for simulation with fixed step
        controller        = input{5}; % structure that contains every information about the specific instance of the problem
        fixed_step        = input{6}; % if is true i use ode4 (runge-kutta)
        torque_saturation = input{7};
        maxtime           = input{8};
        %options         = input{7}; % options

        % update of the parameters of activation functions and some reference
        % (if they are optimized)
        controller.UpdateParameters(bestAction.parameters)

        tic
        [t, q, qd] = DynSim(time_sym_struct,controller,qinit,qdinit,fixed_step,'TorqueSat',torque_saturation,'maxtime',maxtime);
        toc
        %toc(controller.current_time) for debugging the time deadline
        % generate cartesian position and cartesian velocity
        for ii =1:size(q{1},1) % generalize to multichain
            T =controller.subchains.sub_chains{1}.fkine(q{1}(ii,:)); % generalize to multichain
            p = [p;T(1:3,4)'];
            J = controller.subchains.sub_chains{1}.jacob0(q{1}(ii,:)); % generalize to multichain
            pd_cur = J*(qd{1}(ii,:))';% generalize to multichain
            pd = [pd ; pd_cur'];  
        end
    elseif (strcmp(simulator,'icub_matlab'))
        %% compute the joint position and velocity
        % due to some change in the code i have to set the floating base
        % properties
        input{2}.active_floating_base = false;
        input{2}.numContacts = sum(input{2}.feet_on_ground,2);
        input{2}.contactLinkNames={'l_sole','r_sole'};
        % for some reason the icub robot does not work if  i save it in a file 
        % so i have to create it again
        bot2 = iCub(bot1.model_name);
        controller = input{4}; % structure that contains every information about the specific instance of the problem
        controller.visual_param.fc = [];
        % here i subistitute the icub with the new one
        bot1 = bot2;
        controller.subchains.whole_system = bot2;
        time_sym_struct = input{3};
        % update of the parameters of activation functions and some reference
        % (if they are optimized)
        controller.UpdateParameters(bestAction.parameters)

        tic
        [t, q, qd]=DynSim_iCub(controller,input{2});
        toc
        %toc(controller.current_time) for debugging the time deadline
        for ii =1:size(q,1) 
            cur_q = q(ii,:);
            cur_qd = qd(ii,:);
            p_cur_r = bot1.offlineFkine(cur_q','r_hand'); % generalize to multichain
            p_r = [p_r;p_cur_r'];
            J_r = bot1.offlineJacob0(cur_q','r_hand'); % generalize to multichain
            pd_cur_r = J_r*(cur_qd)';% generalize to multichain
            pd_r = [pd_r ; pd_cur_r'];  
            p_cur_l = bot1.offlineFkine(cur_q','l_hand'); % generalize to multichain
            p_l = [p_l;p_cur_l'];
            J_l = bot1.offlineJacob0(cur_q','l_hand'); % generalize to multichain
            pd_cur_l = J_l*(cur_qd)';% generalize to multichain
            pd_l = [pd_l ; pd_cur_l'];
        end
    end
    %% compute the interpolated torque    
    tau_=InterpTorque(controller,time_sym_struct,interpolation_step);
    % copy best action inside the new folder
    fileID = fopen(strcat(complete_path,'/','best_action.txt'),'wt');
    fprintf(fileID,'%f ',bestAction.parameters);
    fclose(fileID);

    %% plot parameters
    dim_tit=16;
    dim_lab=14;
    dim_leg=12;
    %% plot alpha 
    time = time_struct.ti:time_struct.step:time_struct.tf;
    vec_values = zeros(size(time));
    handle_vec = [];
    for ii = 1:size(controller.alpha,1)
        for jj = 1:size(controller.alpha,2)         
            i=1;
            for t = time
                vec_values(i) = controller.alpha{ii,jj}.GetValue(t); 
                i=i+1;
            end
            handle_vec(ii,jj) = figure;
            plot(time,vec_values);
            grid on;
            xlab = xlabel('t [s]');
            set(xlab,'FontSize',dim_lab,'Interpreter','latex');
            ylab = ylabel(strcat('\alpha_{',num2str(ii),num2str(jj),'}'));
            set(ylab,'FontSize',dim_lab);      
        end
    end
    % save alpha plot fig and jpg
    for ii = 1:size(controller.alpha,1)
        for jj = 1:size(controller.alpha,2)
          saveas(handle_vec(ii,jj),strcat(complete_path,'/','alpha',num2str(ii),num2str(jj)),'pdf');
          saveas(handle_vec(ii,jj),strcat(complete_path,'/','alpha',num2str(ii),num2str(jj)));
        end
    end
    %% plot of fitnes function
    evolutions = size(bestAction.hist,2);
    % remove failed mean and failed variance
    % -10000000 is the penalty that is applied when i have a failure (not anymore to change)
    index = 1;
    evo = 1;
    mean = 0;

    if(strcmp(learn_approach,'CMAES'))
       for ww =1:evolutions
           % i discard the evolutions with failure final mean
           if(bestAction.hist(1,ww).performance>-1)
               listperformance = bestAction.hist(1,ww).listperformance;
               %remove all the failure from the computation of the variance
               listperformance = listperformance(listperformance~=-1);
               variance(index) = var(listperformance);
               mean(index) = bestAction.hist(1,ww).performance;
               evo(index) = index;
               index = index + 1; 
           end
       end
    elseif(strcmp(learn_approach,'(1+1)CMAES'))  
      for ww =1:evolutions 
         if(bestAction.listperformance(ww)> -1 )
            %remove all the failure from the computation of the variance
            variance(index) = 0;
            mean(index)= bestAction.listperformance(ww);
            evo(index) = index;
            index = index + 1;  
         end
      end
    end

    handle = figure;
    hold on;
    grid on;

    plot(evo,mean);
    %variance plot of cmaes not significant
    plot(evo, mean + 2 * variance, ':');
    plot(evo, mean - 2 * variance, ':');
    xlab = xlabel('evolutions');
    set(xlab,'FontSize',dim_lab,'Interpreter','latex');
    ylab = ylabel('fitness');
     set(ylab,'FontSize',dim_lab,'Interpreter','latex');
    saveas(handle,strcat(complete_path,'/','fit'),'pdf');
    saveas(handle,strcat(complete_path,'/','fit'));

    %%  plot x y z
    if(strcmp(simulator,'rbt'))
        handle = figure;
        plot(p(:,1:3));
        grid on;
        leg = legend('X','Y','Z');
        set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('cartesian position [m]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        saveas(handle,strcat(complete_path,'/','e_e'),'pdf');
        saveas(handle,strcat(complete_path,'/','e_e'));
    elseif (strcmp(simulator,'icub_matlab'))
        % left hand
        handle = figure;
        plot(p_l(:,1:3));
        grid on;
        leg = legend('X','Y','Z');
        set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('cartesian position [m]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        saveas(handle,strcat(complete_path,'/','e_e_l'),'pdf');
        saveas(handle,strcat(complete_path,'/','e_e_l'));
         % right hand
        handle = figure;
        plot(p_r(:,1:3));
        grid on;
        leg = legend('X','Y','Z');
        set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('cartesian position [m]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        saveas(handle,strcat(complete_path,'/','e_e_r'),'pdf');
        saveas(handle,strcat(complete_path,'/','e_e_r'));
    end
    %% plot q
    if(strcmp(simulator,'rbt'))
        handle = figure;
        plot(q{1}*RAD);
        grid on;
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('joints position [deg]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        for i = 1:size(q{1},2)
           str_name{i} = strcat('q_{',num2str(i),'}');
        end
        leg=legend(str_name);
        set(leg,'FontSize',dim_leg,'Location','best');
        saveas(handle,strcat(complete_path,'/','q'),'pdf');
        saveas(handle,strcat(complete_path,'/','q'));
    elseif (strcmp(simulator,'icub_matlab'))
        handle = figure;
        plot(q*RAD);
        grid on;
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('joints position [deg]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        for i = 1:size(q,2)
           str_name{i} = strcat('q_{',num2str(i),'}');
        end
        leg=legend(str_name);
        set(leg,'FontSize',dim_leg,'Location','best');
        saveas(handle,strcat(complete_path,'/','q'),'pdf');
        saveas(handle,strcat(complete_path,'/','q'));
    end
    %% plot tau
    if(strcmp(simulator,'rbt'))
        handle = figure;
        plot(tau_)
        grid on;
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('joints torque [N*m]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        for i = 1:size(q{1},2)
           str_name1{i} = strcat('U_{',num2str(i),'}');
        end
        leg=legend(str_name1);
        set(leg,'FontSize',dim_leg,'Location','best');
        saveas(handle,strcat(complete_path,'/','tau'),'pdf');
        saveas(handle,strcat(complete_path,'/','tau'));
    elseif (strcmp(simulator,'icub_matlab'))
        handle = figure;
        plot(tau_)
        grid on;
        xlab=xlabel('time [ms]'); % x-axis label
        set(xlab,'FontSize',dim_lab,'Interpreter','latex');
        ylab=ylabel('joints torque [N*m]'); % y-axis label
        set(ylab,'FontSize',dim_lab,'Interpreter','latex');
        for i = 1:size(q,2)
           str_name1{i} = strcat('U_{',num2str(i),'}');
        end
        leg=legend(str_name1);
        set(leg,'FontSize',dim_leg,'Location','best');
        saveas(handle,strcat(complete_path,'/','tau'),'pdf');
        saveas(handle,strcat(complete_path,'/','tau'));
    end
    %% return the value for the average results
    alpha_ = controller.alpha;
    fitness_ = mean;
    torque_ = tau_;
    if(strcmp(simulator,'rbt'))
        cartesian_position{1} = p;
    elseif (strcmp(simulator,'icub_matlab'))
        cartesian_position_{1} = p_l;
        cartesian_position_{2} = p_r; 
    end
    joint_position_ = q;
    %% close all the figure
    close all;

end





