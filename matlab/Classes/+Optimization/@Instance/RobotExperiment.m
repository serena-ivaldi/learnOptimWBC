% order of option in this specific instance of the problem


% simulator        % rbt v-rep
% qinit            % initial position 
% qdinit           % initial velocity
% time_sym_struct  %time struct for simulation with fixed step
% controller       % structure that contains every information about the specific instance of the problem
% fixed_step       % if is true i use ode4 (runge-kutta)
% options          % options 



function output = RobotExperiment(obj,parameters)

    simulator       = obj.input_4_run{1}; % rbt v-rep
    qinit           = obj.input_4_run{2}; % initial position 
    qdinit          = obj.input_4_run{3}; % initial velocity
    time_sym_struct = obj.input_4_run{4}; %time struct for simulation with fixed step
    controller      = obj.input_4_run{5}; % structure that contains every information about the specific instance of the problem
    fixed_step      = obj.input_4_run{6}; % if is true i use ode4 (runge-kutta)
    %options         = input{7}; % options 
    


    controller.UpdateParameters(parameters)

    if(strcmp(simulator,'rbt'))
        %tic 
        [t, q, qd]=DynSim(time_sym_struct,controller,qinit,qdinit,fixed_step);
        %toc 
    end

    output{1} = t;
    output{2} = q;
    output{3} = qd;
    
end