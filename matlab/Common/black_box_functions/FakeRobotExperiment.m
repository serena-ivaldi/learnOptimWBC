% order of option in this specific instance of the problem


% simulator        % rbt v-rep
% qinit            % initial position
% qdinit           % initial velocity
% time_sym_struct  %time struct for simulation with fixed step
% controller       % structure that contains every information about the specific instance of the problem
% fixed_step       % if is true i use ode4 (runge-kutta)
% options          % options



function output = FakeRobotExperiment(obj,parameters)

simulator         = obj.input_4_run{1}; % rbt or v-rep
controller        = obj.input_4_run{4}; % structure that contains every information about the specific instance of the problem
params            = obj.input_4_run{2};
% update of the parameters of activation functions and some reference
% (if they are optimized)
controller.UpdateParameters(parameters)

tic
[t, q, qd]=FakeDynSim_iCub(controller,params);
toc
%toc(controller.current_time) for debugging the time deadline


output{1} = t;
output{2} = q;
output{3} = qd;

end
