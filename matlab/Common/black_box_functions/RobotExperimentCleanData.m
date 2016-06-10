% order of option in this specific instance of the problem

% simulator        % rbt v-rep
% qinit            % initial position 
% qdinit           % initial velocity
% time_sym_struct  %time struct for simulation with fixed step
% controller       % structure that contains every information about the specific instance of the problem
% fixed_step       % if is true i use ode4 (runge-kutta)
% options          % options 


% when we design postprocessing function is necessary to add a fake input
% because without it matlab is gonna consider the function with solely obj as input
% as a method of the class where the obj belongs to
function RobotExperimentCleanData(obj,fake_input)

controller = obj.input_4_run{5};

controller.CleanTau();
controller.CleanTime();

obj.input_4_run{5} = controller;

end