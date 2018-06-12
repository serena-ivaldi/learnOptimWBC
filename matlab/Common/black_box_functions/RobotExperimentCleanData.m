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

if(strcmp(obj.input_4_run{1},'rbt'))
    controller = obj.input_4_run{5};
elseif(strcmp(obj.input_4_run{1},'icub_matlab'))
    % at the end of each iteration i need to reset rotation value in case
    % of numerical error
    wbm_resetWorldFrame();
    % here i ahve to reset the contact state of the robot 
    obj.input_4_run{1, 2}.contact_sym.state = obj.input_4_run{1, 2}.init_contact_state;
    obj.input_4_run{1, 2}.feet_on_ground    = obj.input_4_run{1, 2}.init_contact_state;
    obj.input_4_run{1, 2}.numContacts       = obj.input_4_run{1, 2}.contact_sym.UpdateContact();
    controller = obj.input_4_run{4};
elseif(strcmp(obj.input_4_run{1},'icub_matlab_sim'))
    setenv('LD_LIBRARY_PATH', obj.input_4_run{2}.new_matlab_LD_LIBRARY_PATH);
    system('gz world -r')
    setenv('LD_LIBRARY_PATH', obj.input_4_run{2}.matlab_LD_LIBRARY_PATH);
    controller = obj.input_4_run{4};
end
controller.CleanTau();
controller.CleanTime();
controller.current_time = [];
if(strcmp(obj.input_4_run{1},'rbt'))
    obj.input_4_run{5} = controller;
elseif(strcmp(obj.input_4_run{1},'icub_matlab'))
    obj.input_4_run{4} = controller;
end


end
