%% here i make these hypothesis:
% 1)that the first k parameter in action control the shape of the time law 
% 2) i have only one target link that im controlling so i have only one reference
% 3) the parameter are all positives and are between 0 and 1

function [run_flag,performance,action]=StickBreaking4MonotoneTimeLaw(obj,action)

    simulator         = obj.input_4_run{1}; % rbt or v-rep
    if(strcmp(simulator,'icub_matlab'))
        
        controller = obj.input_4_run{4}; % structure that contains every information about the specific instance of the problem
        number_of_param_time_law = controller.references.n_of_parameter_per_regressor{1,1}(1);
        
        %% check on the action string
        if( sum(action(1:number_of_param_time_law)<-1) || sum(action(1:number_of_param_time_law)>1) )
            disp('the action parameters set for the time law is malformed (absolute value bigger than 1) ')
            error('malformed action')
        end
        
        if(sum(action(1:number_of_param_time_law))<0)
            disp('value of the action sequence for the time law should be between zero and one for this preprocessor unit')
            warning('fix time law action sequence for non positive values');
        end
        
        length = 1;
        for i = 1:number_of_param_time_law
            cut = ( length*abs(action(i)) );
            length = length - cut;
            if(i == 1)
                action(i) = cut;
            else
                action(i) = action(i-1) + cut;
            end
        end
        run_flag = true;
        performance = [];
    else
        disp('this preprocessro is not desgined to work with the rbt simulator for now')
        error('something wrong happened')
        run_flag = false;
        performance = -1;
    end
  
end