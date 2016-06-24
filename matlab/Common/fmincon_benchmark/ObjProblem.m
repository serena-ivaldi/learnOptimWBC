classdef ObjProblem < handle
    % The class ObjProblem help me to define an object store the data for
    % fmincon and the data i need to use it as black box constraints
    % It was build only to be used with the problem g07, g09, HB from
    % the benchmark
    %written by ugo chervet
    
    properties
        name                    % name of the problem
        n_search_space          % size of the parameter vector
        J0                      % optimal fitness (for metric)
        LB                      % lower bound
        UB                      % upper bound
        X0                      % starting point
        %current_point_for_inspection % for DEBUG
        c                       % constraints value (to be compliant with fmincon)
        ceq                     % constraints value (to be compliant with fmincon)
        
        penalty_handling        % object to handle constraints/penalties inside the optimization routine
        constraints             % flag that activates or deactivates the constraints handling (true: constraints active, false: constraints not active)
        run_function            % function called in run specific for each optimization problem
        fitness                 % fitness function handle
        fitness_result          % in this vector i save the value of the fitness function
        clean_function          % function called to do some stuff after using the run function (optional could be empty)
        input_4_run             % this variable is a cell array that contains the data that are needed to execute the run function
        
        b                       % radius of the topologic ball for the metric3 (express as a % of difference from the optimal)
        
        algorithm_selector      % flag to select the optimization algorithm  
        
        
    end
    
    methods
        % Constructor which define the object according to the problem
        %   ObjProblem(constr,learn_procedure,run_function,fitness,clean_function,input_4_run)
        function obj = ObjProblem(n_search_space,boundaries,constr,algorithm_selector,run_function,fitness,clean_function,input_4_run)                   
            obj.penalty_handling = constr;
            obj.algorithm_selector = algorithm_selector;
            obj.run_function =run_function;
            obj.fitness = fitness;
            obj.clean_function = clean_function;
            obj.input_4_run = input_4_run;

            obj.n_search_space = n_search_space;
            obj.J0 = 0;
            if(iscell(boundaries))
                obj.LB = boundaries{1};
                obj.UB = boundaries{2};
            elseif(isvector(boundaries))
                obj.LB = ones(1,n_search_space).*boundaries(1,1);
                obj.UB = ones(1,n_search_space).*boundaries(1,2);
            else
                error('something wrong with cmaes_value_range')
            end
            obj.b = 2.5; %ie +/- 2,5% from the steady value
        end
        
        function input_vec = CreateInputFromParameters(obj,parameters) 
            input_vec = repmat({parameters},1,obj.penalty_handling.n_constraint);  
       end
        
        % This function compute the fitness value
        function fitvalue = computFit(obj,input)
                try
                    disp('i am in computFit')
                    [output]=obj.run(input);
                    fitvalue =  - obj.fitness(obj,output); %minus because we are maximizing and fmincon only minimize
                    if isnan(fitvalue)
                        disp('Fitness is equal to NaN.') %i can put a breakpoint here to understand how in heaven it's possible that i sometimes get effort = NaN
                        fitvalue = 1;
                    end
                catch err
                    disp('i am in computFit error side')
                    fitvalue = 1; %penalty if the computation of the fitness failed
                end 
        end
        
        % This function has to give back something that let me compute the fitness function
        function [output]=run(obj,parameters)
            %disp('im in run')
            [output]=feval(obj.run_function,obj,parameters);
        end
        
        % Function to be compliante with how fmincon handle constraints
        function [c, ceq] = computeConstr(obj,input)
            try
                obj.computConstrViol(input);
                c = obj.c;
                ceq = obj.ceq;
            catch err
                disp('computeConstr failed');
            end
        end
        
        % Compute the constraints violation
        % input is a fake value
        function obj = computConstrViol(obj,input)                    
            % each time i have to compute this 
            try
                disp('i am in computConstrViol fitness computation')
                [output]=obj.run(input);
                obj.fitness(obj,output); %minus because we are maximizing and fmincon only minimize
            catch err
                disp('i am in computConstrViol error side fitness computation ')
                fitvalue = 1; %penalty if the computation of the fitness failed
            end 
            c_index = -1; %i'm just considering one candidate (cf. FixPenalty class)
            obj.penalty_handling.ComputeConstraintsViolation(c_index);
            c = [];
            ceq = [];
            for i=1:length(obj.penalty_handling.constraints_type)
                if  obj.penalty_handling.constraints_type(i)
                    c = [c; obj.penalty_handling.penalties(1,i)];
                else
                    ceq = [ceq; obj.penalty_handling.penalties(1,i)];
                end
            end   
            obj.c = c;
            obj.ceq = ceq;
        end
       
        % Do the optimization (only with fmincon for now )
        % the signature is exactly the same as optimization.CMAES
        % will need a flag later to switch between fmincon and ipopt
        function [fitness,bestAction] = minimize(obj,starting_point,MaxFunEvals,threshold)   
            switch obj.algorithm_selector
                case 'fmincon'    
                    options = optimoptions('fmincon','OutputFcn',@obj.outfun,'Display','iter','Algorithm','sqp');
                    options.MaxFunEvals = MaxFunEvals;
                    options.TolX = 1e-15; % the step size tolerance
                    %options.UseParallel = true;
                    fminconPb.options = options;
                    fminconPb.solver = 'fmincon';
                    fminconPb.X0 = starting_point;
                    fminconPb.objective = @obj.computFit;
                    obj.fitness_result = [];
                    fminconPb.LB = obj.LB;
                    fminconPb.UB = obj.UB;
                    fminconPb.NONLCON = @obj.computeConstr;
                    %tic
                    [X,FVAL] = fmincon(fminconPb);
                    %m4 = toc;
                otherwise
                    fprintf('Error, no such method is found! \n')
            end
%             m1 = obj.J0 - FVAL; %metric 1 = fitness error
%             
%             obj.computConstrViol(X); %metric constr violation
%             m2 = sum(abs((obj.c > 0).*obj.c)) + sum(abs((obj.ceq ~= 0).*obj.ceq));
%             
%             m3 = obj.IndentifySteadyState(obj.fitness_result,threshold); %metric 3 = # of steps to steady value
            fitness = obj.fitness_result;
            bestAction.parameters = X;
            bestAction.performance = FVAL;
        end
        
        
        % My output function which allow me to store the computed fitness
        % for each step chosen point
        function stop = outfun(obj, x, optimValues, state)
            stop = false;
            if strcmp(state, 'iter')
                obj.fitness_result = [obj.fitness_result; optimValues.fval];
            end
        end
        
        
        % Used to determine the number needed to reach stability
        function zzz = IndentifySteadyState(obj,vector,tresh)
            steady_value = vector(end);
            for zzz = 1:length(vector)
                if(abs(steady_value-vector(zzz))<(tresh/100*steady_value))
                    break
                end
            end
        end
        
    end
end

