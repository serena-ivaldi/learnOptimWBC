classdef ObjProblem < handle
    % The class ObjProblem help me to define an object store the data for
    % fmincon and the data i need to use it as black box constraints
    % It was build only to be used with the problem g07, g09, HB from
    % the benchmark
    %written by ugo chervet
    
    properties
        name                % name of the problem
        n_search_space      % size of the parameter vector
        J0                  % optimal fitness
        LB                  % lower boundaries
        UB                  % upper boundaries
        X0                  % starting point
        constraints_parameters  % parameters of the black box constraints
        c                   % constraints value (to be compliant with fmincon)
        ceq                 % constraints value (to be compliant with fmincon)
        
        penalty_handling % object to handle constraints/penalties inside the optimization routine
        learn_procedure  % string that specify which learning method im going to use
        constraints      % flag that activates or deactivates the constraints handling (true: constraints active, false: constraints not active)
        run_function     % function called in run specific for each optimization problem
        fitness          % fitness function handle
        fitness_result   % in this vector i save the value of the fitness function
        clean_function   % function called to do some stuff after using the run function (optional could be empty)
        input_4_run      % this variable is a cell array that contains the data that are needed to execute the run function
        
        b  %radius of the topologic ball for the metric3 (express as a % of difference from the optimal
    end
    
    methods
        % Constructor which define the object according to the problem
        %either you just passe in the name (g07,g09 or HB) or the same
        %signature than Optimization.Instance
        %   ObjProblem(name)        or
        %   ObjProblem(constr,learn_procedure,run_function,fitness,clean_function,input_4_run)
        function obj = ObjProblem(varargin)
            switch nargin
                case 1
                    obj.name = varargin{1};
                    obj.run_function = @obj.EmptyPreprocessing;
                    switch obj.name
                        case 'g07'
                            obj.n_search_space = 10;
                            obj.J0 = 24.3062091;
                            obj.LB = -10*ones(obj.n_search_space,1);
                            obj.UB = 10*ones(obj.n_search_space,1);
                            obj.fitness = @g07Test;
                        case 'g09'
                            obj.n_search_space = 7;
                            obj.J0 = 680.630057;
                            obj.LB = -10*ones(obj.n_search_space,1);
                            obj.UB = 10*ones(obj.n_search_space,1);
                            obj.fitness = @g09Test;
                        case 'HB'
                            obj.n_search_space = 5;
                            obj.J0 = -30665.539;
                            obj.LB = [78;33;27;27;27];
                            obj.UB = [102.33;45;45;45;45];
                            obj.fitness = @HBTest;
                        otherwise
                            disp('Unknown problem.')
                    end
                case 6
                    %the case where we are using the same structure as Optimization.Instance
                    %see Optimization.Instance for more details
                    obj.name = 'custom';
                    %obj.penalty_handling = varargin{1};
                    %obj.run_function = varargin{3};
                    %obj.fitness = varargin{4};
                    
                    obj.penalty_handling = varargin{1};
                    obj.learn_procedure = varargin{2}; % useless in this object
                    obj.run_function = varargin{3};
                    obj.fitness = varargin{4};
                    obj.clean_function = varargin{5};
                    obj.input_4_run = varargin{6};
                    
                    obj.n_search_space = [];
                    obj.J0 = 0;
                    obj.LB = [];
                    obj.UB = [];
                otherwise
                    disp('ObjProblem invalid number of arguments');
            end
            obj.c = [];
            obj.ceq = [];
            obj.X0 = zeros(obj.n_search_space,1);
            obj.b = 50;
        end
        
        %generation of a random starting point inside the limit boundaries
        function randStartPoint(obj)
            obj.X0=zeros(obj.n_search_space,1);
            for i=1:length(obj.LB)
                obj.X0(i) = ( obj.UB(i) -  obj.LB(i))*rand() +  obj.LB(i);
            end
        end
        
        % compute the fitness value (and is suppose to store each
        % computation in the vector computed_fitness_values)
        function fitvalue = computFit(obj,input)
            %             if ~strcmp(obj.name, 'custom')
            %                 fitvalue = obj.fitness(input);
            %             else
            [output]=obj.run(input);
            fitvalue = obj.fitness(obj,output);
            %end
        end
        
        % this function has to give back something that let me compute the
        % fitness function for that sample
        function [output]=run(obj,parameters)
            %disp('im in run')   
            [output]=feval(obj.run_function,obj,parameters);
       end
        
        % function to be compliante with fmincon
        function [c, ceq] = contraintesFactice(obj,input)
            obj.computConstrViol(input);
            c = obj.c;
            ceq = obj.ceq;
        end
        
        % compute the constraints violation
        % this is my black box constraints
        function obj = computConstrViol(obj,input)
            %display('im in computConstrViol');
            switch obj.name
                case 'g07'
                    % Nonlinear inequality constraints
                    c1 = g07Constr1(input,obj.constraints_parameters);
                    c2 = g07Constr2(input,obj.constraints_parameters);
                    c3 = g07Constr3(input,obj.constraints_parameters);
                    c4 = g07Constr4(input,obj.constraints_parameters);
                    c5 = g07Constr5(input,obj.constraints_parameters);
                    c6 = g07Constr6(input,obj.constraints_parameters);
                    c7 = g07Constr7(input,obj.constraints_parameters);
                    c8 = g07Constr8(input,obj.constraints_parameters);
                    obj.c = [c1; c2; c3; c4; c5; c6; c7; c8];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                case 'g09'
                    % Nonlinear inequality constraints
                    c1 = g09Constr1(input,obj.constraints_parameters);
                    c2 = g09Constr2(input,obj.constraints_parameters);
                    c3 = g09Constr3(input,obj.constraints_parameters);
                    c4 = g09Constr4(input,obj.constraints_parameters);
                    obj.c = [c1; c2; c3; c4];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                case 'HB'
                    % Nonlinear inequality constraints
                    c1 = HBConstr1(input,obj.constraints_parameters);
                    c2 = HBConstr2(input,obj.constraints_parameters);
                    c3 = HBConstr3(input,obj.constraints_parameters);
                    c4 = HBConstr4(input,obj.constraints_parameters);
                    c5 = HBConstr5(input,obj.constraints_parameters);
                    c6 = HBConstr6(input,obj.constraints_parameters);
                    obj.c = [c1; c2; c3; c4; c5; c6];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                case 'custom'
                    for i = 1:obj.penalty_handling.n_constraint
                        g = str2func(obj.penalty_handling.constraints_functions{i});
                        obj.c = [obj.c; g(input)];
                    end
                    obj.ceq = [];
                otherwise
                    disp('Unknown problem.')
            end
        end
        
        
        %will need a flag later to switch between fmincon and ipopt
        function [m1,m2,m3,m4] = minimize(obj,num_of_param,start_action,~,~,boundaries)
            options = optimoptions('fmincon','OutputFcn',@obj.outfun,'Display','iter','Algorithm','sqp');
            options.MaxFunEvals = 500;
            fminconPb.options = options;
            fminconPb.solver = 'fmincon';
            fminconPb.X0 = start_action;
            fminconPb.objective = @obj.computFit;
            obj.fitness_result = [];
            
            if(iscell(boundaries))
                fminconPb.LB = boundaries{1};
                fminconPb.UB = boundaries{2};
            elseif(isvector(boundaries))
                fminconPb.LB = ones(1,num_of_param).*boundaries(1,1);
                fminconPb.UB = ones(1,num_of_param).*boundaries(1,2);
            else
                error('something wrong with cmaes_value_range')
            end
            
            fminconPb.NONLCON = @obj.contraintesFactice;
            
            tic
            [X,FVAL,exitflag,output] = fmincon(fminconPb);
            m4 =  toc;
            
            m1 = obj.J0 - FVAL; %metric 1 = fitness error
            
            [obj.c, obj.ceq] = obj.contraintesFactice(X); %metric constr violation
            m2 = sum(abs((obj.c > 0).*obj.c)) + sum(abs((obj.ceq ~= 0).*obj.ceq));
            
            m3 = obj.IndentifySteadyState(obj.fitness_result,obj.b); %metric 3 = # of steps to steady value
        end
        
        function stop = outfun(obj, x, optimValues, state)
            stop = false;
            if strcmp(state, 'iter')
                % Concatenate current objective function value with history
                obj.fitness_result = [obj.fitness_result; optimValues.fval];
            end
        end
        
        %find the index after which the values of the vector stay inside (last
        %value of the vecto - treshold) and  (last value of the vecto + treshold)
        function zzz = IndentifySteadyState(obj,vector,tresh)
            steady_value = vector(end);
            for zzz = 1:length(vector)
                if(abs(steady_value-vector(zzz))<tresh)
                    break
                end
            end
        end
        
        function [output] = EmptyPreprocessing(~,obj,parameters)            
            output = parameters;            
        end
        
        
    end
    
    
end

