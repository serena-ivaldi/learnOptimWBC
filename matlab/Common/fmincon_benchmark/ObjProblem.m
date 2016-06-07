classdef ObjProblem < handle
    % The class ObjProblem help me to define an object store the data for
    % fmincon and the data i need to use it as black box constraints
    % It was build only to be used with the problem g07, g09, HB from
    % the benchmark
    %written by ugo chervet
    
    properties
        name; %name of the problem
        n; %size of the parameter vector
        J0; %optimal fitness
        LB; %lower boundaries
        UB; %%upper boundaries
        X0; %starting point
        constraints_values; %parameters of the black box constraints
        %computed_fitness_values;
        c; %constraints value (to be compliant with fmincon)
        ceq; %constraints value (to be compliant with fmincon)
    end
    
    methods
        % Constructor whicj=h define the object according to the name of
        % the problem you want to use from the benchmark
        function obj = ObjProblem(varargin)
            switch nargin
                case 1
                    obj.name = name;
                    switch name
                        case 'g07'
                            obj.n = 10;
                            obj.J0 = 24.3062091;
                            obj.LB = -10*ones(obj.n,1);
                            obj.UB = 10*ones(obj.n,1);
                        case 'g09'
                            obj.n = 7;
                            obj.J0 = 680.630057;
                            obj.LB = -10*ones(obj.n,1);
                            obj.UB = 10*ones(obj.n,1);
                        case 'HB'
                            obj.n = 5;
                            obj.J0 = -30665.539;
                            obj.LB = [78;33;27;27;27];
                            obj.UB = [102.33;45;45;45;45];
                        otherwise
                            disp('Unknown problem.')
                    end
                    obj.X0 = zeros(obj.n,1);
                    %obj.computed_fitness_values = [];
                    obj.c = [];
                    obj.ceq = [];
                case 5
                    obj.name = varargin{1};
                    obj.n = varargin{2};
                    obj.J0 = varargin{3};
                    obj.LB = varargin{4};
                    obj.UB = varargin{5};
                    obj.X0 = zeros(obj.n,1);
                    obj.c = [];
                    obj.ceq = [];
                otherwise
                    disp('ObjProblem invalid number of arguments');
            end
        end
        
        %generation of a random starting point inside the limit boundaries
        function randStartPoint(obj)
            obj.X0=zeros(obj.n,1);
            for i=1:length(obj.LB)
                obj.X0(i) = ( obj.UB(i) -  obj.LB(i))*rand() +  obj.LB(i);
            end
        end
        
        % compute the fitsness value (and is suppose to store each
        % computation in the vector computed_fitness_values)
        function [fit,obj] = computFit(obj,x)
            switch obj.name
                case 'g07'
                    fit = ( x(1)^(2) + x(2)^(2) + x(1)*x(2) -14*x(1) -16*x(2) +(x(3) - 10)^(2)...
                        + 4*(x(4)-5)^(2) + (x(5)-3)^2 + 2*(x(6)-1)^2 + 5*x(7)^(2)...
                        + 7*(x(8)-11)^(2) + 2*(x(9)-10)^(2) + (x(10) - 7)^(2) + 45);
                case 'g09'
                    fit = ( (x(1)-10)^(2) + 5*(x(2) - 12 )^(2) + x(3)^(4) +3*(x(4)-11)^(2)...
                        +10*x(5)^(6) +7*x(6)^(2) +x(7)^(4) -4*x(6)*x(7) -10*x(6) -8*x(7) );
                case 'HB'
                    fit = (5.3578547*x(3)^(2) + 0.8356891*x(1)*x(5) + 37.293239*x(1) -40792.141 );
                otherwise
                    disp('Unknown problem.')
            end
        end
        
        % fontion to be compliante with fmincon
        function [c, ceq] = contraintesFactice(obj,x)
            obj.computConstrViol(x);
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
                    c1 = g07Constr1(input,obj.constraints_values);
                    c2 = g07Constr2(input,obj.constraints_values);
                    c3 = g07Constr3(input,obj.constraints_values);
                    c4 = g07Constr4(input,obj.constraints_values);
                    c5 = g07Constr5(input,obj.constraints_values);
                    c6 = g07Constr6(input,obj.constraints_values);
                    c7 = g07Constr7(input,obj.constraints_values);
                    c8 = g07Constr8(input,obj.constraints_values);
                    obj.c = [c1; c2; c3; c4; c5; c6; c7; c8];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                case 'g09'
                    % Nonlinear inequality constraints
                    c1 = g09Constr1(input,obj.constraints_values);
                    c2 = g09Constr2(input,obj.constraints_values);
                    c3 = g09Constr3(input,obj.constraints_values);
                    c4 = g09Constr4(input,obj.constraints_values);
                    obj.c = [c1; c2; c3; c4];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                case 'HB'
                    % Nonlinear inequality constraints
                    c1 = HBConstr1(input,obj.constraints_values);
                    c2 = HBConstr2(input,obj.constraints_values);
                    c3 = HBConstr3(input,obj.constraints_values);
                    c4 = HBConstr4(input,obj.constraints_values);
                    c5 = HBConstr5(input,obj.constraints_values);
                    c6 = HBConstr6(input,obj.constraints_values);
                    obj.c = [c1; c2; c3; c4; c5; c6];
                    % Nonlinear equality constraints
                    obj.ceq = [];
                    
                otherwise
                    disp('Unknown problem.')
            end
        end
    end
end

