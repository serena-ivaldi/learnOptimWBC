% gaussian radial basis function
% parameters have to be  column vectors because of the way the alpha function
% is built before (from sym to function)

classdef  RBF < AbstractAlpha
    
    
    
    properties
        time_struct            % struct with time_struct.ti time_struct.tf time_struct.step
        n_of_basis             % number of functions that compose our RBF
        redundancy             % parameter that control the level of overlapping of the function
        basis_functions        % cell array of basis function
        func                   % handle to rbf(theta,t) 
        param                  % current parameters set of the alpha function
        sample                 % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
        range                  % interval of values admitable for the weight of the basis function defined as [min max] with range i control the shape of the sigmoid exp(rbf - max(range)/2)/(1-exp(rbf - max(range)/2))
        precomp_sample         % if true i precompute the value of the rbf 
    end


    
    methods
        function obj = RBF(time_struct,n_of_basis,redundancy,range,precomp_sample,numeric_theta)
            obj.time_struct    = time_struct;
            obj.n_of_basis     = n_of_basis;
            obj.redundancy     = redundancy;
            obj.range          = range;
            obj.precomp_sample = precomp_sample;
            obj.param          = numeric_theta;
            obj.BuildRBF(numeric_theta);
            
        end

        function BuildRBF(obj,numeric_theta)
            t = sym('t');
            theta = sym('theta',[obj.n_of_basis,1]);
            T = obj.time_struct.tf;
            
            % this value of sigma produces a 15% of overlapping between two 
            % consecutive gaussian with redundancy = 3;
            sigma = (T)/((obj.n_of_basis-1)*obj.redundancy);
            cof = 2*sigma^2;
            sumphi = 0;
            for i=0:(obj.n_of_basis-1)
                
                phi(i+1) = exp(-(t-(i*T)/(obj.n_of_basis-1))^2/cof);
                obj.basis_functions{i+1} = matlabFunction(phi(i+1));
                sumphi = sumphi + phi(i+1);
            end

            c=obj.range(1,2)/2;
            
            rbf = (phi*theta)/sumphi;
            rbf =  (exp(rbf-c)) / (1 + exp(rbf-c));
            rbf = matlabFunction(rbf,'vars', {t,theta});
            obj.func = rbf;
            
            if(obj.precomp_sample)
                time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
                i=1;
                for t = time
                    obj.sample.values(i) = feval(obj.func,t,numeric_theta); 
                    i=i+1;
                end
                obj.sample.time = time;
            end
            
            
        end
        
        
        % return the value of the alpha function at time 
        
        %interface function
        function result = GetValue(obj,t)
            if(obj.precomp_sample)
                [~,ind] = min(abs(obj.sample.time-t));
                result = obj.sample.values(ind);
            else
               %control if tau is not a row vector 
               if(isrow(obj.param))
                  result = feval(obj.func,t,obj.param');
               else
                  result = feval(obj.func,t,obj.param); 
               end
               
            end
            
        end
        
        %in this function i update the param for every function
        % or precompute the value of rbf only if precomp_sample=true
        function ComputeNumValue(obj,theta)
            
            if(obj.precomp_sample)
                time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
                i=1;
                for t = time
                    obj.sample.values(i) = feval(obj.func,t,numeric_theta); 
                    i=i+1;
                end
                obj.sample.time = time;
            else
               obj.param = theta; 
            end    
        
        end
        
        %interface function
        function n=GetParamNum(obj)
            n=obj.n_of_basis;
        end
             
        
        function PlotBasisFunction(obj)
            
            time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
            
            for j=1:obj.n_of_basis
                i=1;
                for t = time
                    results(j,i) = feval(obj.basis_functions{j},t); 
                    i=i+1;
                end
            end
            
            hold all;
            for j=1:obj.n_of_basis
                plot(results(j,:))
            end
            
        end
    end
    
    methods (Static)
        
        function RBFs = BuildCellArray(n_subchain,n_task,time_struct,n_of_basis,redundancy,range,precomp_sample,theta)
            
            if size(theta,2) > n_of_basis;
                index = 1;
                for i=1:n_subchain
                   for j=1:n_task
                        cur_theta = theta(index:index+n_of_basis - 1); 
                        RBFs{i,j} = RBF(time_struct,n_of_basis,redundancy,range,precomp_sample,cur_theta);
                        index = index+n_of_basis;
                   end
                end 
            else
                for i=1:n_subchain
                   for j=1:n_task
                        RBFs{i,j} = RBF(time_struct,n_of_basis,redundancy,range,precomp_sample,theta);
                   end
                end    
            end    
            
        end
        
    end
    
end



