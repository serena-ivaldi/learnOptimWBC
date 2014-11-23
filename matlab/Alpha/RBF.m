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
        sample                 % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
    end


    
    methods
        function obj = RBF(time_struct,n_of_basis,redundancy)
            obj.time_struct = time_struct;
            obj.n_of_basis = n_of_basis;
            obj.redundancy = redundancy;
            obj.BuildRBF();
            
        end

        function BuildRBF(obj)
            t = sym('t');
            theta = sym('theta',[obj.n_of_basis,1]);
            T = obj.time_struct.tf;
            
            % this value of sigma produces a 15% of overlapping between two 
            % consecutive gaussian with redundancy = 3;
            sigma = (T)/((obj.n_of_basis-1)*obj.redundancy);
            cof = 2*sigma^2;
            
            for i=0:(obj.n_of_basis-1)
                
                phi(i+1) = exp(-(t-(i*T)/(obj.n_of_basis-1))^2/cof);
                obj.basis_functions{i+1} = matlabFunction(phi(i+1));
            end

            
            rbf = phi*theta;
            rbf = matlabFunction(rbf,'vars', {t,theta});
            obj.func = rbf;
            
        end
        
        
        % return the value of the alpha function at time t
        % using normalized value
        
        %interface function
        function result = GetValue(obj,t)
            [~,ind] = min(abs(obj.sample.time-t));
            result = obj.sample.normvalues(ind);
        end
        
        %interface function 
        function ComputeNumValue(obj,theta)
            
            time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
            i=1;
            for t = time
                obj.sample.values(i) = feval(obj.func,t,theta); 
                i=i+1;
            end
            obj.sample.time = time;
            % normalize result between zero and one
            minimum = min(obj.sample.values);
            maximum = max(obj.sample.values);
            
            obj.sample.normvalues=(obj.sample.values - min(obj.sample.values))/(maximum -minimum);
            
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
        
        function RBFs = BuildCellArray(n_subchain,n_task,time_struct,n_of_basis,redundancy,theta)
            
            for i=1:n_subchain
               for j=1:n_task
                RBFs{i,j} = RBF(time_struct,n_of_basis,redundancy);
                RBFs{i,j}.ComputeNumValue(theta(:,i));
               end
            end
            
        end
    end
    
end



