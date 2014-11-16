classdef  DMP < AbstractAlpha
    
% gaussian radial basis function
    
    
    properties
        time_struct            % struct with time_struct.ti time_struct.tf time_struct.step
        n_of_basis             % number of functions that compose our RBF
        redundancy             % parameter that control the level of overlapping of the function
        basis_functions        % cell array of basis function
        kp                     % parameter of pd (position)
        kd                     % parameter of pd (velocity)
        Pd                     % final desired position 
        Vd                     % final desired velocity 
        alpha_z                % smoothing factor of non linear force
        func                   % handle to dmp(theta,t) 
        sample                 % value for a specific set of theta and sampling time (sample.time sample.values sample.normvalues)
    end


    
    methods
        function obj = DMP(time_struct,n_of_basis,redundancy,kp,kd,Pd,Vd,alpha_z)
            obj.time_struct = time_struct;
            obj.n_of_basis = n_of_basis;
            obj.redundancy = redundancy;
            obj.BuildRBF();
            
        end

        function BuildDMP(obj)
            t  = sym('t');
            pd = sym('pd');
            pt = sym('pt');
            vd = sym('vd');
            vt = sym('vt');
            
            theta = sym('theta',[obj.n_of_basis,1]);
            T = obj.time_struct.tf;
            
            z = exp(-obj.alpha_z*t);
            
            % this value of sigma produces a 15% of overlapping between two 
            % consecutive gaussian with redundancy = 3;
            sigma = (T)/((obj.n_of_basis-1)*obj.redundancy);
            cof = 2*sigma^2;
            
            for i=0:(obj.n_of_basis-1)
                
                psi(i+1) = (exp(-(t-(i*T)/(obj.n_of_basis-1))^2/cof))*z;
                obj.basis_functions{i+1} = matlabFunction(psi(i+1));
            end

            f = psi*theta;
            dmp = obj.kd*obj.kp*(pd-pt) + obj.kd*(vd-vt) + f;
            obj.func = matlabFunction(dmp,'vars', {t,pd,pt,vd,vt,theta});
                        
        end
        
        
        % return the value of the alpha function at time t
        % using normalized value
        
        %interface function
        function result = GetValue(obj,t)
            [~,ind] = min(abs(obj.sample.time-t));
            result = obj.sample.normvalues(ind);
        end
        
        
        %interface function (TO FIX) 
        function ComputeNumValue(obj,p_init,v_init,theta)
            
            time = obj.time_struct.ti:obj.time_struct.step:obj.time_struct.tf;
            i=1;
            pt = p_init;
            vt = v_init;
            for t = time
                
                at = feval(obj.func,t,obj.pd,obj.pt,vd,vt,theta); 
                
                %new position
                pt = pt + vt*0.2 + at*0.2^2/2;
                %new velocity
                vt = vt + at*0.2;
                
                obj.sample.values(i) = pt;
                i=i+1;
                
            end
            obj.sample.time = time;
            
            % normalize result between zero and one
            minimum = min(obj.sample.values);
            maximum = max(obj.sample.values);
            
            obj.sample.normvalues=(obj.sample.values - min(obj.sample.values))/(maximum - minimum);
            
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
        
        % this function modifies the value of "sample" in timestruct 
        function [p_init,v_init,theta] = TrainByDraw(obj,number_of_pivot,step)
           
           [p ,pd ,pdd] = RecordTrajectory(number_of_pivot,step)
           
           
           
        end
        
        
        
    end
    
    methods (Static)
        
        function DMPs = BuildCellArray(n_of_task,time_struct,n_of_basis,redundancy,kp,kd,Pd,Vd,alpha_z,train,number_of_pivot,step)
            
            for i=1:n_of_task
                DMPs{i} = DMP(time_struct,n_of_basis,redundancy,kp,kd,Pd,Vd,alpha_z);
                if(train)
                     [p_init,v_init,theta] = DMPs{i}.TrainByDraw(number_of_pivot,step);
                     DMPs{i}.ComputeNumValue(obj,p_init,v_init,theta);  
                end
            end
            
        end
    end
    
   




   
end