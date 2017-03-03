%% in order to maximize with a minimization a have to put a minus in front
%% of all the surrogates that are natively built as a maximization problem

%% i returnt the x back because sometimes it happens that i rototrasl the x inside ther function
%% it happens in custom for the boost move
function [ret, x] = Surrogate(self, x,varargin)
        if strcmp(self.kind,'ucb')
            [ret, x] =  self.ucb(x);
            ret = - ret;
            
        elseif strcmp(self.kind,'ei')
            [ret, x] =  self.ei(x);
            ret = - ret;
        
        elseif strcmp(self.kind,'poi')
            [ret, x] =  self.poi(x);
            ret = - ret;
        
        elseif strcmp(self.kind,'eci')
            %self.min_or_max = 'max';
            [ret, x] =  self.eci(x,varargin{1});
            ret = - ret;
        
        elseif strcmp(self.kind,'ecv')
            %self.min_or_max = 'max';
            [ret, x] =  self.ecv(x);
            ret = - ret;
        
%         if strcmp(self.kind,'ec')
%             [ret, x] =  self.ec(x);
%             ret = - ret;
%         end
%         if(strcmp(self.kind,'ecm'))
%             % no need to invert the sign i have to minize this funcition
%             [ret, x] =  self.ecm(x);
%         end
        elseif(strcmp(self.kind,'ucb_constr'))
            %self.min_or_max = 'min';
            % no need to invert the sign i have to minize this funcition
            [ret, x] = self.ucb_constr(x);
        
        elseif strcmp(self.kind,'pcs_constr')
            %self.min_or_max = 'max';
            [ret, x] =  self.pcs_constr(x);
            ret = - ret;
        
        elseif strcmp(self.kind,'mcd_constr')
            %self.min_or_max = 'max';
            [ret, x] =  self.mcd_constr(x,varargin{1});
            ret = - ret;
        elseif strcmp(self.kind,'cucb')
            %self.min_or_max = 'max';
            [ret, x] =  self.cucb(x);
            ret = - ret;    
        
        elseif strcmp(self.kind,'custom');
            %% TOFIX this is not true (self.min_or_max = 'max';) all the times 
            %% it depends on which function im gonna
            %% combine to obtain a custom
            %self.min_or_max = 'max';
            [ret, x] =  varargin{1}(x);
            % i invert the value because i want to maximize but matlab only
            % minimize
            if(strcmp((self.min_or_max),'max'))
                ret = - ret;
            end
              
        end
end






    