%% in order to maximize with a minimization a have to put a minus in front
%% of all the surrogates that are natively built as a maximization problem

%% i returnt the x back because sometimes it happens that i rototrasl the x inside ther function
%% it happens in custom for the boost move
function [ret, x] = Surrogate(self, x, kappa, xi,varargin)
        if strcmp(self.kind,'ucb')
            [ret, x] =  self.ucb(x, kappa);
            ret = - ret;
        end
        if strcmp(self.kind,'ei')
            [ret, x] =  self.ei(x, xi);
            ret = - ret;
        end
        if strcmp(self.kind,'poi')
            [ret, x] =  self.poi(x, xi);
            ret = - ret;
        end
        if strcmp(self.kind,'eci')
            [ret, x] =  self.eci(x, xi);
            ret = - ret;
        end
        if strcmp(self.kind,'ecv')
            [ret, x] =  self.ecv(x);
            ret = - ret;
        end
        if strcmp(self.kind,'custom');
            [ret, x] =  varargin{1}(x,xi);
            ret = - ret;
        end
end






    